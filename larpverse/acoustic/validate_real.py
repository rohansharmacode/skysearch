"""Real-data validation of the acoustic front end (VAD, YAMNet grouping, Whisper, agent scorer).

Subcommands (run from the repo root, heavy ones via scripts/heavy.sh):
  index   list DroneAudioSet parquet shards and their rows by HTTP range requests (footers only)
  fetch   download chosen DroneAudioSet rows (stream one shard at a time, keep WAVs, drop parquet)
          and ESC-50 clips (single files from GitHub)  -> data/audio_real/
  run     run acoustic.models + acoustic.agent.fallback_judge on the clips -> results JSON
  plot    figures + summary tables from the results JSON (needs matplotlib: use .venv)

No pyarrow in .venv-audio: a minimal Parquet reader (Thrift compact footer, v1/v2 data pages,
RLE/bit-packed levels, PLAIN / RLE_DICTIONARY doubles, SNAPPY via the system libsnappy) lives
below. It handles exactly the DroneAudioSet layout and asserts on anything else.
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import io
import json
import os
import re
import struct
import sys
import time
import urllib.request

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "data", "audio_real")
DAS_REPO = "ahlab-drone-project/DroneAudioSet"
DAS_URL = "https://huggingface.co/datasets/" + DAS_REPO + "/resolve/main/"
UA = {"User-Agent": "larpverse-validate/0.1"}


# =========================================================================== HTTP
def http_get(url, start=None, end=None, tries=4):
    h = dict(UA)
    if start is not None:
        h["Range"] = f"bytes={start}-{'' if end is None else end}"
    for k in range(tries):
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=h), timeout=60) as r:
                return r.read()
        except Exception as e:                       # noqa: BLE001 - retry any network error
            if k == tries - 1:
                raise
            print(f"  retry {url[-60:]}: {e}", file=sys.stderr)
            time.sleep(2 + 3 * k)


# =========================================================================== Thrift compact
class _T:
    def __init__(self, b, pos=0):
        self.b, self.p = b, pos

    def byte(self):
        v = self.b[self.p]
        self.p += 1
        return v

    def varint(self):
        s = v = 0
        while True:
            c = self.byte()
            v |= (c & 0x7F) << s
            if not c & 0x80:
                return v
            s += 7

    def zz(self):
        v = self.varint()
        return (v >> 1) ^ -(v & 1)

    def val(self, t):
        if t in (1, 2):
            return t == 1
        if t == 3:
            return self.byte()
        if t in (4, 5, 6):
            return self.zz()
        if t == 7:
            v = struct.unpack_from("<d", self.b, self.p)[0]
            self.p += 8
            return v
        if t == 8:
            n = self.varint()
            v = bytes(self.b[self.p:self.p + n])
            self.p += n
            return v
        if t in (9, 10):
            h = self.byte()
            n, et = h >> 4, h & 15
            if n == 15:
                n = self.varint()
            if et in (1, 2):                    # bools inside lists are one byte each
                return [self.byte() == 1 for _ in range(n)]
            return [self.val(et) for _ in range(n)]
        if t == 11:
            n = self.varint()
            if not n:
                return {}
            kv = self.byte()
            return {self.val(kv >> 4): self.val(kv & 15) for _ in range(n)}
        if t == 12:
            return self.struct()
        raise ValueError(f"thrift type {t}")

    def struct(self):
        out, fid = {}, 0
        while True:
            h = self.byte()
            if h == 0:
                return out
            d, t = h >> 4, h & 15
            fid = fid + d if d else self.zz()
            out[fid] = self.val(t)


# =========================================================================== Parquet (minimal)
_snappy = None


def snappy_uncompress(b: bytes) -> bytes:
    global _snappy
    if _snappy is None:
        _snappy = ctypes.CDLL(ctypes.util.find_library("snappy") or "libsnappy.so.1")
    n = ctypes.c_size_t()
    assert _snappy.snappy_uncompressed_length(b, ctypes.c_size_t(len(b)), ctypes.byref(n)) == 0
    out = ctypes.create_string_buffer(n.value)
    assert _snappy.snappy_uncompress(b, ctypes.c_size_t(len(b)), out, ctypes.byref(n)) == 0
    return out.raw[: n.value]


def _decomp(codec, b):
    if codec == 0:
        return bytes(b)
    if codec == 1:
        return snappy_uncompress(bytes(b))
    raise ValueError(f"parquet codec {codec} not supported")


def rle_hybrid(b: bytes, bw: int, n: int) -> np.ndarray:
    """Parquet RLE / bit-packed hybrid -> n uint32 values."""
    out = np.empty(n, np.uint32)
    k, p, nb = 0, 0, (bw + 7) // 8
    t = _T(b)
    while k < n:
        t.p = p
        h = t.varint()
        p = t.p
        if h & 1:                                  # bit-packed groups of 8
            g = h >> 1
            raw = np.frombuffer(b, np.uint8, g * bw, p)
            p += g * bw
            bits = np.unpackbits(raw, bitorder="little").reshape(-1, bw)
            vals = (bits.astype(np.uint32) << np.arange(bw, dtype=np.uint32)).sum(axis=1)
            m = min(len(vals), n - k)
            out[k:k + m] = vals[:m]
            k += m
        else:
            r = h >> 1
            v = int.from_bytes(b[p:p + nb], "little") if nb else 0
            p += nb
            m = min(r, n - k)
            out[k:k + m] = v
            k += m
    return out


def read_footer(url=None, blob=None, size=None):
    """FileMetaData dict from a remote URL (two small range requests) or local bytes."""
    if blob is None:
        tail = http_get(url, max(0, size - 65536), size - 1)
    else:
        tail = blob[-65536:]
    assert tail[-4:] == b"PAR1"
    n = struct.unpack("<I", tail[-8:-4])[0]
    if n + 8 > len(tail):
        tail = http_get(url, size - n - 8, size - 1)
    return _T(tail[-8 - n:-8]).struct()


def columns(fm):
    """[(dotted path, ColumnMetaData dict)] for row group 0 (DroneAudioSet shards have one)."""
    assert len(fm[4]) == 1, "expected one row group"
    return [(".".join(p.decode() for p in cc[3][3]), cc[3]) for cc in fm[4][0][1]]


def chunk_range(md):
    start = md.get(11) or md[9]                       # dictionary page first when present
    return start, start + md[7]                       # total_compressed_size


def read_column(buf: bytes, md, max_rep: int, max_def: int):
    """Decode one whole (small) column chunk -> (values, rep levels, def levels)."""
    vals, reps, defs = [], [], []
    for v, r, d in iter_pages(_Seq(io.BytesIO(buf)), md, max_rep, max_def):
        vals.append(v)
        reps.append(r)
        defs.append(d)
    if md[1] == 5:
        values = np.concatenate(vals) if vals else np.zeros(0)
    else:
        values = [x for v in vals for x in v]
    return values, np.concatenate(reps), np.concatenate(defs)


class _Seq:
    """Sequential reader with peek over a file or HTTP response; counts bytes read."""

    def __init__(self, fh):
        self.fh, self.buf, self.n = fh, b"", 0

    def peek(self, n):
        while len(self.buf) < n:
            b = self.fh.read(max(n - len(self.buf), 1 << 16))
            if not b:
                break
            self.n += len(b)
            self.buf += b
        return self.buf[:n]

    def take(self, n):
        out = self.peek(n)
        self.buf = self.buf[n:]
        return out


def _plain(b, ptype, n):
    if ptype == 5:                                   # DOUBLE
        return np.frombuffer(b, "<f8", n).copy()
    if ptype == 2:                                   # INT64
        return np.frombuffer(b, "<i8", n).copy()
    if ptype == 6:                                   # BYTE_ARRAY
        out, p = [], 0
        for _ in range(n):
            ln = struct.unpack_from("<I", b, p)[0]
            out.append(bytes(b[p + 4:p + 4 + ln]))
            p += 4 + ln
        return out
    raise ValueError(f"physical type {ptype}")


def schema_levels(fm):
    """{dotted leaf path: (max_rep, max_def)} from the flat schema list."""
    out = {}
    sch = fm[2]

    def walk(i, path, rep, dfn):
        el = sch[i]
        name = el[4].decode()
        rt = el.get(3, 0)                            # 0 required, 1 optional, 2 repeated
        r = rep + (rt == 2)
        d = dfn + (rt != 0)
        nchild = el.get(5, 0)
        j = i + 1
        if nchild == 0:
            out[".".join(path + [name])] = (r, d)
            return j
        for _ in range(nchild):
            j = walk(j, path + [name], r, d)
        return j

    j = 1
    for _ in range(sch[0].get(5, 0)):
        j = walk(j, [], 0, 0)
    return out


def iter_pages(seq: _Seq, md, max_rep, max_def):
    """Stream one column chunk page by page from a sequential reader positioned at the chunk
    start: yields (values, rep, def)."""
    s0, end = chunk_range(md)
    left = end - s0
    dictionary = None
    while left > 0:
        t = _T(seq.peek(8192))
        ph = t.struct()
        seq.take(t.p)
        body = seq.take(ph[3])
        left -= t.p + ph[3]
        if ph[1] == 2:
            dictionary = _plain(_decomp(md[4], body), md[1], ph[7][1])
            continue
        yield _decode_page(ph, body, md, max_rep, max_def, dictionary)


def _decode_page(ph, body, md, max_rep, max_def, dictionary):
    ptype, codec = md[1], md[4]
    if ph[1] == 0:
        raw = _decomp(codec, body)
        h = ph[5]
        n, enc = h[1], h[2]
        p, rl, dl = 0, None, None
        if max_rep:
            ln = struct.unpack_from("<I", raw, p)[0]
            rl = rle_hybrid(raw[p + 4:p + 4 + ln], max_rep.bit_length(), n)
            p += 4 + ln
        if max_def:
            ln = struct.unpack_from("<I", raw, p)[0]
            dl = rle_hybrid(raw[p + 4:p + 4 + ln], max_def.bit_length(), n)
            p += 4 + ln
        data = raw[p:]
    elif ph[1] == 3:
        h = ph[8]
        n, enc, dlen, rlen = h[1], h[4], h[5], h[6]
        rl = rle_hybrid(body[:rlen], max_rep.bit_length(), n) if max_rep else None
        dl = rle_hybrid(body[rlen:rlen + dlen], max_def.bit_length(), n) if max_def else None
        data = body[rlen + dlen:]
        data = _decomp(codec, data) if h.get(7, True) else bytes(data)
    else:
        raise ValueError(f"page type {ph[1]}")
    nn = int(np.sum(dl == max_def)) if dl is not None else n
    if enc == 0:
        v = _plain(data, ptype, nn)
    elif enc in (2, 8):
        idx = rle_hybrid(data[1:], data[0], nn)
        v = dictionary[idx] if isinstance(dictionary, np.ndarray) else [dictionary[i] for i in idx]
    else:
        raise ValueError(f"encoding {enc}")
    if rl is None:
        rl = np.zeros(n, np.uint32)
    if dl is None:
        dl = np.full(n, max_def, np.uint32)
    return v, rl, dl


def das_rows(url, size, want_rows):
    """Range-stream a remote DroneAudioSet shard's audio column; stop after the last wanted row.
    Returns ({row: channel-0 float32}, {row: n channels}, sampling rates, bytes fetched)."""
    fm = read_footer(url, size=size)
    lv = schema_levels(fm)
    cols = dict(columns(fm))
    key = "audio.array.list.element.list.element"
    mr, mdf = lv[key]
    assert (mr, mdf) == (2, 6), lv
    s, e = chunk_range(cols["audio.sampling_rate"])
    sr = [int(v) for v in read_column(http_get(url, s, e - 1), cols["audio.sampling_rate"],
                                      *lv["audio.sampling_rate"])[0]]
    s, e = chunk_range(cols[key])
    req = urllib.request.Request(url, headers=dict(UA, Range=f"bytes={s}-{e - 1}"))
    out = {r: [] for r in want_rows}
    nch = {}
    row, ch = -1, 0
    with urllib.request.urlopen(req, timeout=60) as resp:
        seq = _Seq(resp)
        for vals, rep, dfn in iter_pages(seq, cols[key], mr, mdf):
            assert np.all(dfn == mdf), "nulls in audio column"
            idx = np.arange(len(rep))       # channel index: resets on rep<=1 (new sample), +1 on rep==2
            last = np.maximum.accumulate(np.where(rep <= 1, idx, -1))
            chan = np.where(last >= 0, idx - last, ch + 1 + idx)
            rows = row + np.cumsum(rep == 0)
            for r in np.unique(rows):
                m = rows == r
                nch[int(r)] = max(nch.get(int(r), 0), int(chan[m].max()) + 1)
                if int(r) in out:
                    out[int(r)].append(vals[m & (chan == 0)].astype(np.float32))
            row, ch = int(rows[-1]), int(chan[-1])
            if row > max(want_rows):
                break
    return {r: np.concatenate(v) for r, v in out.items()}, nch, sr, seq.n


# =========================================================================== index / fetch
def das_shards():
    from huggingface_hub import HfApi
    info = HfApi().dataset_info(DAS_REPO, files_metadata=True)
    return [(s.rfilename, s.size) for s in info.siblings if s.rfilename.endswith(".parquet")]


def cmd_index(args):
    """Row index of every DroneAudioSet shard from footers + the tiny file_path column chunk."""
    out = []
    for name, size in das_shards():
        if args.config and not name.startswith(args.config):
            continue
        url = DAS_URL + name
        fm = read_footer(url, size=size)
        cols = dict(columns(fm))
        s, e = chunk_range(cols["file_path"])
        vals, _r, _d = read_column(http_get(url, s, e - 1), cols["file_path"], 0, 1)
        out.append({"shard": name, "size": size, "rows": [v.decode() for v in vals]})
        print(name, size, len(vals), vals[0].decode(), flush=True)
    os.makedirs(DATA, exist_ok=True)
    path = os.path.join(DATA, "das_index.json")
    old = json.load(open(path)) if os.path.exists(path) else []
    keep = {o["shard"]: o for o in old}
    keep.update({o["shard"]: o for o in out})
    json.dump(sorted(keep.values(), key=lambda o: o["shard"]), open(path, "w"), indent=1)
    print("wrote", path, len(keep), "shards")


ESC_RAW = "https://raw.githubusercontent.com/karolpiczak/ESC-50/master/"
# label: (binary human-target, coarse group). Positives are what the agent should rank as a lead.
ESC_CLASSES = {
    "crying_baby": (1, "cry"), "door_wood_knock": (1, "knock"), "coughing": (1, "cough"),
    "breathing": (1, "breathing"),
    "dog": (0, "dog"), "engine": (0, "engine"), "helicopter": (0, "rotor"), "chainsaw": (0, "engine"),
    "wind": (0, "ambient"), "rain": (0, "ambient"), "crackling_fire": (0, "ambient"),
    "glass_breaking": (0, "debris"), "footsteps": (0, "footsteps"), "door_wood_creaks": (0, "debris"),
}


def fetch_esc50(per_class):
    import csv
    import soundfile as sf
    from scipy.signal import resample_poly
    d = os.path.join(DATA, "esc50")
    os.makedirs(d, exist_ok=True)
    meta = http_get(ESC_RAW + "meta/esc50.csv").decode()
    open(os.path.join(d, "esc50.csv"), "w").write(meta)
    rows = [r for r in csv.DictReader(io.StringIO(meta)) if r["category"] in ESC_CLASSES]
    total = 0
    for cat in ESC_CLASSES:
        # round-robin over folds 1..5 (sorted names within a fold) so no single fold dominates
        byf = {}
        for r in sorted((r for r in rows if r["category"] == cat), key=lambda r: r["filename"]):
            byf.setdefault(r["fold"], []).append(r)
        pick = [r for k in range(8) for f in sorted(byf) for r in byf[f][k:k + 1]][:per_class]
        for r in pick:
            out = os.path.join(d, r["filename"].replace(".wav", "_16k.wav"))
            if os.path.exists(out):
                continue
            raw = http_get(ESC_RAW + "audio/" + r["filename"])
            total += len(raw)
            x, fs = sf.read(io.BytesIO(raw), dtype="float32", always_2d=True)
            x = x.mean(axis=1)
            if fs != 16000:
                g = np.gcd(16000, fs)
                x = resample_poly(x, 16000 // g, fs // g).astype(np.float32)
            sf.write(out, x, 16000, subtype="PCM_16")
        print(f"esc50 {cat}: {len(pick)} clips", flush=True)
    print(f"esc50 downloaded {total / 1e6:.1f} MB")


# =========================================================================== run
SNAP_FILES = ("__init__.py", "models.py", "agent.py", "scene.py", "dsp.py", "run.py")
# commit under test: acoustic BL-3 by default; VALIDATE_REV=<rev> evaluates another (before/after)
CODE_REV = os.environ.get("VALIDATE_REV", "eb8cdba")
SNAP = os.path.join(DATA, "code_snapshot", CODE_REV)


def snapshot(rev=None):
    """acoustic/*.py belong to other agents and change while we run, so the code under test is a
    frozen copy of a commit (data/audio_real/code_snapshot/acoustic_snap, from `git show`), made
    on first use or when --rev changes it. Results files carry its hash (models.py + agent.py)."""
    import hashlib
    import subprocess
    d = os.path.join(SNAP, "acoustic_snap")
    taken = os.path.join(d, "REV")
    have = open(taken).read().strip() if os.path.exists(taken) else None
    want = rev or CODE_REV
    if want != have or not all(os.path.exists(os.path.join(d, f)) for f in SNAP_FILES):
        os.makedirs(d, exist_ok=True)
        for f in SNAP_FILES:
            src = subprocess.run(["git", "-C", ROOT, "show", f"{want}:acoustic/{f}"], capture_output=True,
                                 check=True).stdout
            open(os.path.join(d, f), "wb").write(src)
        open(taken, "w").write(want + "\n")
    h = hashlib.sha1()
    for f in ("models.py", "agent.py"):
        h.update(open(os.path.join(d, f), "rb").read())
    return h.hexdigest()[:8]


def code(full=False):
    """(agent, models) - or (agent, models, scene, dsp, run) - from the snapshot; model files and
    the TTS / dog-clip caches stay in acoustic/."""
    if SNAP not in sys.path:
        sys.path.insert(0, SNAP)
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)
    from acoustic_snap import agent, dsp, models, run, scene
    scene.MODEL_DIR = models.MODEL_DIR = os.path.join(HERE, "models")
    scene.DATA_DIR = os.path.join(HERE, "data")
    scene.PIPER_VOICE = os.path.join(scene.MODEL_DIR, "en_US-lessac-low.onnx")
    scene.DOG_WAV = os.path.join(scene.DATA_DIR, "esc50", "1-100032-A-0.wav")
    return (agent, models, scene, dsp, run) if full else (agent, models)


VERSION = snapshot()
RESULTS = os.path.join(DATA, f"results-{VERSION}.jsonl")
RESULTS_FV = os.path.join(DATA, "results_fv.jsonl")
MANIFEST = os.path.join(DATA, "manifest.json")
# No array here: DoA-dependent agent inputs are fixed to one neutral, good geometry so the
# agent's human/urgency (what we validate) depend only on the recognisers.
NEUTRAL_DOA = {"drones_agreeing": 3, "drones_listening": 3, "tri_err_m": 1.0, "tri_resid_m": 0.3,
               "assigned": True, "association": "confident", "association_conf": 0.9,
               "ambiguous_with": [], "overlap_frac": 0.0}
HUMAN_CLASSES = ("voice", "shout", "knocking")


WIN_S = 5.0
SNR_LEVELS = [10, 5, 0, -5, -10, -20]       # dB, rotor noise at throttle-high (controlled mixes)
SNR_LEVELS_LOW = [0, -10]                   # dB, throttle-low noise
ESC_SNRS = [10, 0, -10]
NOISE = {"high": "drone-only-recordings/drone1-only/mic-dist-50cm/throttle-high/mic1_soundskrit-File1.wav",
         "low": "drone-only-recordings/drone1-only/mic-dist-50cm/throttle-low/mic1_soundskrit-File1.wav"}
CAT_LABEL = {"male": "speech", "female": "speech", "crying": "crying", "humansounds": "human_nonvocal",
             "nonhumansounds": "nonhuman"}
POSITIVE = {"speech", "crying", "human_nonvocal", "cry", "knock", "cough", "breathing"}


def _read(rel):
    import soundfile as sf
    return sf.read(os.path.join(DATA, rel), dtype="float32")[0]


def _power(x):
    return float(np.mean(np.asarray(x, np.float64) ** 2)) + 1e-20


def _lag(x, ref, fs=16000, T=60.0, maxlag=20.0):
    """Recording delay vs the digital source by GCC-PHAT on the first T s; None if weak."""
    n, L = int(T * fs), int(maxlag * fs)
    a, b = x[:n + L], ref[:n]
    N = 1 << int(np.ceil(np.log2(len(a) + len(b))))
    R = np.fft.rfft(a, N) * np.conj(np.fft.rfft(b, N))
    c = np.fft.irfft(R / (np.abs(R) + 1e-12), N)
    c = np.concatenate([c[-L:], c[:L]])
    i = int(np.argmax(c))
    return (i - L) / fs if c[i] > 15 * np.median(np.abs(c)) else None


def annotations(k):
    path = os.path.join(DATA, "droneaudioset", "ground-truth-annotations", f"File{k}.txt")
    return [(float(a), float(b), c.strip()) for a, b, c in
            (ln.split("\t") for ln in open(path).read().splitlines() if ln.strip())]


def parse_cond(fp):
    """Condition fields from a DroneAudioSet path."""
    d = {}
    for part in fp.split("/"):
        for key, pre in (("loud", "speaker-loudness-"), ("dist", "speaker-dist-"), ("micd", "mic-dist-"),
                         ("thr", "throttle-")):
            if part.startswith(pre):
                d[key] = part[len(pre):]
        if part.startswith("room"):
            d["room"] = part
    m = re.search(r"File(\d)", fp)
    d["file"] = int(m.group(1)) if m else 0
    d["kind"] = fp.split("-recordings")[0]
    return d


def build_manifest():
    """Every clip the run analyses: ESC-50 clips (clean + rotor-noise mixes), DroneAudioSet 5 s
    windows (source-only, controlled mixes with real rotor noise, real rotors-on recordings), and
    30 s speech segments for ASR. Mixing gains are fixed here so runs are reproducible."""
    import csv
    rng = np.random.default_rng(7)
    noise_p = {k: _power(_read(das_local(v))) for k, v in NOISE.items()}
    noise_len = {k: len(_read(das_local(v))) / 16000 for k, v in NOISE.items()}
    man = []

    def noise_spec(kind, p_sig, snr, dur):
        g = float(np.sqrt(p_sig / (noise_p[kind] * 10 ** (snr / 10))))
        return {"wav": das_local(NOISE[kind]), "t0": round(float(rng.uniform(0, noise_len[kind] - dur - 0.1)), 3),
                "gain": g, "kind": kind, "snr_db": snr}

    # ESC-50
    meta = list(csv.DictReader(open(os.path.join(DATA, "esc50", "esc50.csv"))))
    for r in meta:
        rel = os.path.join("esc50", r["filename"].replace(".wav", "_16k.wav"))
        if r["category"] not in ESC_CLASSES or not os.path.exists(os.path.join(DATA, rel)):
            continue
        base = {"wav": rel, "t0": 0.0, "t1": 5.0, "label": ESC_CLASSES[r["category"]][1],
                "fine": r["category"], "human": ESC_CLASSES[r["category"]][0]}
        man.append(dict(base, id=f"esc|clean|{r['filename']}", set="esc", snr_db=None))
        p = _power(_read(rel))
        for snr in ESC_SNRS:
            man.append(dict(base, id=f"esc_mix|high{snr:+d}|{r['filename']}", set="esc_mix", snr_db=snr,
                            noise=noise_spec("high", p, snr, 5.0)))
    # DroneAudioSet
    gt = {k: _read(os.path.join("das", "ground-truth", f"File{k}.wav")) for k in range(1, 7)}
    so_ref = {}                                          # (loud, room, dist, file) -> SO path
    for w in DAS_WANT:
        c = parse_cond(w)
        if c["kind"] == "source-only":
            so_ref[(c["loud"], c["room"], c["dist"], c["file"])] = w
    for w in dict.fromkeys(DAS_WANT):
        c = parse_cond(w)
        if c["kind"] == "drone-only":
            continue
        x = _read(das_local(w))
        lag = _lag(x, gt[c["file"]])
        lag_ok = lag is not None
        lag = lag if lag_ok else 0.3                     # median of the reliable lags
        cond = f"{c['loud']}-{c['room']}-{c['dist']}" + (f"-{c['thr']}" if "thr" in c else "")
        so = x if c["kind"] == "source-only" else _read(das_local(so_ref[(c["loud"], c["room"], c["dist"], 1)]))
        for a, b, cat in annotations(c["file"]):
            a, b = a + lag + 0.25, b + lag - 0.25
            if b > len(x) / 16000:
                continue
            seg_p = _power(so[int(a * 16000):int(b * 16000)])     # source-only power: SNR numerator
            common = {"wav": das_local(w), "label": CAT_LABEL[cat], "fine": cat,
                      "human": int(CAT_LABEL[cat] in POSITIVE), "cond": cond, "file": c["file"],
                      "lag_s": round(lag, 3), "lag_measured": lag_ok}
            if c["kind"] == "drone-with-source":
                snr = 10 * np.log10(seg_p / noise_p[c["thr"]])
                sets = [("dws", round(float(snr), 1), None)]
            elif w.startswith(SO_ALL_FILES):
                sets = [("so", None, None)] + [("das_mix", s_, ("high", s_)) for s_ in SNR_LEVELS] \
                    + [("das_mix", s_, ("low", s_)) for s_ in SNR_LEVELS_LOW]
            else:
                sets = [("so_cond", None, None)]
            for st, snr, nz in sets:
                tag = cond if nz is None else f"{nz[0]}{nz[1]:+d}"
                extra = {"set": st, "snr_db": snr}
                n = None
                if nz is not None:
                    n = noise_spec(nz[0], seg_p, nz[1], b - a)
                for t in np.arange(a, b - WIN_S + 1e-6, WIN_S):
                    d = dict(common, **extra, id=f"{st}|{tag}|F{c['file']}|{cat}|{t:.1f}", t0=round(float(t), 3),
                             t1=round(float(t + WIN_S), 3))
                    if n is not None:
                        d["noise"] = dict(n, t0=round(n["t0"] + (t - a), 3))
                    man.append(d)
                if cat in ("male", "female"):                     # whole segment for ASR
                    d = dict(common, snr_db=snr, id=f"asr_{st}|{tag}|F{c['file']}|{cat}", t0=round(a, 3),
                             t1=round(b, 3), asr=True, set="asr_" + st)
                    if n is not None:
                        d["noise"] = n
                    man.append(d)
    for k in range(1, 7):                                 # ASR reference: the digital source itself
        for a, b, cat in annotations(k):
            if cat in ("male", "female"):
                man.append({"id": f"asr_ref|gt|F{k}|{cat}", "set": "asr_ref", "wav": f"das/ground-truth/File{k}.wav",
                            "t0": round(a + 0.25, 3), "t1": round(b - 0.25, 3), "asr": True, "label": "speech",
                            "fine": cat, "human": 1, "file": k, "snr_db": None})
    # Team recordings (F. Valisalo, branch acoustic-voice-detection; CC0 Freesound / Wikimedia clips)
    fdir = os.path.join(DATA, "frances")
    if os.path.exists(os.path.join(fdir, "manifest.csv")):
        for r in csv.DictReader(open(os.path.join(fdir, "manifest.csv"))):
            rel = os.path.join("frances", r["file"])
            x = _read(rel)
            dur = min(len(x) / 16000, 14.0)              # run.extract_clip keeps <= 14 s
            human = int(r["category"] == "voice")
            base = {"wav": rel, "t0": 0.0, "t1": round(dur, 3), "label": "speech" if human else r["category"],
                    "fine": r["category"], "human": human, "kw_expected": bool(human and "help" in r["title"].lower())}
            man.append(dict(base, id=f"fv|clean|{r['file']}", set="fv", snr_db=None))
            p = _power(x[:int(dur * 16000)])
            for snr in (0, -10):
                man.append(dict(base, id=f"fv_mix|high{snr:+d}|{r['file']}", set="fv_mix", snr_db=snr,
                                noise=noise_spec("high", p, snr, dur)))
    json.dump(man, open(MANIFEST, "w"), indent=0)
    print(f"manifest: {len(man)} clips -> {MANIFEST}")
    return man


def load_clip(c):
    import soundfile as sf

    def rd(rel, t0, dur):
        with sf.SoundFile(os.path.join(DATA, rel)) as f:
            assert f.samplerate == 16000, rel
            f.seek(int(t0 * 16000))
            return f.read(int(dur * 16000), dtype="float32", always_2d=True)[:, 0]

    dur = c["t1"] - c["t0"]
    x = rd(c["wav"], c["t0"], dur)
    if c.get("noise"):
        nz = c["noise"]
        y = rd(nz["wav"], nz["t0"], dur)
        n = min(len(x), len(y))
        x = x[:n] + nz["gain"] * y[:n]
    return x.astype(np.float32)


def analyse(c, x, whisper="base.en"):
    agent, models = code()
    vad = models.vad_speech_ratio(x)
    cls, scores, heur, top = models.classify(x, vad)
    text = models.transcribe(x, whisper) if vad >= 0.15 else ""
    feat = {"clip_id": c["id"], "pile_id": "X", "drone_id": "d1", "duration_s": round(len(x) / 16000, 1),
            "continuous": False, "bearing_deg": 0.0, "width_deg": 25.0, "snr_db": 10.0,
            "vad_speech_ratio": vad, "transcript": text, "sound_class": cls, "class_scores": scores,
            "heuristics": heur, "yamnet_top": [[lbl, sc] for lbl, sc in top], "doa": dict(NEUTRAL_DOA)}
    j = agent.fallback_judge(feat)
    out = {"id": c["id"], "vad": vad, "cls": cls, "scores": scores, "heur": heur,
           "top": [[lbl, sc] for lbl, sc in top], "text": text, "human": j["human"],
           "urgency": j["urgency"], "reliability": j["reliability"], "priority": agent.clip_priority(j)}
    if c.get("asr") or c.get("kw_expected"):     # speech clips: also transcribe without the VAD gate
        out["text_ungated"] = text if vad >= 0.15 else models.transcribe(x, whisper)
    return out


# Frances' classifier rule (acoustic/acoustic_pipeline.py on branch acoustic-voice-detection),
# re-implemented for single-channel clips: per 2 s block (hop 1 s) the best over 0.975 s YAMNet
# windows (hop 0.4875 s) of max(human classes) - max(animal classes); a block is voice at >= 0.2;
# a site is flagged with >= 2 voice blocks (her bearing clustering needs an array: not applied).
FV_HUMAN = {"Speech", "Child speech, kid speaking", "Conversation", "Narration, monologue", "Babbling", "Shout",
            "Bellow", "Whoop", "Yell", "Children shouting", "Screaming", "Crying, sobbing",
            "Baby cry, infant cry", "Whimper", "Wail, moan", "Groan", "Hubbub, speech noise, speech babble"}
FV_ANIMAL = {"Animal", "Domestic animals, pets", "Dog", "Bark", "Yip", "Howl", "Bow-wow", "Growling",
             "Whimper (dog)", "Livestock, farm animals, working animals", "Wild animals"}
_fv = {}


def fv_score(x):
    import csv
    if not _fv:
        from ai_edge_litert.interpreter import Interpreter
        md = os.path.join(DATA, "frances", "models")
        it = Interpreter(model_path=os.path.join(md, "yamnet.tflite"), num_threads=1)
        it.allocate_tensors()
        names = [r["display_name"] for r in csv.DictReader(open(os.path.join(md, "yamnet_class_map.csv")))]
        _fv.update(it=it, i=it.get_input_details()[0]["index"], o=it.get_output_details()[0]["index"],
                   h=[k for k, n in enumerate(names) if n in FV_HUMAN],
                   a=[k for k, n in enumerate(names) if n in FV_ANIMAL])
    W = 15600

    def block_score(b):
        b = np.pad(b.astype(np.float64), (0, max(W - b.size, 0)))
        rms = np.sqrt(np.mean(b ** 2))
        if rms == 0:
            return 0.0
        b = np.clip(b * (0.1 / rms), -1, 1).astype(np.float32)
        best = -1.0
        for s0 in range(0, b.size - W + 1, W // 2):
            _fv["it"].set_tensor(_fv["i"], b[s0:s0 + W])
            _fv["it"].invoke()
            sc = _fv["it"].get_tensor(_fv["o"])[0]
            best = max(best, float(sc[_fv["h"]].max() - sc[_fv["a"]].max()))
        return best

    n = len(x)
    starts = list(range(0, max(n - 32000, 0) + 1, 16000))
    scores = [block_score(x[s0:s0 + 32000]) for s0 in starts]
    nv = int(sum(v >= 0.2 for v in scores))
    return {"id": None, "fv_max": round(max(scores), 3), "fv_blocks": nv, "fv_nblocks": len(scores),
            "fv_flag": nv >= min(2, len(scores))}


def cmd_run(args):
    man = json.load(open(MANIFEST)) if os.path.exists(MANIFEST) and not args.rebuild else build_manifest()
    out_path = RESULTS_FV if args.fv else RESULTS
    done = set()
    if os.path.exists(out_path):
        done = {json.loads(line)["id"] for line in open(out_path) if line.strip()}
    if args.fv:
        man = [c for c in man if not c.get("asr")]
    todo = [c for c in man if c["id"] not in done and (not args.only or re.search(args.only, c["id"]))]
    print(f"{len(done)} done, {len(todo)} to do", flush=True)
    t0 = time.time()
    n = 0
    with open(out_path, "a") as fh:
        for c in todo:
            if time.time() - t0 > args.budget_s:
                break
            r = dict(fv_score(load_clip(c)), id=c["id"]) if args.fv else analyse(c, load_clip(c))
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            n += 1
    left = len(todo) - n
    print(f"analysed {n} clips in {time.time() - t0:.0f} s; {left} left" + (" (rerun to continue)" if left else ""))


# DroneAudioSet rows we keep (mic1 = the mono "soundskrit" mic; the 8-ch arrays cost 8x).
#   SO  = source-only (speaker in a room, no drone: closest to our rotors-off listen)
#   DO  = drone-only (rotor noise, no source), used as real noise for controlled-SNR mixes
#   DWS = drone-with-source (real rotors-on recordings)
SO_ALL_FILES = "source-only-recordings/speaker-loudness-90db/room1/speaker-dist-1m/mic-dist-50cm"
DAS_CONDS = [("room1", "speaker-dist-1m"), ("room1", "speaker-dist-3m"), ("room1", "speaker-dist-5m")]
DAS_WANT = (
    [f"{SO_ALL_FILES}/mic1_soundskrit-File{k}.wav" for k in (1, 3, 4, 5, 6)]
    + [f"source-only-recordings/speaker-loudness-{L}/{room}/{sd}/mic-dist-50cm/mic1_soundskrit-File1.wav"
       for L in ("90db", "60db") for room, sd in DAS_CONDS if (L, sd) != ("90db", "speaker-dist-1m")]
    + [f"source-only-recordings/speaker-loudness-90db/room2/speaker-dist-{d}m/mic-dist-50cm/mic1_soundskrit-File1.wav"
       for d in (3, 6, 9)]
    + ["drone-only-recordings/drone1-only/mic-dist-50cm/throttle-low/mic1_soundskrit-File1.wav",
       "drone-only-recordings/drone1-only/mic-dist-50cm/throttle-high/mic1_soundskrit-silence.wav",
       "drone-only-recordings/drone1-only/mic-dist-50cm/throttle-high/mic1_soundskrit-File1.wav"]
    + [f"drone-with-source-recordings/speaker-loudness-{L}/{room}/drone1/{sd}/mic-dist-50cm/throttle-low/"
       "mic1_soundskrit-File1.wav" for L in ("90db", "60db") for room, sd in DAS_CONDS]
    + [f"drone-with-source-recordings/speaker-loudness-90db/room2/drone1/speaker-dist-{d}m/mic-dist-50cm/"
       "throttle-low/mic1_soundskrit-File1.wav" for d in (3, 6, 9)]
    + [f"drone-with-source-recordings/speaker-loudness-{L}/room1/drone1/speaker-dist-1m/mic-dist-50cm/"
       "throttle-high/mic1_soundskrit-File1.wav" for L in ("90db", "60db")]
)


def das_local(fp):
    return os.path.join("das", fp.replace("/", "__"))


def fetch_das():
    import soundfile as sf
    idx = json.load(open(os.path.join(DATA, "das_index.json")))
    where = {r: (o["shard"], i, o["size"]) for o in idx for i, r in enumerate(o["rows"])}
    missing = [w for w in DAS_WANT if w not in where]
    assert not missing, missing
    os.makedirs(os.path.join(DATA, "das"), exist_ok=True)
    by_shard = {}
    for w in DAS_WANT:
        if not os.path.exists(os.path.join(DATA, das_local(w))):
            sh, i, size = where[w]
            by_shard.setdefault((sh, size), []).append((i, w))
    total = 0
    for (sh, size), items in sorted(by_shard.items()):
        rows, nch, sr, nbytes = das_rows(DAS_URL + sh, size, [i for i, _ in items])
        total += nbytes
        for i, w in items:
            assert sr[i] == 16000 and nch[i] == 1, (w, sr[i], nch[i])
            sf.write(os.path.join(DATA, das_local(w)), rows[i], 16000, subtype="PCM_16")
        print(f"{sh}: rows {[i for i, _ in items]} ({nbytes / 1e6:.1f} MB)", flush=True)
    # ground-truth digital source files (one 24 MB shard)
    gt = os.path.join(DATA, "das", "ground-truth")
    if not os.path.exists(os.path.join(gt, "File1.wav")):
        os.makedirs(gt, exist_ok=True)
        o = next(o for o in idx if o["shard"].startswith("ground-truth"))
        rows, nch, sr, nbytes = das_rows(DAS_URL + o["shard"], o["size"], list(range(len(o["rows"]))))
        total += nbytes
        for i, r in enumerate(o["rows"]):
            sf.write(os.path.join(gt, os.path.basename(r)), rows[i], sr[i], subtype="PCM_16")
    print(f"DroneAudioSet fetched {total / 1e6:.1f} MB")


def cmd_fetch(args):
    fetch_esc50(args.esc_per_class)
    if args.das:
        fetch_das()


# =========================================================================== what-if (no source edits)
WHATIF = {
    "baseline": {},
    "knock_strict": {"knocking": [353, 454]},                       # Knock, Thump only (drop Tap, Wood, Wood block)
    "human_plus": {"voice": [0, 1, 2, 3, 5, 19, 20, 21, 22, 33, 36, 65], "shout": [6, 7, 8, 9, 10, 11]},
    "both": {"knocking": [353, 454], "voice": [0, 1, 2, 3, 5, 19, 20, 21, 22, 33, 36, 65],
             "shout": [6, 7, 8, 9, 10, 11]},
}
WHATIF_SETS = r"^(esc|so|fv|so_cond)\||^(das_mix|esc_mix)\|high\+0\||^dws\|90db-room1-[135]m-low"


def cmd_whatif(args):
    """Candidate class-group changes, evaluated in-process by temporarily swapping
    acoustic.models.GROUPS (the file is not edited). Also raw Whisper without the
    hallucination filter on noisy speech segments. -> data/audio_real/whatif-<version>.jsonl"""
    agent, models = code()
    man = [c for c in json.load(open(MANIFEST)) if not c.get("asr") and re.search(WHATIF_SETS, c["id"])]
    res = {json.loads(line)["id"]: json.loads(line) for line in open(RESULTS)}
    path = os.path.join(DATA, f"whatif-{VERSION}.jsonl")
    done = {json.loads(line)["id"] for line in open(path)} if os.path.exists(path) else set()
    base = {k: list(v) for k, v in models.GROUPS.items()}
    t0 = time.time()
    with open(path, "a") as fh:
        for c in man:
            if c["id"] in done or c["id"] not in res:
                continue
            if time.time() - t0 > args.budget_s:
                break
            x = load_clip(c)
            r = res[c["id"]]
            out = {"id": c["id"]}
            for name, ch in WHATIF.items():
                models.GROUPS.clear()
                models.GROUPS.update(base)
                models.GROUPS.update(ch)
                cls, scores, heur, top = models.classify(x, r["vad"])
                f = {"class_scores": scores, "vad_speech_ratio": r["vad"], "transcript": r["text"], "heuristics": heur,
                     "sound_class": cls, "doa": dict(NEUTRAL_DOA), "width_deg": 25.0, "snr_db": 10.0}
                j = agent.fallback_judge(f)
                out[name] = {"cls": cls, "human": j["human"], "priority": agent.clip_priority(j)}
            models.GROUPS.clear()
            models.GROUPS.update(base)
            fh.write(json.dumps(out) + "\n")
    if args.whisper_raw:
        from faster_whisper import WhisperModel
        wm = WhisperModel("base.en", device="cpu", compute_type="int8", cpu_threads=2,
                          download_root=os.path.join(models.MODEL_DIR, "whisper"))
        rp = os.path.join(DATA, f"whisper_raw-{VERSION}.jsonl")
        with open(rp, "w") as fh:
            for c in json.load(open(MANIFEST)):
                if c.get("asr") and re.search(r"asr_(das_mix\|high(\+5|\+0|-5)|dws\|90db-room1-1m-low)", c["id"]):
                    segs, _ = wm.transcribe(models._norm(load_clip(c), 0.1), language="en", beam_size=1,
                                            temperature=0.0, condition_on_previous_text=False, vad_filter=False)
                    segs = list(segs)
                    fh.write(json.dumps({"id": c["id"], "raw": " ".join(sg.text.strip() for sg in segs),
                                         "segs": [[round(sg.no_speech_prob, 2), round(sg.avg_logprob, 2)]
                                                  for sg in segs]}) + "\n")
    print(f"what-if done in {time.time() - t0:.0f} s")


# =========================================================================== synthetic head-to-head
# EVALUATOR: reads scenario truth after both pipelines have run (rule 9).
SYNTH = os.path.join(DATA, "synth-{fam}-%s.jsonl")
MATCH_DEG = 15.0                       # a bearing flag counts for a source within this angle
HUMAN_KINDS = {"voice", "weak_voice", "knock"}
_fvmod = {}


def fv_module():
    """Teammate's pipeline (branch acoustic-voice-detection, copied read-only to data/audio_real/frances)."""
    if "m" not in _fvmod:
        import importlib.util
        spec = importlib.util.spec_from_file_location("fv_pipeline", os.path.join(DATA, "frances", "acoustic_pipeline.py"))
        m = importlib.util.module_from_spec(spec)
        sys.modules["fv_pipeline"] = m           # dataclasses resolve the module by name
        spec.loader.exec_module(m)
        _fvmod["m"] = m
    return _fvmod["m"]


def _brg(o, p):
    return float(np.degrees(np.arctan2(p[1] - o[1], p[0] - o[0])) % 360)


def _gap(a, b):
    return abs((a - b + 180) % 360 - 180)


def score_flags(flags, sources):
    """flags [(drone_xy, bearing)], sources [(kind, xy)] -> per-flag nearest source + error."""
    out = []
    for o, b in flags:
        best = min(((_gap(b, _brg(o, xy)), kind, i) for i, (kind, xy) in enumerate(sources)), default=None)
        out.append({"err": round(best[0], 1), "kind": best[1], "src": best[2]} if best and best[0] <= MATCH_DEG
                   else {"err": None, "kind": None, "src": None, "nearest_err": None if not best else round(best[0], 1)})
    return out


def ours_single_array(sig, offs, fs, run, dsp, models, agent):
    """Our front end on ONE array (no multi-drone pile association): CLEAN SRP-PHAT per 0.5 s
    block, directions seen in >= 2 blocks (10 deg clusters), LCMV beam toward each with nulls on
    the others, its active blocks -> VAD / YAMNet+heuristics / gated Whisper / fallback agent."""
    model = dsp.SrpModel(offs, fs)
    Rt, Wt, Rs, Ws = dsp.gcc_blocks(model, sig, fs)
    B = Rt.shape[0]
    comps = [dsp.clean(model, Rt[b], Wt[b]) for b in range(B)]
    pts = [(az, amp, b) for b in range(B) for az, amp, _w in comps[b]]
    dirs = []
    while pts:
        cnt = [sum(amp for az2, amp, _ in pts if _gap(az, az2) <= 10) for az, _, _ in pts]
        c = pts[int(np.argmax(cnt))][0]
        mem = [q for q in pts if _gap(q[0], c) <= 10]
        pts = [q for q in pts if _gap(q[0], c) > 10]
        blocks = sorted({b for _, _, b in mem})
        if len(blocks) >= 2:
            w = np.array([amp for _, amp, _ in mem])
            ang = np.radians([az for az, _, _ in mem])
            dirs.append((float(np.degrees(np.arctan2((w * np.sin(ang)).sum(), (w * np.cos(ang)).sum())) % 360), blocks))
    blk = int(dsp.BLOCK_S * fs)
    out = []
    for i, (az, blocks) in enumerate(dirs[:6]):
        nulls = [d for j, (d, _) in enumerate(dirs[:6]) if j != i and _gap(d, az) > 25][:4]
        y = dsp.beamform(sig, offs, fs, az, nulls=nulls)
        act = np.zeros(B, bool)
        act[blocks] = True
        act = act | np.r_[act[1:], False] | np.r_[False, act[:-1]]
        clip = np.concatenate([y[b * blk:(b + 1) * blk] for b in np.flatnonzero(act)])[: 14 * fs]
        vad = models.vad_speech_ratio(clip)
        cls, scores, heur, top = models.classify(clip, vad)
        text = models.transcribe(clip) if vad >= 0.15 else ""
        f = {"class_scores": scores, "vad_speech_ratio": vad, "transcript": text, "heuristics": heur,
             "sound_class": cls, "width_deg": 25.0, "snr_db": 10.0,
             "doa": {"drones_agreeing": 1, "drones_listening": 1, "tri_err_m": None, "assigned": False}}
        j = agent.fallback_judge(f)
        out.append({"bearing": round(az, 1), "blocks": len(blocks), "cls": cls, "human": j["human"],
                    "text": text, "flag": j["human"] >= LEAD})
    return out


def synth_family_a(seed, noise_x):
    """Our scenario generator + scene synthesis (3 landed 6-mic 0.3 m arrays, pyroomacoustics,
    TTS voices, numpy knocks, ESC dog, generator, debris). Ours = full run.analyse; teammate =
    detect_voice per drone on the same signals."""
    from sim.scenario import make_scenario
    agent, models, scene_m, dsp, run = code(full=True)
    fv = fv_module()
    sc = make_scenario(seed)
    scene, truth = scene_m.build_scene(sc, noise_rms=4.0e-3 * noise_x)
    t = time.time()
    state = run.analyse(scene, (), mode="fallback")
    t_ours = time.time() - t
    srcs = [(v["kind"], v["xy"], p) for p, v in truth.items() if p != scene.start_pile]
    ours = []
    for c in state["clips"]:
        o = scene.drones[c["drone_id"]]
        tr = truth.get(c["pile_id"])
        ours.append({"pile": c["pile_id"], "human": c["human"], "flag": c["human"] >= LEAD, "cls": c["sound_class"],
                     "association": c.get("association"), "truth_kind": tr["kind"] if tr else None,
                     "brg_err": round(_gap(c["bearing_deg"], _brg(o, tr["xy"])), 1) if tr else None})
    t = time.time()
    fvd = []
    for d, xy in scene.drones.items():
        mic = np.column_stack([xy[0] + scene.mic_offsets[0], xy[1] + scene.mic_offsets[1], np.zeros(scene.mic_offsets.shape[1])])
        for det in fv.detect_voice(scene.signals[d].T.astype(float), scene.fs, mic):
            fvd.append({"drone": d, "bearing": det.bearing_deg, "blocks": det.voice_blocks})
    t_fv = time.time() - t
    allsrc = [(v["kind"], v["xy"]) for v in truth.values()]
    for f, m in zip(fvd, score_flags([(scene.drones[f["drone"]], f["bearing"]) for f in fvd], allsrc)):
        f.update(m)
    return {"family": "A", "seed": seed, "noise_x": noise_x, "start_pile": scene.start_pile,
            "sources": [{"kind": k, "pile": p, "xy": xy} for k, xy, p in srcs],
            "all_sources": [{"kind": k, "xy": xy} for k, xy in allsrc],
            "drones": {k: list(v) for k, v in scene.drones.items()},
            "ours": ours, "fv": fvd, "t_ours": round(t_ours, 2), "t_fv": round(t_fv, 2), "audio_s": scene.duration,
            "n_drones": len(scene.drones)}


def synth_family_b(seed, ambient):
    """Teammate's simulator (8-mic 0.1 m array, 30 s, diffuse ambient + wind, 1/r free field).
    Sources: 1-2 real 'help' recordings (her CC0 voice clips) + synthetic voice, distractors =
    her dog recordings / generator / engine / birds. Teammate = detect_voice; ours = our front
    end on one array (ours_single_array)."""
    fv = fv_module()
    agent, models, scene_m, dsp, run = code(full=True)
    rng = np.random.default_rng(seed)
    fdir = os.path.join(DATA, "frances")
    voices = sorted(f for f in os.listdir(fdir) if f.startswith("voice_"))
    dogs = sorted(f for f in os.listdir(fdir) if f.startswith("dog_"))
    sites = [fv.Site(k, (float(x), float(y), 0.0), (float(np.cos(q / 2)), 0.0, 0.0, float(np.sin(q / 2))))
             for k, (x, y, q) in zip("ABC", [(0, 0, rng.uniform(0, 6.28)), (60, 0, rng.uniform(0, 6.28)),
                                             (0, 80, rng.uniform(0, 6.28))])]
    sources = []
    for site in sites:
        nv = int(rng.integers(0, 3))                  # 0-2 voices per site
        nd = int(rng.integers(1, 3))                  # 1-2 distractors
        for k in range(nv + nd):
            r, a = rng.uniform(6, 25), rng.uniform(0, 2 * np.pi)
            pos = (site.position[0] + r * np.cos(a), site.position[1] + r * np.sin(a), 0.3)
            if k < nv:
                wav = os.path.join(fdir, voices[int(rng.integers(len(voices)))]) if rng.uniform() < 0.7 else None
                sources.append(fv.Source("voice", pos, float(rng.uniform(75, 90)), wav))
            else:
                kind = ["generator", "engine", "birds", "recording"][int(rng.integers(4))]
                wav = os.path.join(fdir, dogs[int(rng.integers(len(dogs)))]) if kind == "recording" else None
                sources.append(fv.Source(kind, pos, float(rng.uniform(70, 85)), wav))
    scene = fv.Scene(tuple(sites), tuple(sources), scan_seconds=30.0, ambient_spl=ambient, wind_spl=ambient - 5,
                     seed=seed)
    cfg = fv.ArrayConfig()
    rows = []
    for site in sites:
        stems = fv.simulate_site(scene, site, cfg)
        audio = stems["voice"] + stems["noise"]
        mic = fv.mic_positions_world(site, cfg)
        o = site.position[:2]
        t = time.time()
        dets = fv.detect_voice(audio.astype(float), cfg.fs, mic)
        t_fv = time.time() - t
        offs = (mic[:, :2] - mic[:, :2].mean(axis=0)).T
        t = time.time()
        oo = ours_single_array(audio.T.astype(np.float64), offs, cfg.fs, run, dsp, models, agent)
        t_ours = time.time() - t
        # every source in the scene can be heard (1/r): flags are matched against all of them;
        # recall counts the site's own sources (range <= 30 m, "local")
        srcs = [(s.kind if s.kind != "recording" else "dog", s.position[:2]) for s in sources]
        fvd = [{"bearing": d.bearing_deg, "blocks": d.voice_blocks} for d in dets]
        for f, m in zip(fvd, score_flags([(o, f["bearing"]) for f in fvd], srcs)):
            f.update(m)
        for f, m in zip(oo, score_flags([(o, f["bearing"]) for f in oo], srcs)):
            f.update(m)
        rows.append({"family": "B", "seed": seed, "ambient": ambient, "site": site.id,
                     "sources": [{"kind": k, "xy": list(xy), "brg": round(_brg(o, xy), 1),
                                  "range": round(float(np.hypot(xy[0] - o[0], xy[1] - o[1])), 1)} for k, xy in srcs],
                     "snr_db": fv.band_snr_db(stems, cfg), "fv": fvd, "ours": oo, "t_fv": round(t_fv, 2),
                     "t_ours": round(t_ours, 2), "audio_s": 30.0})
    return rows


def synth_summary():
    """Per family and level: human / voice recall, false flags per scene, flag precision,
    bearing error of matched flags, runtime. Pools all levels as 'all'."""
    out = {}
    for fam in ("A", "B"):
        path = SYNTH.format(fam=fam) % VERSION
        if not os.path.exists(path):
            continue
        rows = [json.loads(line) for line in open(path)]
        lvkey = "noise_x" if fam == "A" else "ambient"
        for lv in sorted({r[lvkey] for r in rows}) + ["all"]:
            R = [r for r in rows if lv == "all" or r[lvkey] == lv]
            res = {}
            for who in ("ours", "fv"):
                hum = vo = hum_hit = vo_hit = flags = tp = 0
                errs, errs_all, t = [], [], []
                for r in R:
                    if fam == "A":
                        srcs = r["sources"]
                        allsrc = r["all_sources"]
                        if who == "ours":
                            flagged = {c["pile"] for c in r["ours"] if c["flag"]}
                            hit = lambda s_: s_["pile"] in flagged
                            fl = [c for c in r["ours"] if c["flag"]]
                            flags += len(fl)
                            tp += sum(c["truth_kind"] in HUMAN_KINDS for c in fl)
                            errs += [c["brg_err"] for c in fl if c["truth_kind"] in HUMAN_KINDS]
                            errs_all += [c["brg_err"] for c in r["ours"] if c["brg_err"] is not None]
                        else:
                            matched = {tuple(allsrc[f["src"]]["xy"]) for f in r["fv"] if f["src"] is not None}
                            hit = lambda s_, m=matched: tuple(s_["xy"]) in m
                            flags += len(r["fv"])
                            tp += sum(f["kind"] in HUMAN_KINDS for f in r["fv"])
                            errs += [f["err"] for f in r["fv"] if f["kind"] in HUMAN_KINDS]
                            errs_all += [f["err"] for f in r["fv"] if f["err"] is not None]
                    else:
                        srcs = r["sources"]
                        fl = [f for f in r[who] if (f.get("flag", True))]
                        flags += len(fl)
                        tp += sum(f["kind"] == "voice" for f in fl)
                        got = {f["src"] for f in fl if f["kind"] == "voice"}
                        hit = lambda s_, g=got, ss=srcs: ss.index(s_) in g
                        errs += [f["err"] for f in fl if f["kind"] == "voice"]
                        errs_all += [f["err"] for f in r[who] if f["err"] is not None]
                    for s_ in srcs:
                        if s_.get("range", 0) > 30:
                            continue
                        if s_["kind"] in HUMAN_KINDS:
                            hum += 1
                            hum_hit += bool(hit(s_))
                        if s_["kind"] in ("voice", "weak_voice"):
                            vo += 1
                            vo_hit += bool(hit(s_))
                    t.append(r["t_" + who] / r["audio_s"])
                res[who] = {"human_recall": hum_hit / max(hum, 1), "n_human": hum, "voice_recall": vo_hit / max(vo, 1),
                            "n_voice": vo, "flags": flags, "precision": tp / flags if flags else float("nan"),
                            "false_per_scene": (flags - tp) / max(len(R), 1),
                            "brg_err_med": float(np.median(errs)) if errs else float("nan"),
                            "brg_err_p90": float(np.percentile(errs, 90)) if errs else float("nan"),
                            "rtf": float(np.mean(t)), "n_scenes": len(R)}
            out[f"{fam}|{lv}"] = res
    return out


def cmd_synth(args):
    path = SYNTH.format(fam=args.family) % VERSION
    done = set()
    if os.path.exists(path):
        done = {(r["seed"], r.get("noise_x", r.get("ambient"))) for r in map(json.loads, open(path))}
    levels = [1, 4, 10] if args.family == "A" else [45, 60, 70]
    t0 = time.time()
    with open(path, "a") as fh:
        for lv in levels:
            for seed in args.seeds:
                if (seed, lv) in done:
                    continue
                if time.time() - t0 > args.budget_s:
                    print("budget reached; rerun to continue")
                    return
                rows = synth_family_a(seed, lv) if args.family == "A" else synth_family_b(seed, lv)
                for r in rows if isinstance(rows, list) else [rows]:
                    fh.write(json.dumps(r) + "\n")
                fh.flush()
                print(f"family {args.family} seed {seed} level {lv} done ({time.time() - t0:.0f} s)", flush=True)
    print("all done")


# =========================================================================== report
LEAD = 0.4                       # acoustic.agent.HUMAN_LEAD (not imported: report runs in .venv)
CLASSES = ["voice", "shout", "knocking", "dog", "engine", "debris", "noise"]


def _words(t):
    """Lower-case words; screams/interjections (ah, aah, ahhh, oh, uh) dropped and immediate
    repeats collapsed, so a scream rendered as one 'AHHHH' or ten 'Ah!' scores the same."""
    w = [x for x in re.sub(r"[^a-z' ]+", " ", (t or "").lower()).split() if not re.fullmatch(r"a+h+|aa+|o+h+|u+h+", x)]
    return [x for i, x in enumerate(w) if i == 0 or x != w[i - 1]]


def wer(ref, hyp):
    r, h = _words(ref), _words(hyp)
    d = np.arange(len(h) + 1)
    for i in range(1, len(r) + 1):
        prev, d[0] = d.copy(), i
        for j in range(1, len(h) + 1):
            d[j] = min(prev[j] + 1, d[j - 1] + 1, prev[j - 1] + (r[i - 1] != h[j - 1]))
    return int(d[len(h)]), len(r)


def auc(pos, neg):
    """P(score_pos > score_neg) + 0.5 P(tie) (Mann-Whitney)."""
    pos, neg = np.asarray(pos, float), np.asarray(neg, float)
    if not len(pos) or not len(neg):
        return float("nan")
    gt = (pos[:, None] > neg[None, :]).mean()
    eq = (pos[:, None] == neg[None, :]).mean()
    return float(gt + 0.5 * eq)


def prf(y, p):
    y, p = np.asarray(y, bool), np.asarray(p, bool)
    tp, fp, fn = int(np.sum(y & p)), int(np.sum(~y & p)), int(np.sum(y & ~p))
    pr = tp / (tp + fp) if tp + fp else float("nan")
    rc = tp / (tp + fn) if tp + fn else float("nan")
    f1 = 2 * pr * rc / (pr + rc) if pr == pr and rc == rc and pr + rc else float("nan")
    return {"tp": tp, "fp": fp, "fn": fn, "tn": int(np.sum(~y & ~p)), "precision": pr, "recall": rc, "f1": f1}


def load_joined():
    man = {c["id"]: c for c in json.load(open(MANIFEST))}
    if os.path.exists(RESULTS_FV):
        for line in open(RESULTS_FV):
            r = json.loads(line)
            if r["id"] in man:
                man[r["id"]].update({k: v for k, v in r.items() if k != "id"})
    rows = []
    for line in open(RESULTS):
        r = json.loads(line)
        r["human_score"] = r.pop("human")            # agent score; "human" stays the 0/1 label
        if r["id"] in man:
            rows.append(dict(man[r["id"]], **r))
    return rows


def group_key(r):
    st = r["set"]
    if st in ("esc", "so", "fv"):
        return st
    if st in ("esc_mix", "das_mix", "fv_mix"):
        return f"{st}|{r['noise']['kind']}|{r['noise']['snr_db']:+d}"
    if st in ("dws", "so_cond"):
        return f"{st}|{r['cond']}"
    return st


def detection(rows):
    y = [r["human"] for r in rows]
    cls = prf(y, [r["cls"] in HUMAN_CLASSES for r in rows])
    agt = prf(y, [r["human_score"] >= LEAD for r in rows])
    return {"n": len(rows), "n_pos": int(sum(y)), "n_neg": len(y) - int(sum(y)), "class": cls, "agent": agt,
            "auc_priority": auc([r["priority"] for r in rows if r["human"]], [r["priority"] for r in rows if not r["human"]]),
            "auc_human": auc([r["human_score"] for r in rows if r["human"]],
                             [r["human_score"] for r in rows if not r["human"]]),
            "snr_db": float(np.mean([r["snr_db"] for r in rows])) if rows and rows[0].get("snr_db") is not None else None,
            "by_label": {lab: {"n": len(v), "flag_class": float(np.mean([x["cls"] in HUMAN_CLASSES for x in v])),
                               "flag_agent": float(np.mean([x["human_score"] >= LEAD for x in v])),
                               "pred": dict(sorted(((c, sum(x["cls"] == c for x in v)) for c in CLASSES
                                                    if any(x["cls"] == c for x in v)), key=lambda t: -t[1]))}
                         for lab in sorted({r["label"] for r in rows})
                         for v in [[x for x in rows if x["label"] == lab]]}}


def asr_summary(rows):
    ref = {(r["file"], r["fine"]): r for r in rows if r["set"] == "asr_ref"}
    out = {}
    for r in rows:
        if not r.get("asr") or r["set"] == "asr_ref":
            continue
        g = r["set"] + "|" + (f"{r['noise']['kind']}{r['noise']['snr_db']:+d}" if r.get("noise") else r.get("cond", ""))
        rf = ref[(r["file"], r["fine"])]
        rt = rf.get("text_ungated") or rf["text"]
        e_g, n = wer(rt, r["text"])
        e_u, _ = wer(rt, r.get("text_ungated", r["text"]))
        help_ref = bool(re.search(r"\bhelp", rt.lower()))
        o = out.setdefault(g, {"n": 0, "err_gated": 0, "err_ungated": 0, "ref_words": 0, "help_ref": 0,
                               "help_hit_gated": 0, "help_hit_ungated": 0, "help_false": 0, "vad": [], "snr": [],
                               "gated_out": 0, "examples": [], "seg_wer": []})
        o["seg_wer"].append(round(e_u / max(n, 1), 3))
        o["n"] += 1
        o["err_gated"] += e_g
        o["err_ungated"] += e_u
        o["ref_words"] += n
        o["vad"].append(r["vad"])
        o["gated_out"] += int(r["vad"] < 0.15)
        if r.get("snr_db") is not None:
            o["snr"].append(r["snr_db"])
        hit_g = bool(re.search(r"\bhelp", r["text"].lower()))
        hit_u = bool(re.search(r"\bhelp", r.get("text_ungated", r["text"]).lower()))
        if help_ref:
            o["help_ref"] += 1
            o["help_hit_gated"] += hit_g
            o["help_hit_ungated"] += hit_u
        else:
            o["help_false"] += hit_u
        if len(o["examples"]) < 2:
            o["examples"].append({"id": r["id"], "ref": rt[:160], "hyp": r.get("text_ungated", r["text"])[:160]})
    for o in out.values():
        o["wer_gated"] = o["err_gated"] / max(o["ref_words"], 1)
        o["wer_ungated"] = o["err_ungated"] / max(o["ref_words"], 1)
        o["wer_median"] = float(np.median(o["seg_wer"]))
        o["vad_mean"] = float(np.mean(o.pop("vad")))
        sn = o.pop("snr")
        o["snr_db"] = float(np.mean(sn)) if sn else None
    return out


def cmd_report(args):
    rows = load_joined()
    groups = {}
    for r in rows:
        if not r.get("asr"):
            groups.setdefault(group_key(r), []).append(r)
    det = {g: detection(v) for g, v in sorted(groups.items())}
    det["ALL_clean"] = detection(groups["esc"] + groups["so"])
    asr = asr_summary(rows)
    conf = {}
    for st in ("esc", "so", "dws"):
        sel = [r for r in rows if r["set"] == st and not r.get("asr")]
        labs = sorted({r["fine"] for r in sel}, key=lambda f: (-max(x["human"] for x in sel if x["fine"] == f), f))
        conf[st] = {"rows": labs, "cols": CLASSES,
                    "m": [[sum(1 for r in sel if r["fine"] == f and r["cls"] == c) for c in CLASSES] for f in labs]}
    # teammate (F.V.) rule vs ours on identical clips
    cmp_ = {}
    for g, v in sorted(groups.items()):
        if all("fv_flag" in r for r in v):
            y = [r["human"] for r in v]
            cmp_[g] = {"n": len(v), "ours_agent": prf(y, [r["human_score"] >= LEAD for r in v]),
                       "ours_class": prf(y, [r["cls"] in HUMAN_CLASSES for r in v]),
                       "fv": prf(y, [r["fv_flag"] for r in v]),
                       "fv_auc": auc([r["fv_max"] for r in v if r["human"]], [r["fv_max"] for r in v if not r["human"]]),
                       "ours_auc": auc([r["priority"] for r in v if r["human"]],
                                       [r["priority"] for r in v if not r["human"]]),
                       "by_label": {lab: [round(float(np.mean([x["human_score"] >= LEAD for x in w])), 2),
                                          round(float(np.mean([x["fv_flag"] for x in w])), 2), len(w)]
                                    for lab in sorted({r["fine"] for r in v})
                                    for w in [[x for x in v if x["fine"] == lab]]}}
    kw = {}
    for g in ("fv", "fv_mix|high|+0", "fv_mix|high|-10"):
        sel = [r for r in groups.get(g, []) if r.get("kw_expected")]
        if sel:
            kw[g] = {"n": len(sel), "help_gated": sum(bool(re.search(r"\bhelp", r["text"].lower())) for r in sel),
                     "help_ungated": sum(bool(re.search(r"\bhelp", r.get("text_ungated", "").lower())) for r in sel),
                     "examples": [(r["fine"] + ":" + r["id"].split("|")[-1], r.get("text_ungated", "")[:80]) for r in sel[:13]]}
    wi = {}
    wpath = os.path.join(DATA, f"whatif-{VERSION}.jsonl")
    if os.path.exists(wpath):
        byid = {r["id"]: r for r in rows}
        W = [json.loads(line) for line in open(wpath)]
        for g in sorted({group_key(byid[w["id"]]) for w in W if w["id"] in byid}):
            ws = [w for w in W if w["id"] in byid and group_key(byid[w["id"]]) == g]
            y = [byid[w["id"]]["human"] for w in ws]
            wi[g] = {v: {"n": len(ws), "agent": prf(y, [w[v]["human"] >= LEAD for w in ws]),
                         "class": prf(y, [w[v]["cls"] in HUMAN_CLASSES for w in ws]),
                         "auc": auc([w[v]["priority"] for w, t in zip(ws, y) if t],
                                    [w[v]["priority"] for w, t in zip(ws, y) if not t])} for v in WHATIF}
    raw = {}
    rpath = os.path.join(DATA, f"whisper_raw-{VERSION}.jsonl")
    if os.path.exists(rpath):
        ref = {(r["file"], r["fine"]): (r.get("text_ungated") or r["text"]) for r in rows if r["set"] == "asr_ref"}
        man = {c["id"]: c for c in json.load(open(MANIFEST))}
        for line in open(rpath):
            q = json.loads(line)
            c = man[q["id"]]
            g = q["id"].split("|")[0] + "|" + q["id"].split("|")[1]
            e, n = wer(ref[(c["file"], c["fine"])], q["raw"])
            o = raw.setdefault(g, {"n": 0, "err": 0, "words": 0, "help": 0, "examples": []})
            o["n"] += 1
            o["err"] += e
            o["words"] += n
            o["help"] += bool(re.search(r"\bhelp", q["raw"].lower()))
            o["examples"].append(q["raw"][:120])
        for o in raw.values():
            o["wer"] = o["err"] / max(o["words"], 1)
    summ = {"detection": det, "asr": asr, "confusion": conf, "fv_compare": cmp_, "fv_keywords": kw,
            "whatif": wi, "whisper_raw": raw}
    json.dump(summ, open(os.path.join(DATA, "summary.json"), "w"), indent=1, default=float)
    summ["code_version"] = VERSION
    summ["synth"] = synth_summary()
    # other commits evaluated on the clean sets (VALIDATE_REV=<rev> run --only ...)
    import hashlib
    vers = {}
    sdir = os.path.join(DATA, "code_snapshot")
    for rev in sorted(os.listdir(sdir)) if os.path.isdir(sdir) else []:
        d = os.path.join(sdir, rev, "acoustic_snap")
        if os.path.exists(os.path.join(d, "models.py")):
            h = hashlib.sha1()
            for f in ("models.py", "agent.py"):
                h.update(open(os.path.join(d, f), "rb").read())
            vers[h.hexdigest()[:8]] = rev
    man = {c["id"]: c for c in json.load(open(MANIFEST))}
    summ["versions"] = {}
    for hsh, rev in vers.items():
        path = os.path.join(DATA, f"results-{hsh}.jsonl")
        if not os.path.exists(path):
            continue
        rr = []
        for line in open(path):
            r = json.loads(line)
            r["human_score"] = r.pop("human")
            rr.append(dict(man[r["id"]], **r))
        summ["versions"][rev] = {g: detection([r for r in rr if r["set"] == g]) for g in ("esc", "so", "fv")
                                 if any(r["set"] == g for r in rr)}
    print(f"code under test: acoustic/models.py + agent.py sha1 {VERSION}; {len(rows)} clips")
    print_tables(summ)
    if args.plot:
        plot(summ, rows, args.plot)


def _f(v, k=2):
    return "-" if v is None or v != v else f"{v:.{k}f}"


def print_tables(summ):
    print("\n## Detection (class = YAMNet group in voice/shout/knocking; agent = fallback human >= 0.4)")
    print("| group | n (pos/neg) | SNR dB | class P / R / F1 | agent P / R / F1 | AUC priority |")
    print("|---|---|---|---|---|---|")
    for g, d in summ["detection"].items():
        c, a = d["class"], d["agent"]
        print(f"| {g} | {d['n']} ({d['n_pos']}/{d['n_neg']}) | {_f(d['snr_db'], 1)} | "
              f"{_f(c['precision'])} / {_f(c['recall'])} / {_f(c['f1'])} | "
              f"{_f(a['precision'])} / {_f(a['recall'])} / {_f(a['f1'])} | {_f(d['auc_priority'])} |")
    print("\n## Per-label share flagged human (agent), and top predicted classes")
    for g in ("esc", "so", "dws|90db-room1-1m-low", "das_mix|high|+0", "das_mix|high|-10"):
        if g in summ["detection"]:
            print(f"- {g}: " + "; ".join(f"{lab} {v['flag_agent']:.2f} (n={v['n']}, {v['pred']})"
                                       for lab, v in summ["detection"][g]["by_label"].items()))
    print("\n## ASR (reference = Whisper base.en on the digital source segment)")
    print("| group | segs | SNR dB | mean VAD | gated out | WER gated | WER ungated | median seg WER | help hit (ungated) | help false |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for g, o in sorted(summ["asr"].items()):
        print(f"| {g} | {o['n']} | {_f(o['snr_db'], 1)} | {o['vad_mean']:.2f} | {o['gated_out']} | "
              f"{o['wer_gated']:.2f} | {o['wer_ungated']:.2f} | {o['wer_median']:.2f} | {o['help_hit_ungated']}/{o['help_ref']} | {o['help_false']} |")
    print("\n## Ours vs teammate (F.V.) rule on identical clips: P / R / F1, AUC")
    print("| group | n | ours agent | ours class | F.V. rule | AUC ours / F.V. |")
    print("|---|---|---|---|---|---|")
    for g, d in summ["fv_compare"].items():
        o, c, f = d["ours_agent"], d["ours_class"], d["fv"]
        print(f"| {g} | {d['n']} | {_f(o['precision'])} / {_f(o['recall'])} / {_f(o['f1'])} | "
              f"{_f(c['precision'])} / {_f(c['recall'])} / {_f(c['f1'])} | "
              f"{_f(f['precision'])} / {_f(f['recall'])} / {_f(f['f1'])} | {_f(d['ours_auc'])} / {_f(d['fv_auc'])} |")
    for g in ("esc", "so", "fv"):
        if g in summ["fv_compare"]:
            print(f"- {g} share flagged [ours, F.V., n]: {summ['fv_compare'][g]['by_label']}")
    print("\n## 'help' keyword on teammate voice clips:", json.dumps(summ["fv_keywords"], indent=1))
    if summ.get("versions"):
        print("\n## Code versions on the clean sets (agent P / R / F1, AUC; per-label share flagged)")
        for rev, dd in summ["versions"].items():
            for g, d in dd.items():
                a = d["agent"]
                print(f"- {rev} {g}: {_f(a['precision'])} / {_f(a['recall'])} / {_f(a['f1'])}, AUC {_f(d['auc_priority'])}; "
                      + ", ".join(f"{k} {v['flag_agent']:.2f}" for k, v in d["by_label"].items()))
    if summ.get("synth"):
        print("\n## Synthetic head-to-head (A = our 3-drone scenes, B = teammate's 1-array scenes)")
        print("| family / level | pipeline | scenes | human recall | voice recall | flags | precision | false/scene | "
              "brg err med / p90 | real-time factor |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        for g, res in summ["synth"].items():
            for who, d in res.items():
                print(f"| {g} | {who} | {d['n_scenes']} | {d['human_recall']:.2f} (n={d['n_human']}) | "
                      f"{d['voice_recall']:.2f} (n={d['n_voice']}) | {d['flags']} | {_f(d['precision'])} | "
                      f"{d['false_per_scene']:.2f} | {_f(d['brg_err_med'], 1)} / {_f(d['brg_err_p90'], 1)} | {d['rtf']:.3f} |")
    if summ["whatif"]:
        print("\n## What-if class groups (agent P / R / F1, AUC priority)")
        print("| group | n | " + " | ".join(WHATIF) + " |")
        print("|---|---|" + "---|" * len(WHATIF))
        for g, d in summ["whatif"].items():
            print(f"| {g} | {d['baseline']['n']} | " + " | ".join(
                f"{_f(d[v]['agent']['precision'])} / {_f(d[v]['agent']['recall'])} / {_f(d[v]['agent']['f1'])}, "
                f"{_f(d[v]['auc'])}" for v in WHATIF) + " |")
    if summ["whisper_raw"]:
        print("\n## Whisper without the hallucination filter / VAD gate")
        for g, o in summ["whisper_raw"].items():
            print(f"- {g}: n={o['n']} WER {o['wer']:.2f}, 'help' in {o['help']}; e.g. {o['examples'][:2]}")
    for st, c in summ["confusion"].items():
        print(f"\n## Confusion {st} (rows true, cols predicted {c['cols']})")
        for f, m in zip(c["rows"], c["m"]):
            print(f"  {f:18s} {m}")


def plot(summ, rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"           # categorical slots 1-3 (validated set)
    INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(3, 2, figsize=(13, 15.5), facecolor="#fcfcfb")
    det = summ["detection"]

    # (a) ESC-50 clean confusion, row-normalised
    a = ax[0, 0]
    c = summ["confusion"]["esc"]
    m = np.array(c["m"], float)
    mn = m / np.maximum(m.sum(axis=1, keepdims=True), 1)
    a.imshow(mn, cmap="Blues", vmin=0, vmax=1, aspect="auto")
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            if m[i, j]:
                a.text(j, i, int(m[i, j]), ha="center", va="center", fontsize=8, color="white" if mn[i, j] > 0.55 else INK)
    a.set_xticks(range(len(c["cols"])), c["cols"], rotation=30)
    pos = {k for k, v in ESC_CLASSES.items() if v[0]}
    a.set_yticks(range(len(c["rows"])), [("* " if f in pos else "") + f for f in c["rows"]])
    a.set_title("(a) ESC-50 clean, 12 clips/class: predicted pipeline class\n(* = should be flagged human)",
                loc="left", fontsize=10, color=INK)

    # (b) detection vs SNR: controlled mixes (lines) + real rotors-on recordings (hollow)
    a = ax[0, 1]
    for kind, ls in (("high", "-"), ("low", ":")):
        ks = sorted([k for k in det if k.startswith(f"das_mix|{kind}|")], key=lambda k: int(k.split("|")[2]))
        if not ks:
            continue
        x = [int(k.split("|")[2]) for k in ks]
        for key, col, lab in (("recall", C1, "human recall"), ("fpr", C2, "false-positive rate")):
            y = [det[k]["agent"]["recall"] if key == "recall" else
                 det[k]["agent"]["fp"] / max(det[k]["n_neg"], 1) for k in ks]
            a.plot(x, y, ls, color=col, lw=2, marker="o", ms=5,
                   label=f"{lab}, mix thr-{kind}" if kind == "high" or key == "recall" else None)
        a.plot(x, [det[k]["auc_priority"] for k in ks], ls, color=C3, lw=2, marker="s", ms=5,
               label=f"priority AUC, mix thr-{kind}" if kind == "high" else None)
    d0 = det.get("so")
    if d0:
        a.scatter([15], [d0["agent"]["recall"]], s=60, color=C1, zorder=5)
        a.scatter([15], [d0["agent"]["fp"] / max(d0["n_neg"], 1)], s=60, color=C2, zorder=5)
        a.scatter([15], [d0["auc_priority"]], s=60, color=C3, marker="s", zorder=5)
        a.annotate("rotors off\n(source-only)", (15, 0.04), ha="center", fontsize=8, color=MUTED)
    for k in [k for k in det if k.startswith("dws|")]:
        sn = det[k]["snr_db"]
        a.scatter([sn], [det[k]["agent"]["recall"]], s=50, facecolors="none", edgecolors=C1, lw=1.5, zorder=4)
        a.scatter([sn], [det[k]["agent"]["fp"] / max(det[k]["n_neg"], 1)], s=50, facecolors="none",
                  edgecolors=C2, lw=1.5, zorder=4)
    a.scatter([], [], s=50, facecolors="none", edgecolors=MUTED, label="real rotors-on recordings (DWS)")
    a.set_xlabel("SNR at the mic, dB (segment power vs rotor noise power)")
    a.set_ylabel("rate")
    a.set_ylim(-0.03, 1.03)
    a.grid(color=GRID, lw=0.6)
    a.legend(fontsize=7.5, frameon=False, loc="upper center", bbox_to_anchor=(0.45, -0.13), ncol=2)
    a.set_title("(b) DroneAudioSet 5 s windows: agent human flag (>= 0.4) vs SNR", loc="left", fontsize=10, color=INK)

    # (c) ASR vs SNR
    a = ax[1, 0]
    asr = summ["asr"]
    ks = sorted([k for k in asr if k.startswith("asr_das_mix|high")], key=lambda k: int(k.split("high")[1]))
    x = [int(k.split("high")[1]) for k in ks]
    a.plot(x, [min(asr[k]["wer_median"], 1.5) for k in ks], "-", color=C1, lw=2, marker="o", ms=5,
           label="median segment WER vs clean-source transcript (no VAD gate)")
    a.plot(x, [asr[k]["help_hit_ungated"] / max(asr[k]["help_ref"], 1) for k in ks], "-", color=C2, lw=2,
           marker="s", ms=5, label="'help' recovered (segments whose reference says it)")
    a.plot(x, [1 - asr[k]["gated_out"] / asr[k]["n"] for k in ks], "--", color=C3, lw=2, marker="^", ms=5,
           label="segments passing the VAD >= 0.15 gate")
    if "asr_so|90db-room1-1m" in asr:
        o = asr["asr_so|90db-room1-1m"]
        a.scatter([15], [o["wer_median"]], s=60, color=C1, zorder=5)
        a.scatter([15], [o["help_hit_ungated"] / max(o["help_ref"], 1)], s=60, color=C2, marker="s", zorder=5)
        a.annotate("rotors off", (15, -0.08), ha="center", fontsize=8, color=MUTED)
    for k in [k for k in asr if k.startswith("asr_dws|")]:
        o = asr[k]
        a.scatter([o["snr_db"]], [min(o["wer_median"], 1.5)], s=50, facecolors="none", edgecolors=C1, lw=1.5)
        if o["help_ref"]:
            a.scatter([o["snr_db"]], [o["help_hit_ungated"] / o["help_ref"]], s=50, marker="s",
                      facecolors="none", edgecolors=C2, lw=1.5)
    a.scatter([], [], s=50, facecolors="none", edgecolors=MUTED, label="real rotors-on recordings (DWS)")
    a.set_xlabel("SNR at the mic, dB")
    a.set_ylabel("rate (WER clipped at 1.5)")
    a.set_ylim(-0.12, 1.55)
    a.grid(color=GRID, lw=0.6)
    a.legend(fontsize=7.5, frameon=False, loc="upper center", bbox_to_anchor=(0.45, -0.13), ncol=2)
    a.set_title("(c) faster-whisper base.en on ~30 s speech/scream/help segments", loc="left", fontsize=10, color=INK)

    # (d) agent priority, positives vs negatives, per set
    a = ax[1, 1]
    sets = [("esc", "ESC-50 clean"), ("so", "DAS rotors off"), ("das_mix|high|+0", "mix 0 dB"),
            ("das_mix|high|-10", "mix -10 dB")]
    rng = np.random.default_rng(0)
    for i, (g, name) in enumerate(sets):
        sel = [r for r in rows if not r.get("asr") and group_key(r) == g]
        for off, hv, col in ((-0.17, 1, C1), (0.17, 0, C2)):
            y = np.array([r["priority"] for r in sel if r["human"] == hv], float)
            a.scatter(i + off + rng.uniform(-0.09, 0.09, len(y)), y, s=9, color=col, alpha=0.55, lw=0)
            if len(y):
                a.plot([i + off - 0.13, i + off + 0.13], [np.median(y)] * 2, color=INK, lw=2)
        if g in det:
            a.text(i, 103, f"AUC {det[g]['auc_priority']:.2f}", ha="center", fontsize=8, color=INK)
    a.scatter([], [], color=C1, label="human (speech/scream, cry, knock, cough, breathing)")
    a.scatter([], [], color=C2, label="non-human (dog, engine, rotor, ambient, debris)")
    a.set_xticks(range(len(sets)), [n for _, n in sets])
    a.set_ylabel("agent clip priority (0-100, neutral DoA)")
    a.set_ylim(-3, 110)
    a.grid(axis="y", color=GRID, lw=0.6)
    a.legend(fontsize=7.5, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2)
    a.set_title("(d) agent fallback scorer: clip priority, human vs non-human (bar = median)",
                loc="left", fontsize=10, color=INK)
    # (e)/(f) head-to-head with the teammate's pipeline: real clips (F1) and synthetic scenes
    a = ax[2, 0]
    cmpd = summ.get("fv_compare", {})
    gs = [("fv", "teammate clips"), ("fv_mix|high|+0", "  + rotor 0 dB"), ("esc", "ESC-50"),
          ("esc_mix|high|+0", "  + rotor 0 dB"), ("so", "DAS rotors off"), ("das_mix|high|+0", "  + rotor 0 dB"),
          ("dws|90db-room1-1m-low", "DAS rotors on")]
    gs = [(g, n) for g, n in gs if g in cmpd]
    xx = np.arange(len(gs))
    for off, key, col, lab in ((-0.2, "ours_agent", C1, "ours (agent human >= 0.4)"), (0.2, "fv", C2, "teammate rule")):
        v = [np.nan_to_num(cmpd[g][key]["f1"]) for g, _ in gs]
        a.bar(xx + off, v, 0.36, color=col, label=lab)
    a.set_xticks(xx, [n for _, n in gs], rotation=25, ha="right")
    a.set_ylabel("F1, human vs non-human")
    a.set_ylim(0, 1)
    a.grid(axis="y", color=GRID, lw=0.6)
    a.legend(fontsize=7.5, frameon=False, loc="upper right")
    a.set_title("(e) same real clips, both classifiers (single channel)", loc="left", fontsize=10, color=INK)
    a = ax[2, 1]
    sy = summ.get("synth", {})
    cols = [k for k in sy if not k.endswith("|all")]
    xx = np.arange(len(cols))
    for off, who, col, lab in ((-0.2, "ours", C1, "ours"), (0.2, "fv", C2, "teammate")):
        a.bar(xx + off, [sy[k][who]["human_recall"] for k in cols], 0.36, color=col, label=f"{lab}: human-source recall")
        a.plot(xx + off, [min(sy[k][who]["false_per_scene"], 3) / 3 for k in cols], "D", color=INK if who == "ours" else MUTED,
               ms=6, label=f"{lab}: false flags per scene / 3")
    a.set_xticks(xx, [("A noise x" if k.startswith("A") else "B ambient ") + k.split("|")[1] + ("" if k.startswith("A") else " dB")
                      for k in cols], rotation=25, ha="right")
    a.set_ylim(0, 1.05)
    a.grid(axis="y", color=GRID, lw=0.6)
    a.legend(fontsize=7, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2)
    a.set_title("(f) synthetic scenes: A = our 3-drone site, B = teammate's 1-array sites", loc="left", fontsize=10,
                color=INK)
    fig.suptitle("Larpverse acoustic front end on real audio (ESC-50, DroneAudioSet mic1)", x=0.01, ha="left",
                 fontsize=12, color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(path, dpi=110, facecolor=fig.get_facecolor())
    print("wrote", path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("index")
    a.add_argument("--config", default="", help="shard prefix, e.g. drone-with-source")
    a.set_defaults(fn=cmd_index)
    a = sub.add_parser("fetch")
    a.add_argument("--esc-per-class", type=int, default=12)
    a.add_argument("--das", action="store_true", help="also fetch the chosen DroneAudioSet rows")
    a.set_defaults(fn=cmd_fetch)
    a = sub.add_parser("run")
    a.add_argument("--budget-s", type=float, default=240.0, help="stop after this many seconds (resumable)")
    a.add_argument("--only", default="", help="regex on clip id")
    a.add_argument("--rebuild", action="store_true", help="rebuild the clip manifest")
    a.add_argument("--fv", action="store_true", help="score with the teammate's (F.V.) classifier rule instead")
    a.set_defaults(fn=cmd_run)
    a = sub.add_parser("whatif")
    a.add_argument("--budget-s", type=float, default=240.0)
    a.add_argument("--whisper-raw", action="store_true")
    a.set_defaults(fn=cmd_whatif)
    a = sub.add_parser("synth")
    a.add_argument("--family", choices=["A", "B"], required=True)
    a.add_argument("--seeds", type=int, nargs="+", default=[3, 7, 11, 19, 23, 29, 31, 37])
    a.add_argument("--budget-s", type=float, default=240.0)
    a.set_defaults(fn=cmd_synth)
    a = sub.add_parser("report")
    a.add_argument("--plot", default=os.path.join(ROOT, "docs", "acoustic_real_validation.png"))
    a.set_defaults(fn=cmd_report)
    args = ap.parse_args(argv)
    args.fn(args)


if __name__ == "__main__":
    main()

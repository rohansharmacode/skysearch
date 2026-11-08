"""SYNTHETIC acoustic scene built from the shared scenario (sim/scenario.py): 3x3 rubble
piles, three landed drones with 6-mic arrays, pyroomacoustics free field, 16 kHz.

Frames follow contracts/INTERFACES.md: metres, x east, y north, bearings CCW from +x in degrees.
Piles A1..C3: column A/B/C west->east, row 1..3 north->south (~22 m spacing, jittered centres).

build_scene() is the ONLY acoustic code that reads scenario.truth (source kinds, positions,
levels, TTS text). It returns a Scene with public geometry and mic signals, plus a separate
truth summary used only by run.py's evaluation printout. The analysis and agent never see it.
"""
from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass, field

import numpy as np
from scipy import signal as sps

FS = 16000
C_SOUND = 343.0
SPACING = 22.0
PILES = [f"{c}{r}" for r in (1, 2, 3) for c in "ABC"]
DRONE_RADIUS = 8.6           # landed drones sit this far from the start pile centre
MIC_RADIUS = 0.15            # 0.3 m aperture
N_MICS = 6
DURATION = 26.0              # listening window, seconds (rotors off)

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(HERE, "data")
MODEL_DIR = os.path.join(HERE, "models")
PIPER_VOICE = os.path.join(MODEL_DIR, "en_US-lessac-low.onnx")
DOG_WAV = os.path.join(DATA_DIR, "esc50", "1-100032-A-0.wav")   # ESC-50, CC BY-NC 3.0

def pile_xy(pid: str) -> tuple[float, float]:
    """Nominal (un-jittered) grid centre; the shared scenario jitters centres by up to 1.5 m."""
    col = "ABC".index(pid[0].upper())
    row = int(pid[1]) - 1
    return (-SPACING + SPACING * col, SPACING - SPACING * row)


def drone_positions(center_xy) -> dict[str, tuple[float, float]]:
    """Default landing spots: three points DRONE_RADIUS from a pile centre at 120 deg spacing."""
    cx, cy = center_xy
    out = {}
    for i, ang in enumerate((90.0, 210.0, 330.0)):
        a = np.radians(ang)
        out[f"d{i + 1}"] = (round(float(cx + DRONE_RADIUS * np.cos(a)), 2), round(float(cy + DRONE_RADIUS * np.sin(a)), 2))
    return out


def mic_offsets(n=N_MICS, radius=MIC_RADIUS) -> np.ndarray:
    """(2, n) mic offsets relative to the array centre (uniform circular array)."""
    a = 2 * np.pi * np.arange(n) / n
    return np.vstack([radius * np.cos(a), radius * np.sin(a)])


# --------------------------------------------------------------------------- source synthesis
def _rms_norm(x):
    r = np.sqrt(np.mean(x ** 2)) + 1e-12
    return x / r


def tts(text: str) -> np.ndarray:
    """Piper TTS (en_US-lessac-low, 16 kHz). Cached as wav in acoustic/data/tts/."""
    import soundfile as sf
    d = os.path.join(DATA_DIR, "tts")
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, hashlib.sha1(text.encode()).hexdigest()[:12] + ".wav")
    if os.path.exists(path):
        x, _ = sf.read(path, dtype="float32")
        return x
    from piper import PiperVoice
    voice = PiperVoice.load(PIPER_VOICE)
    chunks = list(voice.synthesize(text))
    x = np.concatenate([c.audio_float_array for c in chunks]).astype(np.float32)
    assert chunks[0].sample_rate == FS
    sf.write(path, x, FS)
    return x


def knocking(rng, dur=7.0) -> np.ndarray:
    """Patterned knocking: groups of three knocks (0.22 s apart) every ~2 s, on a slab/pipe."""
    n = int(dur * FS)
    x = np.zeros(n)
    t = np.arange(int(0.12 * FS)) / FS
    modes = [(180, 0.035), (430, 0.025), (960, 0.015), (2100, 0.006)]
    knock = sum(np.exp(-t / tau) * np.sin(2 * np.pi * f * t) * (1.0 / (1 + i)) for i, (f, tau) in enumerate(modes))
    knock[: int(0.004 * FS)] += rng.normal(0, 0.6, int(0.004 * FS))      # contact click
    g = 0.2
    while g < dur - 1.0:
        for k in range(3):
            s = int((g + 0.22 * k + rng.normal(0, 0.01)) * FS)
            if s + len(knock) < n:
                x[s: s + len(knock)] += knock * rng.uniform(0.8, 1.0)
        g += 2.0 + rng.uniform(-0.15, 0.15)
    return x


def generator_hum(rng, dur) -> np.ndarray:
    """Diesel generator: 50 Hz harmonic series, 25 Hz firing modulation, mechanical noise."""
    n = int(dur * FS)
    t = np.arange(n) / FS
    f0 = 50.0 * (1 + 0.002 * np.cumsum(rng.normal(0, 1, n)) / np.sqrt(n))
    ph = 2 * np.pi * np.cumsum(f0) / FS
    x = sum(np.sin(k * ph + rng.uniform(0, 2 * np.pi)) / k ** 0.7 for k in range(1, 40))
    x *= 1 + 0.3 * np.sin(2 * np.pi * 25 * t)
    b, a = sps.butter(2, [200 / (FS / 2), 2500 / (FS / 2)], "band")
    x = _rms_norm(x) + 0.5 * _rms_norm(sps.lfilter(b, a, rng.normal(0, 1, n)))
    return x


def debris(rng, dur=1.4) -> np.ndarray:
    """Debris settling: low rumble plus a cluster of gravel ticks and cracks, decaying."""
    n = int(dur * FS)
    env = np.exp(-np.arange(n) / (0.45 * FS))
    b, a = sps.butter(2, 250 / (FS / 2))
    x = 2.0 * sps.lfilter(b, a, rng.normal(0, 1, n)) * env
    for _ in range(int(rng.integers(25, 45))):
        s = int(rng.uniform(0, 0.8) * n)
        L = int(rng.integers(30, 200))
        if s + L < n:
            x[s: s + L] += rng.normal(0, 1, L) * np.exp(-np.arange(L) / (L / 4)) * rng.uniform(0.3, 1.5) * env[s]
    return x


def dog_bark(rng) -> np.ndarray:
    """One real dog clip (ESC-50 1-100032-A-0, Freesound 100032, CC BY-NC 3.0), else a synthetic bark."""
    if os.path.exists(DOG_WAV):
        import soundfile as sf
        x, fs = sf.read(DOG_WAV, dtype="float64")
        if x.ndim > 1:
            x = x.mean(axis=1)
        x = sps.resample_poly(x, 160, 441) if fs == 44100 else sps.resample_poly(x, FS, fs)
        e = np.convolve(np.abs(x), np.ones(800) / 800, "same")
        idx = np.where(e > 0.1 * e.max())[0]
        return x[max(0, idx[0] - 800): idx[-1] + 800]
    out = []
    for _ in range(4):    # synthetic fallback: harmonic burst with pitch drop
        L = int(0.18 * FS)
        t = np.arange(L) / FS
        f = 550 - 250 * t / t[-1]
        ph = 2 * np.pi * np.cumsum(f) / FS
        b = sum(np.sin(k * ph) / k for k in range(1, 12)) * np.exp(-t / 0.06)
        out += [b, np.zeros(int(rng.uniform(0.25, 0.5) * FS))]
    return np.concatenate(out)


# --------------------------------------------------------------------------- scene
@dataclass
class Scene:
    """What the analysis may see: public geometry and the recorded mic signals. No truth."""
    seed: int
    start_pile: str
    fs: int
    duration: float
    piles: dict                       # name -> (x, y) public pile centres (from the LiDAR map)
    drones: dict                      # id -> (x, y)
    mic_offsets: np.ndarray           # (2, M)
    signals: dict                     # drone id -> (M, N) float32


# per sim.scenario sound kind: emission RMS at 1 m (before level_db), rubble depth (m), repeats
KINDS = {
    "voice":      dict(level=1.6, depth=0.4),
    "weak_voice": dict(level=1.3, depth=1.0),
    "knock":      dict(level=1.1, depth=0.8),
    "dog":        dict(level=1.6, depth=0.0),
    "generator":  dict(level=0.3, depth=0.0),
    "settling":   dict(level=0.9, depth=0.3),
}


def through_rubble(x, depth):
    """Low-pass and attenuate with burial depth (crude, labelled synthetic)."""
    if depth <= 0:
        return x
    fc = 4000.0 / (1.0 + 1.6 * depth)
    b, a = sps.butter(2, fc / (FS / 2))
    return sps.lfilter(b, a, x) * 10 ** (-3.0 * depth / 20)


def _place(track, clip, t0):
    s = int(t0 * FS)
    e = min(len(track), s + len(clip))
    if e > s:
        track[s:e] += clip[: e - s]


def public_piles(scenario) -> dict:
    return {p.name: (float(p.x), float(p.y)) for p in scenario.piles}


def nearest_pile(piles: dict, xy) -> str:
    return min(piles, key=lambda n: np.hypot(piles[n][0] - xy[0], piles[n][1] - xy[1]))


def rotor_noise(rng, dur, bpf=None) -> np.ndarray:
    """Hovering multirotor: blade-pass comb (BPF ~120-200 Hz and harmonics, slow wobble) + broadband
    turbulence. Not from truth: our own drones' positions are public mission state."""
    n = int(dur * FS)
    t = np.arange(n) / FS
    f0 = (bpf or rng.uniform(120, 200)) * (1 + 0.01 * np.sin(2 * np.pi * 0.3 * t))
    ph = 2 * np.pi * np.cumsum(f0) / FS
    x = sum(np.sin(k * ph + rng.uniform(0, 6.28)) / k ** 0.8 for k in range(1, 16))
    b, a = sps.butter(2, [150 / (FS / 2), 3000 / (FS / 2)], "band")
    return _rms_norm(x) + 0.7 * _rms_norm(sps.lfilter(b, a, rng.normal(0, 1, n)))


def _load16(clip):
    """np.ndarray (16 kHz mono) or a WAV path (any rate, first channel) -> float 16 kHz."""
    if isinstance(clip, (str, os.PathLike)):
        import soundfile as sf
        x, fs = sf.read(str(clip), dtype="float64", always_2d=True)
        x = x[:, 0]
        if fs != FS:
            from math import gcd
            g = gcd(int(fs), FS)
            x = sps.resample_poly(x, FS // g, int(fs) // g)
        return x
    return np.asarray(clip, float)


def _active_rms(x):
    a = x[np.abs(x) > 1e-4]
    return float(np.sqrt(np.mean(a ** 2))) if len(a) else 1.0


def source_track(src: dict, n: int) -> np.ndarray:
    """One external source (live Isaac dummy / dummy-audio library) as a dry track at the 1 m level.

    src: {"xy": [x, y], "clip": ndarray|path, "kind": scene.KINDS key or other, "level_db": 0.0,
          "depth_m": 0.0 (apply through_rubble; 0 for clips that already carry rubble loss),
          "t0_s": 0.0 or [start, ...], "loop": False, "ref_rms": None (RMS of the dry parent clip, so
          a rubble variant keeps its loss; default the clip's own active RMS)}"""
    x = _load16(src["clip"])
    x = x / (src.get("ref_rms") or _active_rms(x))
    track = np.zeros(n)
    starts = src.get("t0_s", 0.0)
    starts = [float(starts)] if np.isscalar(starts) else [float(v) for v in starts]
    if src.get("loop"):
        period = len(x) / FS + float(src.get("gap_s", 0.0))
        starts = [t for s0 in starts for t in np.arange(s0, n / FS, period)]
    for t0 in starts:
        _place(track, x, t0)
    depth = float(src.get("depth_m") or 0.0)
    if depth > 0:
        track = through_rubble(track, depth)
    level = KINDS.get(src.get("kind"), {}).get("level", 1.0)
    return track * level * 10 ** (float(src.get("level_db", 0.0)) / 20)


def fast_simulate(room, n: int) -> np.ndarray:
    """pyroomacoustics room.simulate() for an anechoic room, done as one FFT per source and one
    inverse FFT per mic (the same RIRs from room.compute_rir, linear convolution, first n samples).
    Equal to room.simulate() to float precision, ~10x faster for 7 sources x 18 mics x 26 s."""
    room.compute_rir()
    sigs = [np.asarray(sv.signal, float) for sv in room.sources]
    L = max(len(r) for row in room.rir for r in row)
    nfft = int(2 ** np.ceil(np.log2(max(len(x) for x in sigs) + L)))
    S_ = [np.fft.rfft(x, nfft) for x in sigs]
    out = np.zeros((len(room.rir), n))
    for m, row in enumerate(room.rir):
        acc = np.zeros(nfft // 2 + 1, complex)
        for k, r in enumerate(row):
            acc += S_[k] * np.fft.rfft(r, nfft)
        y = np.fft.irfft(acc, nfft)[:n]
        out[m, :len(y)] = y
    return out


def build_scene(scenario, drones: dict | None = None, seed: int | None = None,
                duration=DURATION, noise_rms=4.0e-3, rotors: dict | None = None, rotor_level=2.0,
                sources: list | None = None):
    """Synthesise what the landed arrays hear. ONLY this function reads scenario.truth.

    drones: {id: (x, y)}; default three points DRONE_RADIUS from the start pile at 120 deg.
    rotors: {id: (x, y)} of our own drones hovering during the listen (blade-pass noise sources).
    sources: external sources (isaac/live.py dummies, assets/audio/dummies) that REPLACE the
    scenario's sounds; see source_track for the dict. The scenario then only gives pile geometry.
    Returns (Scene, truth_summary); the summary is for evaluation printouts only."""
    import pyroomacoustics as pra

    seed = scenario.seed if seed is None else seed
    rng = np.random.default_rng(seed + 1000)
    piles = public_piles(scenario)
    if drones is None:
        drones = drone_positions(piles[scenario.truth["start_pile"]])
    drones = {k: (float(v[0]), float(v[1])) for k, v in drones.items()}
    start_pile = nearest_pile(piles, np.mean(list(drones.values()), axis=0))
    offs = mic_offsets()
    n = int(duration * FS)

    sounds = [] if sources is not None else list(scenario.truth["sounds"])
    slots = list(np.array([0.5, 4.5, 8.5, 12.5]) + rng.uniform(0, 1.0, 4))
    rng.shuffle(slots)
    slot = dict(zip(["voice", "weak_voice", "knock", "dog"], slots))

    room = pra.AnechoicRoom(dim=2, fs=FS)
    truth = {}
    for snd in sounds:
        kind = snd["kind"]
        track = np.zeros(n)
        events = []
        if kind in ("voice", "weak_voice"):
            clip = _rms_norm(tts(snd.get("text") or "Help!").astype(float))
            for t0 in (slot[kind], slot[kind] + 11.5):
                _place(track, clip, t0); events.append(round(float(t0), 2))
        elif kind == "knock":
            _place(track, _rms_norm(knocking(rng)), slot[kind]); events.append(round(float(slot[kind]), 2))
        elif kind == "dog":           # a dog barks in bouts: twice now, once again later
            bark = _rms_norm(dog_bark(rng))
            for t0 in (slot[kind], slot[kind] + len(bark) / FS + 0.4, slot[kind] + 10.0):
                _place(track, bark, t0); events.append(round(float(t0), 2))
        elif kind == "generator":
            track += _rms_norm(generator_hum(rng, duration)); events.append(0.0)
        elif kind == "settling":
            for t0 in sorted(rng.uniform(1.0, duration - 2.0, 2)):
                _place(track, _rms_norm(debris(rng)), t0); events.append(round(float(t0), 2))
        else:
            continue
        p = KINDS[kind]
        gain = p["level"] * 10 ** (float(snd.get("level_db", 0.0)) / 20)
        track = through_rubble(track, p["depth"]) * gain
        room.add_source(list(snd["xy"]), signal=track)
        truth[snd["pile"]] = {"kind": kind, "xy": snd["xy"], "events_s": events, "text": snd.get("text")}

    for src in sources or []:
        xy = [float(src["xy"][0]), float(src["xy"][1])]
        room.add_source(xy, signal=source_track(src, n))
        pile = src.get("pile") or nearest_pile(piles, xy)
        starts = src.get("t0_s", 0.0)
        truth[pile] = {"kind": src.get("kind"), "xy": xy, "text": src.get("text"),
                       "events_s": [round(float(v), 2) for v in (np.atleast_1d(starts))]}

    for rid, xy in (rotors or {}).items():          # our own hovering drones (public positions)
        room.add_source([float(xy[0]), float(xy[1])], signal=rotor_noise(rng, duration) * rotor_level)

    ids = list(drones)
    mics = np.hstack([np.array(drones[d])[:, None] + offs for d in ids])
    room.add_microphone_array(pra.MicrophoneArray(mics, FS))
    sim = fast_simulate(room, n)
    if sim.shape[1] < n:
        sim = np.pad(sim, ((0, 0), (0, n - sim.shape[1])))

    # ambient: gusty low-frequency wind (independent per mic) + white sensor noise
    t = np.arange(n) / FS
    bw, aw = sps.butter(2, 120 / (FS / 2))
    out = {}
    for k, d in enumerate(ids):
        block = sim[k * N_MICS:(k + 1) * N_MICS].copy()
        gust = 1 + 0.6 * np.sin(2 * np.pi * rng.uniform(0.05, 0.2) * t + rng.uniform(0, 6.28))
        for m in range(N_MICS):
            wind = _rms_norm(sps.lfilter(bw, aw, rng.normal(0, 1, n))) * gust * 3 * noise_rms
            block[m] += wind + rng.normal(0, noise_rms, n)
        out[d] = block.astype(np.float32)
    return Scene(seed, start_pile, FS, duration, piles, drones, offs, out), truth


def single_source_scene(xy, array_xy=(0.0, 0.0), seed=0, duration=3.0) -> Scene:
    """Clean synthetic test: one broadband source, one array, grid piles. Used by tests."""
    import pyroomacoustics as pra
    rng = np.random.default_rng(seed)
    n = int(duration * FS)
    room = pra.AnechoicRoom(dim=2, fs=FS)
    b, a = sps.butter(2, [300 / (FS / 2), 4000 / (FS / 2)], "band")
    room.add_source(list(xy), signal=sps.lfilter(b, a, rng.normal(0, 1, n)))
    offs = mic_offsets()
    room.add_microphone_array(pra.MicrophoneArray(np.array(array_xy)[:, None] + offs, FS))
    room.simulate()
    sig = room.mic_array.signals[:, :n] + rng.normal(0, 1e-4, (N_MICS, n))
    return Scene(seed, "B2", FS, duration, {p: pile_xy(p) for p in PILES}, {"d1": tuple(array_xy)},
                 offs, {"d1": sig.astype(np.float32)})

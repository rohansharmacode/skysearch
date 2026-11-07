"""Stage 1 (open models, local, always on): denoise, flag outliers against the background, triage them.

Per drone/channel:
  1. Background profile. From an explicit background recording (the drone's own channel, e.g. the first
     N seconds or the whole listen; its loudest 30 % is dropped so events inside it do not count) or,
     when none is given, by minimum statistics: the quietest 30 % of the clip's frames.
  2. Denoise. noisereduce (stationary spectral gating, MIT) when installed, else our own Wiener-style
     spectral subtraction (numpy/scipy). The noise estimate is the background profile's PSD.
  3. Novelty per 16 ms frame on the denoised signal: 40-band log-mel compared with the denoised background
       band   = mean of the 5 largest robust z-scores (median / MAD per band)      -> narrow-band events
       maha   = Mahalanobis distance (shrunk covariance) / sqrt(bands)            -> broad-band change
       flux   = robust z of positive spectral flux                                -> onsets (knocks, taps)
     novelty = max(median5(0.5 * (band + maha)), 0.5 * flux): the stationary part is median-smoothed over
     5 frames (80 ms), the onset part is not (a knock is 1-3 frames). Background ~ 1-2.
  4. Segment contiguous high-novelty regions (hysteresis 4.0 on / 2.5 off, gaps < 1.6 s merged so a
     knock pattern stays one clip,
     >= 0.08 s, padded 0.25 s, split at 15 s) into candidate clips.
  5. Triage each candidate cheaply with the existing open models: Silero VAD speech probability and
     YAMNet classes (plus the knock-periodicity heuristic) ->
       likely_human  speech / shout / scream / cry / moan / cough / breathing / knock / tap
       likely_other  dog / engine / helicopter / chainsaw / siren / glass / debris ...   -> dismissed
       background    YAMNet says silence / wind / static / hum and nothing else          -> dismissed
       uncertain     anything else
     likely_human and uncertain are forwarded to stage 2 (acoustic/gemini_audio.py); dismissed candidates
     stay in the output as "dismissed: dog" etc.

Library:
    from acoustic import novelty
    res = novelty.detect(x, background=bg)        # dict: candidates (each with a denoised clip), novelty curve
"""
from __future__ import annotations

import numpy as np
from scipy import signal as sps

FS = 16000
NFFT = 512
HOP = 256                    # 16 ms frames
N_MELS = 40
FMIN, FMAX = 60.0, 7600.0
TOPK = 5
THR_ON, THR_OFF = 4.0, 2.5
MERGE_GAP_S = 1.6
MIN_DUR_S = 0.08
PAD_S = 0.25
MAX_CLIP_S = 15.0
QUIET_FRAC = 0.3
SHRINK = 0.3

# YAMNet display names (yamnet_class_map.csv) per stage-1 group
HUMAN_NAMES = {
    "Speech", "Child speech, kid speaking", "Conversation", "Narration, monologue", "Babbling", "Shout", "Yell",
    "Children shouting", "Screaming", "Whispering", "Crying, sobbing", "Baby cry, infant cry", "Whimper",
    "Wail, moan", "Sigh", "Groan", "Grunt", "Breathing", "Wheeze", "Gasp", "Pant", "Cough", "Throat clearing",
    "Sneeze", "Hubbub, speech noise, speech babble", "Chatter", "Knock", "Tap", "Wood block", "Thump, thud",
    "Humming", "Singing", "Laughter",
}
OTHER_NAMES = {
    "Animal", "Domestic animals, pets", "Dog", "Bark", "Howl", "Bow-wow", "Growling", "Whimper (dog)",
    "Canidae, dogs, wolves", "Cat", "Caterwaul", "Bird", "Bird vocalization, bird call, bird song", "Insect",
    "Livestock, farm animals, working animals", "Vehicle", "Motor vehicle (road)", "Motorcycle", "Aircraft",
    "Aircraft engine", "Helicopter", "Fixed-wing aircraft, airplane", "Engine", "Light engine (high frequency)",
    "Medium engine (mid frequency)", "Heavy engine (low frequency)", "Engine knocking", "Engine starting",
    "Idling", "Chainsaw", "Sawing", "Drill", "Dental drill, dentist's drill", "Mechanisms", "Mechanical fan",
    "Siren", "Civil defense siren", "Emergency vehicle", "Police car (siren)", "Ambulance (siren)",
    "Fire engine, fire truck (siren)", "Fire alarm", "Vehicle horn, car horn, honking", "Train",
    "Glass", "Shatter", "Breaking", "Crack", "Crackle", "Fire", "Rumble", "Clatter", "Crumpling, crinkling",
    "Creak", "Squeak", "Bang", "Gunshot, gunfire", "Fireworks", "Firecracker",
    "Water tap, faucet", "Traffic noise, roadway noise",
}
BACKGROUND_NAMES = {
    "Silence", "Wind", "Wind noise (microphone)", "Rustling leaves", "Rustle", "Rain", "Raindrop",
    "Rain on surface", "Noise", "Environmental noise", "Static", "White noise", "Pink noise", "Hum",
    "Mains hum",
}
# YAMNet name -> stage-2 style sound type (fallback path and display)
SOUND_TYPE = {
    "Shout": "call_for_help", "Yell": "call_for_help", "Children shouting": "call_for_help",
    "Screaming": "scream", "Crying, sobbing": "crying", "Baby cry, infant cry": "crying", "Whimper": "crying",
    "Wail, moan": "moaning", "Groan": "moaning", "Sigh": "moaning", "Cough": "coughing",
    "Throat clearing": "coughing", "Knock": "knocking", "Tap": "knocking", "Wood block": "knocking",
    "Thump, thud": "knocking", "Breathing": "other", "Wheeze": "other", "Gasp": "other",
}

_mel_cache = {}


# --------------------------------------------------------------------------- spectra
def _stft(x):
    _, _, Z = sps.stft(x, FS, window="hann", nperseg=NFFT, noverlap=NFFT - HOP, boundary="zeros", padded=True)
    return Z                                            # (F, T)


def _istft(Z, n):
    _, y = sps.istft(Z, FS, window="hann", nperseg=NFFT, noverlap=NFFT - HOP, boundary=True)
    y = y[:n]
    return np.pad(y, (0, max(0, n - len(y))))


def mel_fb(n_mels=N_MELS, nfft=NFFT, fs=FS, fmin=FMIN, fmax=FMAX):
    """Triangular HTK-style mel filterbank, (n_mels, nfft//2 + 1)."""
    k = (n_mels, nfft, fs, fmin, fmax)
    if k not in _mel_cache:
        hz2mel = lambda f: 2595.0 * np.log10(1.0 + f / 700.0)          # noqa: E731
        mel2hz = lambda m: 700.0 * (10 ** (m / 2595.0) - 1.0)          # noqa: E731
        pts = mel2hz(np.linspace(hz2mel(fmin), hz2mel(fmax), n_mels + 2))
        f = np.linspace(0, fs / 2, nfft // 2 + 1)
        fb = np.zeros((n_mels, len(f)))
        for i in range(n_mels):
            lo, c, hi = pts[i], pts[i + 1], pts[i + 2]
            fb[i] = np.clip(np.minimum((f - lo) / (c - lo + 1e-9), (hi - f) / (hi - c + 1e-9)), 0, None)
        _mel_cache[k] = fb
    return _mel_cache[k]


def _quiet_frames(P, frac=QUIET_FRAC):
    e = P.sum(axis=0)
    n = max(8, int(frac * P.shape[1]))
    return np.argsort(e)[:n]


# --------------------------------------------------------------------------- denoise
BG_KEEP = 0.7                # an explicit background keeps its quietest 70 % (drops events inside it)


def quiet_audio(x, frac, block_s=0.25):
    """Concatenation of the quietest ``frac`` of 0.25 s blocks of x (in time order)."""
    x = np.asarray(x, float)
    B = int(block_s * FS)
    nb = len(x) // B
    if nb < 4:
        return x
    e = (x[: nb * B].reshape(nb, B) ** 2).mean(axis=1)
    keep = np.sort(np.argsort(e)[: max(2, int(round(frac * nb)))])
    return np.concatenate([x[i * B:(i + 1) * B] for i in keep])


def noise_psd(bg=None, x=None):
    """Noise power spectrum (F,): mean over a background recording, else over x's quietest frames."""
    if bg is not None and len(bg) >= NFFT * 4:
        P = np.abs(_stft(quiet_audio(bg, BG_KEEP))) ** 2
        return P.mean(axis=1)
    P = np.abs(_stft(np.asarray(x, float))) ** 2
    return P[:, _quiet_frames(P)].mean(axis=1)


def wiener_denoise(x, N, alpha=2.0, floor=0.08):
    """Spectral subtraction with a Wiener-style gain, temporally smoothed (decision-directed lite)."""
    x = np.asarray(x, float)
    Z = _stft(x)
    P = np.abs(Z) ** 2
    snr_post = P / (alpha * N[:, None] + 1e-20)
    G = np.clip(1.0 - 1.0 / np.maximum(snr_post, 1e-9), floor, 1.0)
    G = sps.lfilter([0.6], [1, -0.4], G, axis=1)       # smooth over time: less musical noise
    return _istft(Z * G, len(x))


def denoise(x, bg=None, method="auto"):
    """Returns (denoised signal, method used). method: auto | noisereduce | wiener | none."""
    x = np.asarray(x, float)
    if method == "none":
        return x, "none"
    if method in ("auto", "noisereduce"):
        try:
            import noisereduce as nr
            if bg is not None and len(bg) >= NFFT * 4:
                noise = quiet_audio(bg, BG_KEEP)
            else:                                               # quietest blocks of x as the noise sample
                noise = quiet_audio(x, QUIET_FRAC)
            y = nr.reduce_noise(y=x, sr=FS, y_noise=np.asarray(noise, float), stationary=True,
                                prop_decrease=0.9, n_fft=NFFT, hop_length=HOP)
            return np.asarray(y, float)[: len(x)], "noisereduce"
        except ImportError:
            if method == "noisereduce":
                raise
    return wiener_denoise(x, noise_psd(bg, x)), "wiener"


# --------------------------------------------------------------------------- profile + novelty
def logmel(x, floor=None):
    P = np.abs(_stft(np.asarray(x, float))) ** 2
    M = mel_fb() @ P
    if floor is None:
        floor = 1e-10
    return np.log10(M + floor) * 10.0, P                # dB, (n_mels, T)


def _flux(L):
    d = np.diff(L, axis=1, prepend=L[:, :1])
    return np.maximum(d, 0).mean(axis=0)


class BackgroundProfile:
    """Per-band robust statistics of the (denoised) background: median, MAD-sigma, shrunk covariance,
    spectral-flux distribution. ``update`` blends in new background frames (rolling profile)."""

    def __init__(self, L, floor):
        self.floor = floor
        self._fit(L)

    def _fit(self, L):
        self.L = L
        self.mu = np.median(L, axis=1)
        mad = np.median(np.abs(L - self.mu[:, None]), axis=1)
        self.sigma = np.maximum(1.4826 * mad, 0.5)          # dB; never trust a near-zero spread
        C = np.cov(L) if L.shape[1] > 2 else np.diag(self.sigma ** 2)
        C = (1 - SHRINK) * C + SHRINK * np.diag(np.diag(C))
        C += np.eye(len(C)) * 0.25
        self.icov = np.linalg.pinv(C)
        f = _flux(L)
        self.flux_med = float(np.median(f))
        self.flux_sig = max(float(1.4826 * np.median(np.abs(f - self.flux_med))), 0.05)

    @classmethod
    def from_signal(cls, y, frames=None):
        """y: denoised background (or denoised clip, with ``frames`` = its quiet frame indices)."""
        M = mel_fb() @ (np.abs(_stft(np.asarray(y, float))) ** 2)
        floor = np.maximum(np.median(M, axis=1) * 0.05, 1e-12)[:, None]
        L = 10.0 * np.log10(M + floor)
        if frames is not None:
            L = L[:, frames]
        return cls(L, floor)

    def update(self, L_new, keep=600):
        """Rolling window: add background frames, keep the newest ``keep``."""
        self._fit(np.concatenate([self.L, L_new], axis=1)[:, -keep:])

    def score(self, y):
        """Novelty per frame of the denoised signal y: (novelty, parts dict, L)."""
        M = mel_fb() @ (np.abs(_stft(np.asarray(y, float))) ** 2)
        L = 10.0 * np.log10(M + self.floor)
        d = L - self.mu[:, None]
        z = d / self.sigma[:, None]
        band = np.sort(np.maximum(z, 0), axis=0)[-TOPK:].mean(axis=0)
        maha = np.sqrt(np.maximum(np.einsum("it,ij,jt->t", d, self.icov, d), 0) / len(self.mu))
        fz = (_flux(L) - self.flux_med) / self.flux_sig
        st = 0.5 * (band + maha)
        st = sps.medfilt(st, 5) if len(st) >= 5 else st
        nov = np.maximum(st, 0.5 * fz)
        return nov, {"band": band, "maha": maha, "flux_z": fz}, L


def segment(nov, on=THR_ON, off=THR_OFF, merge_gap=MERGE_GAP_S, min_dur=MIN_DUR_S, pad=PAD_S,
            max_len=MAX_CLIP_S, n_samples=None):
    """Hysteresis segmentation of the novelty curve -> [(s0, s1) in samples]."""
    fps = FS / HOP
    regs, inside, start = [], False, 0
    for i, v in enumerate(nov):
        if not inside and v >= on:
            inside, start = True, i
            while start > 0 and nov[start - 1] >= off:
                start -= 1
        elif inside and v < off:
            regs.append([start, i])
            inside = False
    if inside:
        regs.append([start, len(nov)])
    merged = []
    for r in regs:
        if merged and (r[0] - merged[-1][1]) / fps < merge_gap:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    n = n_samples if n_samples is not None else len(nov) * HOP + NFFT
    out = []
    for a, b in merged:
        if (b - a) / fps < min_dur:
            continue
        s0 = max(0, int((a / fps - pad) * FS))
        s1 = min(n, int((b / fps + pad) * FS) + NFFT)
        L = int(max_len * FS)
        while s1 - s0 > L:
            out.append((s0, s0 + L))
            s0 += L
        if s1 - s0 >= int(0.1 * FS):
            out.append((s0, s1))
    return out


# --------------------------------------------------------------------------- stage-1 triage
_idx = {}


def _group_idx(labels):
    if "g" not in _idx:
        pos = {n: i for i, n in enumerate(labels)}
        _idx["g"] = {g: np.array([pos[n] for n in names if n in pos])
                     for g, names in (("human", HUMAN_NAMES), ("other", OTHER_NAMES), ("bg", BACKGROUND_NAMES))}
    return _idx["g"]


def classify_candidate(clip):
    """Cheap open-model triage of one candidate clip -> dict (label, scores, reason, top classes)."""
    from . import models
    x = np.asarray(clip, np.float32)
    if len(x) < FS:                                         # YAMNet/VAD want >= ~1 s of context
        x = np.pad(x, (0, FS - len(x)))
    vp = models.vad_probs(x)
    vad_top = float(np.mean(np.sort(vp)[-max(1, len(vp) // 10):])) if len(vp) else 0.0
    vad_frac = float(np.mean(vp > 0.5)) if len(vp) else 0.0
    fr, labels = models.yamnet_scores(x)
    agg = 0.5 * fr.mean(axis=0) + 0.5 * fr.max(axis=0)
    g = _group_idx(labels)
    hum_i = g["human"][np.argmax(agg[g["human"]])]
    oth_i = g["other"][np.argmax(agg[g["other"]])]
    h_y, o_y = float(agg[hum_i]), float(agg[oth_i])
    b_y = float(agg[g["bg"]].max())
    kp = models.knock_periodicity(np.asarray(clip, float))
    human = max(h_y, 0.9 * vad_top if vad_frac > 0.05 else 0.6 * vad_top, 0.8 * kp)
    top = [[labels[i], round(float(agg[i]), 3)] for i in np.argsort(-agg)[:3]]
    return triage(human, o_y, b_y, vad_top, vad_frac, kp, labels[hum_i], labels[oth_i], top)


def triage(human, other, bg, vad_top, vad_frac, kp, human_name, other_name, top):
    """Decision rule (documented, testable without models)."""
    if human >= 0.35 and human >= 0.6 * other:
        label, why = "likely_human", f"{human_name} {human:.2f}" + (f", VAD {vad_top:.2f}" if vad_top > 0.3 else "")
    elif (other >= 0.25 and human < 0.25 and other >= 1.5 * human) or (other >= 0.15 and human < 0.12
                                                                        and other >= 2 * human):
        label, why = "likely_other", f"{other_name} {other:.2f} (human {human:.2f})"
    elif max(human, other) < 0.12 and vad_top < 0.25 and bg >= 0.2:
        label, why = "background", f"{top[0][0]} {top[0][1]:.2f}; no human or other class"
    else:
        label, why = "uncertain", f"human {human:.2f} ({human_name}), other {other:.2f} ({other_name})"
    sound = SOUND_TYPE.get(human_name, "speech" if human_name in HUMAN_NAMES and vad_top >= 0.5 else "other")
    if label == "likely_other":
        sound = "animal" if other_name in OTHER_NAMES and any(
            k in other_name for k in ("Dog", "Bark", "Animal", "Cat", "Bird", "Bow", "Howl", "Growl", "Canid",
                                      "Insect", "Livestock")) else "machinery" if any(
            k in other_name for k in ("ngine", "Vehicle", "Helicopter", "Aircraft", "Chainsaw", "Saw", "Drill",
                                      "Mechan", "Siren", "Idling", "Train", "Motor", "fan")) else "other"
    return {"label": label, "reason": why, "human_score": round(float(human), 3), "other_score": round(float(other), 3),
            "background_score": round(float(bg), 3), "vad_top": round(float(vad_top), 3),
            "vad_frac": round(float(vad_frac), 3), "knock_periodicity": round(float(kp), 3),
            "human_class": human_name, "other_class": other_name, "yamnet_top": top, "sound_type": sound}


def dismissed_tag(c):
    """'dog' from 'Bow-wow' etc., for the "dismissed: dog" display."""
    if c["label"] == "background":
        return "background"
    n = c["other_class"].lower()
    for key, tag in (("dog", "dog"), ("bark", "dog"), ("bow-wow", "dog"), ("howl", "dog"), ("growl", "dog"),
                     ("canid", "dog"), ("cat", "cat"), ("bird", "bird"), ("helicopter", "helicopter"),
                     ("aircraft", "aircraft"), ("chainsaw", "chainsaw"), ("siren", "siren"), ("engine", "engine"),
                     ("idling", "engine"), ("vehicle", "vehicle"), ("glass", "glass"), ("shatter", "glass"),
                     ("crack", "debris"), ("rumble", "debris"), ("clatter", "debris"), ("breaking", "debris"),
                     ("creak", "debris"), ("fire", "fire"), ("drill", "machinery"),
                     ("saw", "machinery"), ("mechan", "machinery"), ("fan", "machinery")):
        if key in n:
            return tag
    return n


# --------------------------------------------------------------------------- public
def detect(x, *, background=None, bg_seconds=None, drone_id=None, denoise_method="auto", classifier=None,
           max_candidates=8, thr_on=THR_ON, thr_off=THR_OFF):
    """Stage 1 on one channel. Returns a dict:

        candidates: [{cand_id, drone_id, t0, t1, novelty_peak, novelty_mean, label, reason, forwarded,
                      dismissed, human_score, other_score, vad_top, yamnet_top, sound_type, clip (denoised)}]
        novelty: per-frame curve (np.ndarray), fps, denoise method, profile source, background_level.

    background: a background recording of this drone/channel (preferred). bg_seconds: else use the first N
    seconds of x as background. Neither: minimum statistics (quietest 30 % of frames of x).
    classifier(clip) -> dict like classify_candidate (tests inject a stub)."""
    x = np.asarray(x, float)
    if bg_seconds and background is None:
        background = x[: int(bg_seconds * FS)]
    y, method = denoise(x, background, denoise_method)
    if background is not None and len(background) >= NFFT * 8:
        yb, _ = denoise(background, background, denoise_method)
        Pb = np.abs(_stft(yb)) ** 2
        prof = BackgroundProfile.from_signal(yb, frames=_quiet_frames(Pb, BG_KEEP))
        src = f"background {len(background) / FS:.1f} s (quietest {int(BG_KEEP * 100)}%)"
    else:
        Py = np.abs(_stft(y)) ** 2
        prof = BackgroundProfile.from_signal(y, frames=_quiet_frames(Py))
        src = "minimum statistics (quietest 30% of frames)"
    nov, parts, _ = prof.score(y)
    segs = segment(nov, thr_on, thr_off, n_samples=len(x))
    fps = FS / HOP
    cands = []
    for s0, s1 in segs:
        a, b = int(s0 / HOP), max(int(s0 / HOP) + 1, int(s1 / HOP))
        seg = nov[a:b] if b <= len(nov) else nov[a:]
        cands.append({"t0": round(s0 / FS, 2), "t1": round(s1 / FS, 2),
                      "novelty_peak": round(float(seg.max()) if len(seg) else 0.0, 2),
                      "novelty_mean": round(float(seg.mean()) if len(seg) else 0.0, 2),
                      "clip": y[s0:s1].astype(np.float32)})
    cands.sort(key=lambda c: -c["novelty_peak"])
    extra = cands[max_candidates:]
    cands = sorted(cands[:max_candidates], key=lambda c: c["t0"])
    clf = classifier or classify_candidate
    for i, c in enumerate(cands):
        c.update(clf(c["clip"]))
        c["cand_id"] = f"{drone_id or 'ch'}-n{i + 1}"
        c["drone_id"] = drone_id
        c["forwarded"] = c["label"] in ("likely_human", "uncertain")
        c["dismissed"] = None if c["forwarded"] else dismissed_tag(c)
    groups = forward_groups(cands, y, drone_id)
    return {"candidates": cands, "groups": groups, "novelty": nov, "fps": fps, "denoise": method, "profile": src,
            "dropped_low_novelty": len(extra),
            "novelty_background_p95": round(float(np.percentile(nov, 95)), 2) if len(nov) else 0.0}


GROUP_GAP_S = 3.0
GROUP_PAD_S = 0.5
GROUP_MIN_S = 2.0


def forward_groups(cands, y, drone_id=None, gap=GROUP_GAP_S, pad=GROUP_PAD_S, min_s=GROUP_MIN_S, max_s=MAX_CLIP_S):
    """Stage-2 units: forwarded candidates within ``gap`` s merged, padded with context, at least ``min_s``
    long (a 1 s fragment invites the audio model to guess words), at most ``max_s``. Each group carries the
    stage-1 classes of its most human-like member."""
    fw = sorted([c for c in cands if c["forwarded"]], key=lambda c: c["t0"])
    spans = []
    for c in fw:
        if spans and c["t0"] - spans[-1][1] <= gap and c["t1"] - spans[-1][0] <= max_s:
            spans[-1][1] = max(spans[-1][1], c["t1"])
            spans[-1][2].append(c)
        else:
            spans.append([c["t0"], c["t1"], [c]])
    out, dur = [], len(y) / FS
    for k, (a, b, mem) in enumerate(spans):
        a, b = max(0.0, a - pad), min(dur, b + pad)
        if b - a < min_s:
            m = (min_s - (b - a)) / 2
            a, b = max(0.0, a - m), min(dur, b + m)
        b = min(b, a + max_s)
        best = max(mem, key=lambda c: (c["human_score"], c["novelty_peak"]))
        g = {k2: best[k2] for k2 in ("label", "reason", "human_score", "other_score", "vad_top", "vad_frac",
                                      "knock_periodicity", "human_class", "other_class", "yamnet_top", "sound_type")}
        g.update(cand_id=f"{drone_id or 'ch'}-g{k + 1}", drone_id=drone_id, t0=round(a, 2), t1=round(b, 2),
                 novelty_peak=max(c["novelty_peak"] for c in mem), members=[c["cand_id"] for c in mem],
                 vad_top=max(c["vad_top"] for c in mem), forwarded=True, dismissed=None,
                 clip=np.asarray(y[int(a * FS): int(b * FS)], np.float32))
        for c in mem:
            c["group"] = g["cand_id"]
        out.append(g)
    return out


def public_candidate(c):
    """JSON-safe candidate (no audio)."""
    return {k: v for k, v in c.items() if k != "clip"}

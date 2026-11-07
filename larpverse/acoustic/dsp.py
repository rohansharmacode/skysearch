"""Array DSP: GCC-PHAT / SRP-PHAT maps per block, CLEAN sidelobe removal, multi-drone
association of bearings to piles, delay-and-sum / LCMV beamforming, SNR.
Bearings are degrees CCW from +x (east).

Localisation (see localise_piles):
  1. Per drone, per 0.5 s block: PHAT-weighted cross-spectra of every mic pair over the
     time-frequency cells that rise MASK_K x above that bin's median (transients), plus one
     unmasked whole-window set (stationary sources).
  2. SRP map in coherence units (1.0 = one fully coherent source) and CLEAN: take the peak,
     subtract the array's point-spread function for that direction (same frequency weighting),
     repeat. A strong source's sidelobes and leakage are removed instead of becoming peaks.
  3. Per block, the CLEAN components of the three drones are associated with piles greedily:
     a pile needs >= 2 drones with a component within tolerance (measurement noise + the pile's
     angular extent), or one drone with a strong component that points at no other pile. A
     component explains one pile only. Visited / excluded piles take part as explainers, so a
     loud source under the landed drones is not handed to a neighbour.
  4. Per pile: bearings averaged over its blocks, a bearing likelihood over ALL piles
     (association confidence), the share of blocks where a rival pile fit almost as well.
"""
from __future__ import annotations

import numpy as np
from scipy import signal as sps
from scipy.signal import find_peaks

from .scene import C_SOUND

ALGO_VERSION = "assoc-v11"        # bump when localisation / association changes (cache keys)
NFFT = 512
HOP = 256
BLOCK_S = 0.5
FREQ_RANGE = (300.0, 4000.0)
AZ = np.arange(360.0)            # 1 degree grid
TOL_DEG = 12.0                   # legacy peak-matching tolerance (tests / circ_peaks users)
PEAK_Z = 2.2
MASK_K = 4.0                     # TF cells kept for transient maps: power > MASK_K x per-bin median
MIN_CELLS = 30                   # a block needs this many kept TF cells to be analysed
CLEAN_GAIN = 0.8
CLEAN_MAX = 6                    # components per map
CLEAN_ABS = 0.05                 # coherence floor for a component
CLEAN_REL = 0.2                  # ... and >= this x the strongest component of the same map
SIG_MEAS = 3.5                   # deg, bearing noise of one component
PILE_R = 2.0                     # m, typical offset of a source from its pile centre
TOL_K = 2.5                      # association gate = TOL_K x combined sigma
SINGLE_AMP = 0.35                # one-drone support needs this coherence and no other pile in the gate
RIVAL = 0.5                      # rival pile with likelihood >= RIVAL x best on the same components: ambiguous block
CONF_MIN = 0.6                   # association confidence for a "confident" pile assignment
POST_MIN = 0.2                   # below this bearing posterior the detection is rejected outright


def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def bearing_to(o, p):
    return float(np.degrees(np.arctan2(p[1] - o[1], p[0] - o[0])) % 360.0)


def stft(x: np.ndarray) -> np.ndarray:
    """x (M, N) -> X (M, F, S), Hann window."""
    M, N = x.shape
    S = 1 + (N - NFFT) // HOP
    idx = np.arange(NFFT)[None, :] + HOP * np.arange(S)[:, None]
    fr = x[:, idx] * np.hanning(NFFT)[None, None, :]
    return np.fft.rfft(fr, axis=-1).transpose(0, 2, 1)


class SrpModel:
    """SRP-PHAT on a 1 degree azimuth grid for one array geometry (far field)."""

    def __init__(self, offs: np.ndarray, fs: int):
        f = np.fft.rfftfreq(NFFT, 1 / fs)
        self.fi = np.where((f >= FREQ_RANGE[0]) & (f <= FREQ_RANGE[1]))[0]
        self.f = f[self.fi]
        M = offs.shape[1]
        self.pairs = [(m, n) for m in range(M) for n in range(m + 1, M)]
        u = np.vstack([np.cos(np.radians(AZ)), np.sin(np.radians(AZ))])
        lead = offs.T @ u / C_SOUND                                   # (M, A): mic leads centre by
        self.dt = np.stack([lead[m] - lead[n] for m, n in self.pairs])  # (P, A)
        E = np.exp(-2j * np.pi * self.f[None, :, None] * self.dt[:, None, :])   # (P, F, A)
        self.E2 = E.reshape(-1, len(AZ))                              # (P*F, A)

    def cross(self, X: np.ndarray) -> np.ndarray:
        """X (M, F_all, S) -> PHAT-normalised pair cross-spectra (P, F, S), complex64."""
        Xb = X[:, self.fi, :]
        C = np.stack([Xb[m] * np.conj(Xb[n]) for m, n in self.pairs])
        return (C / (np.abs(C) + 1e-20)).astype(np.complex64)

    def map(self, R: np.ndarray, w: np.ndarray) -> np.ndarray:
        """R (P, F) summed normalised cross-spectra over w (F,) cells -> coherence map (A,)."""
        tot = len(self.pairs) * float(w.sum())
        return np.real(R.reshape(-1) @ self.E2) / max(tot, 1e-9)

    def psf(self, w: np.ndarray, a0: int) -> np.ndarray:
        """Map of one coherent far-field source at azimuth a0 with cell weights w (peak 1)."""
        v = (np.conj(self.E2[:, a0]).reshape(len(self.pairs), -1) * w[None, :]).reshape(-1)
        p = np.real(v @ self.E2)
        return p / max(p[a0], 1e-12)


def gcc_blocks(model: SrpModel, sig: np.ndarray, fs: int, mask_k=MASK_K):
    """Per-block masked cross-spectra and one unmasked whole-window set.

    Returns (Rt (B, P, F), Wt (B, F) kept-cell counts, Rs (P, F), Ws (F,))."""
    X = stft(sig.astype(np.float64))
    per = int(BLOCK_S * fs / HOP)
    B = X.shape[2] // per
    C = model.cross(X)[:, :, : B * per]                            # (P, F, B*per)
    P = np.mean(np.abs(X[:, model.fi, : B * per]) ** 2, axis=0)     # (F, S)
    keep = P > mask_k * np.median(P, axis=1, keepdims=True)
    Rt = (C * keep[None]).reshape(C.shape[0], C.shape[1], B, per).sum(axis=3).transpose(2, 0, 1)
    Wt = keep.reshape(keep.shape[0], B, per).sum(axis=2).T.astype(float)
    Rs = C.sum(axis=2)
    Ws = np.full(C.shape[1], float(C.shape[2]))
    return Rt, Wt, Rs, Ws


def _fwhm(m: np.ndarray, i: int) -> float:
    half = 0.5 * m[i]
    left = right = 0
    while left < 179 and m[(i - left - 1) % 360] > half:
        left += 1
    while right < 179 and m[(i + right + 1) % 360] > half:
        right += 1
    return float(left + right + 1)


def clean(model: SrpModel, R: np.ndarray, w: np.ndarray, max_comp=CLEAN_MAX, gain=CLEAN_GAIN):
    """CLEAN on an SRP-PHAT map: iteratively take the strongest direction and subtract the
    array's point-spread function there. Returns [(az_deg, coherence, fwhm_deg)] strongest
    first; sidelobes and leakage of strong sources do not survive as components."""
    if w.sum() < MIN_CELLS:
        return []
    res = model.map(R, w)
    floor = max(CLEAN_ABS, 4.0 / np.sqrt(2.0 * w.sum() * np.sqrt(2.0 * R.shape[0])))   # ~4 sigma of noise
    comps = {}
    for _ in range(3 * max_comp):
        i = int(np.argmax(res))
        a = float(res[i])
        if a < floor:
            break
        psf = model.psf(w, i)
        res = res - gain * a * psf
        j = next((k for k in comps if abs(wrap180(k - i)) <= 3), None)
        if j is None:
            if len(comps) >= max_comp:
                break
            comps[i] = [0.0, _fwhm(psf, i)]
            j = i
        comps[j][0] += gain * a
    out = sorted(((float(k), v[0], v[1]) for k, v in comps.items()), key=lambda c: -c[1])
    return [c for c in out if c[1] >= max(CLEAN_ABS, CLEAN_REL * out[0][1])] if out else []


def srp_blocks(sig: np.ndarray, offs: np.ndarray, fs: int, mask_k=MASK_K):
    """Coherence maps (no CLEAN): Zt (B, 360) masked per block and Zs (360,) whole window."""
    model = SrpModel(offs, fs)
    Rt, Wt, Rs, Ws = gcc_blocks(model, sig, fs, mask_k)
    Zt = np.array([model.map(Rt[b], Wt[b]) if Wt[b].sum() >= MIN_CELLS else np.zeros(len(AZ))
                   for b in range(len(Wt))])
    return Zt, model.map(Rs, Ws)


def circ_peaks(z: np.ndarray, height=PEAK_Z, k=4):
    """Peaks of a circular spectrum: list of (az_deg, height), strongest first."""
    pad = 20
    zz = np.concatenate([z[-pad:], z, z[:pad]])
    pk, pr = find_peaks(zz, height=height, distance=8)
    pk = pk - pad
    keep = (pk >= 0) & (pk < len(z))
    pk, h = pk[keep], pr["peak_heights"][keep]
    order = np.argsort(-h)[:k]
    return [(float(AZ[pk[i]]), float(h[i])) for i in order]


def triangulate(origins, bearings_deg):
    """Least-squares intersection of bearing rays. Returns (xy, rms perpendicular residual m)."""
    A = np.zeros((2, 2))
    b = np.zeros(2)
    rows = []
    for o, th in zip(origins, bearings_deg):
        u = np.array([np.cos(np.radians(th)), np.sin(np.radians(th))])
        nrm = np.array([-u[1], u[0]])
        A += np.outer(nrm, nrm)
        b += nrm * (nrm @ np.asarray(o, float))
        rows.append((np.asarray(o, float), nrm, u))
    if np.linalg.cond(A) > 1e6:
        return None, float("inf")
    p = np.linalg.solve(A, b)
    res = [float(nrm @ (p - o)) for o, nrm, _ in rows]
    behind = any(float(u @ (p - o)) < 0 for o, _, u in rows)
    return p, (float(np.sqrt(np.mean(np.square(res)))) if not behind else float("inf"))


_FFT_CACHE: dict = {}
_SOS_BAND = sps.butter(4, [300, 4000], "band", fs=16000, output="sos")


def beamform(sig: np.ndarray, offs: np.ndarray, fs: int, bearing_deg: float, nulls=(), load=0.1) -> np.ndarray:
    """Far-field frequency-domain beamformer toward bearing_deg. Returns mono float32.

    No nulls: delay-and-sum. With nulls: min-norm LCMV (unit gain on target, zero gain toward
    each null bearing, diagonal loading `load` to bound white-noise gain at low frequency)."""
    M, N = sig.shape
    key = (id(sig), N, float(sig[:, :256].sum()), float(sig[:, -256:].sum()))   # id + fingerprint
    if _FFT_CACHE.get("key") != key:                             # one FFT per drone signal
        _FFT_CACHE.update(key=key, X=np.fft.rfft(sig.astype(np.float64), axis=1))
    Xf = _FFT_CACHE["X"]                                         # (M, F)
    f = np.fft.rfftfreq(N, 1 / fs)

    def steer(th):
        u = np.array([np.cos(np.radians(th)), np.sin(np.radians(th))])
        tau = (offs.T @ u) / C_SOUND                             # mic leads the centre by tau
        return np.exp(2j * np.pi * f[:, None] * tau[None, :])    # (F, M)

    if not len(nulls):
        Y = np.mean(Xf * np.conj(steer(bearing_deg)).T, axis=0)
    else:
        C = np.stack([steer(bearing_deg)] + [steer(n) for n in nulls], axis=2)   # (F, M, K)
        K = C.shape[2]
        G = np.conj(C).transpose(0, 2, 1) @ C + load * M * np.eye(K)[None]
        g = np.zeros((K, 1)); g[0] = 1.0
        w = C @ np.linalg.solve(G, np.broadcast_to(g, (len(f), K, 1)))          # (F, M, 1)
        Y = np.sum(np.conj(w[:, :, 0]).T * Xf, axis=0)
    return np.fft.irfft(Y, n=N).astype(np.float32)


def _steady(scene, ids, exp, B, ratio=0.35):
    """Stationarity test for a candidate direction: band-limited delay-and-sum power per block
    on the nearest drone; a steady source (generator) keeps p20/p80 high, transients do not."""
    d = min(ids, key=lambda k: abs(exp[k]))  # any drone works; keep it deterministic
    y = sps.sosfilt(_SOS_BAND, beamform(scene.signals[d], scene.mic_offsets, scene.fs, exp[d]).astype(float))
    blk = int(BLOCK_S * scene.fs)
    p = np.array([np.mean(y[b * blk:(b + 1) * blk] ** 2) for b in range(B)])
    return bool(np.percentile(p, 20) >= ratio * np.percentile(p, 80))


def _sigma(rng_m: float) -> float:
    """Bearing sigma (deg) for a pile at range rng_m: measurement noise + pile angular extent."""
    ext = np.degrees(np.arctan2(PILE_R, max(rng_m, 1.0)))
    return float(np.hypot(SIG_MEAS, ext))


class _Geom:
    """Expected bearings, sigmas and gates from every drone to every pile (public geometry)."""

    def __init__(self, scene):
        self.ids = list(scene.drones)
        self.piles = list(scene.piles)
        self.exp = {d: {p: bearing_to(scene.drones[d], scene.piles[p]) for p in self.piles} for d in self.ids}
        rng = {d: {p: float(np.hypot(*(np.subtract(scene.piles[p], scene.drones[d])))) for p in self.piles}
               for d in self.ids}
        self.sig = {d: {p: _sigma(rng[d][p]) for p in self.piles} for d in self.ids}

    def gate(self, d, p):
        return TOL_K * self.sig[d][p]


def _match(g: _Geom, comps: dict, used: dict, p: str, only=None):
    """Best unused component per drone inside pile p's gate. Returns {d: (idx, err, amp, k)}."""
    out = {}
    for d in g.ids:
        best = None
        for i, (az, amp, _w) in enumerate(comps[d]):
            if used[d][i] or (only is not None and only.get(d) != i):
                continue
            err = float(wrap180(az - g.exp[d][p]))
            if abs(err) <= g.gate(d, p):
                k = amp * np.exp(-0.5 * (err / g.sig[d][p]) ** 2)
                if best is None or k > best[3]:
                    best = (i, err, amp, k)
        if best is not None:
            out[d] = best
    return out


def associate(g: _Geom, comps: dict):
    """Greedy exclusive association of one block's CLEAN components {d: [(az, amp, w)]} to piles.

    Returns [{pile, match {d: (idx, err, amp, k)}, score, rival, rival_ratio}], best first."""
    used = {d: [False] * len(comps[d]) for d in g.ids}
    events = []
    while True:
        cands = []
        for p in g.piles:
            m = _match(g, comps, used, p)
            if len(m) >= 2:
                cands.append((sum(v[3] for v in m.values()), p, m))
            elif len(m) == 1:                            # one drone: strong and unambiguous only
                (d, v), = m.items()
                az = comps[d][v[0]][0]
                others = [q for q in g.piles if q != p and abs(wrap180(az - g.exp[d][q])) <= g.gate(d, q)]
                if v[2] >= SINGLE_AMP and not others:
                    cands.append((0.5 * v[3], p, m))     # half weight: no range information
        if not cands:
            return events
        score, p, m = max(cands, key=lambda c: c[0])
        rival, ratio = None, 0.0
        for q in g.piles:                                # another pile fitting the SAME components
            if q == p:
                continue
            mq = _match(g, comps, used, q, only={d: v[0] for d, v in m.items()})
            if len(mq) == len(m):                         # must explain every drone p explains
                # joint bearing likelihood ratio q : p over the same components (amplitudes cancel):
                # one drone that clearly separates the two piles is enough to resolve them
                ll = sum(-0.5 * (mq[d][1] / g.sig[d][q]) ** 2 - np.log(g.sig[d][q])
                         + 0.5 * (m[d][1] / g.sig[d][p]) ** 2 + np.log(g.sig[d][p]) for d in m)
                r = float(np.exp(min(ll, 5.0)))
                if r > ratio:
                    rival, ratio = q, float(r)
        events.append({"pile": p, "match": m, "score": float(score), "rival": rival, "rival_ratio": ratio})
        for d, v in m.items():
            used[d][v[0]] = True


def bearing_posterior(g: _Geom, bearings: dict) -> dict:
    """P(pile | measured bearings {d: az}) over ALL piles, flat prior, Gaussian bearing errors."""
    ll = {}
    for p in g.piles:
        ll[p] = sum(-0.5 * (float(wrap180(az - g.exp[d][p])) / g.sig[d][p]) ** 2 - np.log(g.sig[d][p])
                    for d, az in bearings.items())
    mx = max(ll.values())
    z = sum(np.exp(v - mx) for v in ll.values())
    return {p: float(np.exp(v - mx) / z) for p, v in ll.items()}


def _circ_mean(az, w):
    a = np.radians(np.asarray(az, float))
    return float(np.degrees(np.arctan2(np.sum(w * np.sin(a)), np.sum(w * np.cos(a)))) % 360.0)


def localise_piles(scene, exclude=()):
    """Find sources and assign them to piles with multi-drone association (module docstring).

    Every pile, excluded ones included, competes for the CLEAN components, so the sources of
    visited piles explain their own energy; detections on excluded piles come back flagged
    `excluded` (callers use them as interferers to null, not as clips).

    Transient sources: per-block association (>= 2 blocks = 1 s). Stationary sources: the
    unmasked whole-window maps, kept as `continuous` when the beam power is steady or the
    source has no transient blocks (a generator). Returns (detections, n_blocks)."""
    g = _Geom(scene)
    ids = g.ids
    model = SrpModel(scene.mic_offsets, scene.fs)
    gcc = {d: gcc_blocks(model, scene.signals[d], scene.fs) for d in ids}
    B = min(v[1].shape[0] for v in gcc.values())
    blocks = []                                          # per block: events
    for b in range(B):
        comps = {d: clean(model, gcc[d][0][b], gcc[d][1][b]) for d in ids}
        blocks.append((comps, associate(g, comps)))
    scomps = {d: clean(model, gcc[d][2], gcc[d][3]) for d in ids}
    sevents = {e["pile"]: e for e in associate(g, scomps)}

    dets = []
    for pid in g.piles:
        evs = [(b, e) for b, (_c, es) in enumerate(blocks) for e in es if e["pile"] == pid]
        active = np.zeros(B, bool)
        score = np.zeros(B)
        for b, e in evs:
            active[b] = True
            score[b] = e["score"]
        stat = sevents.get(pid)
        continuous = False
        if stat is not None and len(stat["match"]) >= 2:
            exp_s = {d: scomps[d][v[0]][0] for d, v in stat["match"].items()}
            continuous = active.sum() < 2 or active.mean() >= 0.5 or _steady(scene, list(exp_s), exp_s, B)
        if not continuous and active.sum() < 2:
            continue
        # per-drone measured bearing: amplitude-weighted over the pile's blocks
        hits = {d: [] for d in ids}
        if continuous:
            for d, v in stat["match"].items():
                az, amp, w = scomps[d][v[0]]
                hits[d].append((az, amp, w))
            n_act = 1
            active = np.ones(B, bool)
            score = np.full(B, stat["score"])
            amb = [(stat["rival"], stat["rival_ratio"])]
        else:
            for b, e in evs:
                for d, v in e["match"].items():
                    hits[d].append(blocks[b][0][d][v[0]])
            n_act = len(evs)
            amb = [(e["rival"], e["rival_ratio"]) for _b, e in evs]
        per_drone = {}
        for d in ids:
            h = hits[d]
            if h:
                brg = _circ_mean([x[0] for x in h], np.array([x[1] for x in h]))
                width = float(np.median([x[2] for x in h]))
                amp = float(np.mean([x[1] for x in h]))
            else:
                brg, width, amp = g.exp[d][pid], float("nan"), 0.0
            per_drone[d] = {"bearing_deg": round(brg, 1), "width_deg": round(width, 1) if h else None,
                            "sigma_deg": round(max(1.0, SIG_MEAS / np.sqrt(len(h))) if h else g.gate(d, pid), 1),
                            "coherence": round(amp, 3), "hit_frac": round(len(h) / n_act, 2),
                            "expected_deg": round(g.exp[d][pid], 1), "measured": bool(h)}
        agreeing = [d for d in ids if per_drone[d]["hit_frac"] >= 0.5]
        consistent = [d for d in agreeing                # bearing residual to this pile's centre
                      if abs(wrap180(per_drone[d]["bearing_deg"] - g.exp[d][pid])) <= min(15.0, g.gate(d, pid))]
        meas = {d: per_drone[d]["bearing_deg"] for d in agreeing}
        post = bearing_posterior(g, meas) if meas else {p: 1.0 / len(g.piles) for p in g.piles}
        best = max(post, key=post.get)
        amb_frac = float(np.mean([r >= RIVAL for _q, r in amb]))
        rivals = sorted({q for q, r in amb if q and r >= RIVAL})
        conf = post[pid] * (1.0 - 0.5 * amb_frac) * (1.0 if len(agreeing) >= 2 else 0.6)
        if best != pid:
            rivals = sorted(set(rivals) | {best})
        tri_xy, resid = (None, float("inf"))
        if len(agreeing) >= 2:
            tri_xy, resid = triangulate([scene.drones[d] for d in agreeing], [meas[d] for d in agreeing])
        tri_err = float(np.hypot(*(tri_xy - np.array(scene.piles[pid])))) if tri_xy is not None else float("inf")
        confident = conf >= CONF_MIN and best == pid and len(agreeing) >= 2
        for d in ids:                                    # cone width for drones that never saw it
            if per_drone[d]["width_deg"] is None:
                per_drone[d]["width_deg"] = round(float(np.nanmedian(
                    [v["width_deg"] for v in per_drone.values() if v["width_deg"] is not None] or [60.0])), 1)
        dets.append({
            "pile_id": pid, "excluded": pid in set(exclude), "active_blocks": active, "block_score": score,
            "continuous": bool(continuous), "n_agree_mean": float(np.mean([len(e["match"]) for _b, e in evs]))
            if evs and not continuous else float(len(agreeing)),
            "drones_agreeing": agreeing, "drones_consistent": consistent, "per_drone": per_drone,
            "tri_xy": None if tri_xy is None else [round(float(v), 2) for v in tri_xy],
            "tri_err_m": round(tri_err, 2) if np.isfinite(tri_err) else None,
            "tri_resid_m": round(resid, 2) if np.isfinite(resid) else None,
            "posterior": round(post[pid], 3), "best_pile": best, "ambiguous_frac": round(amb_frac, 2),
            "ambiguous_with": rivals, "assoc_conf": round(float(conf), 3),
            "association": "confident" if confident else "ambiguous",
            "assigned": bool(confident),
        })
    return dets, B


def extract_clip(scene, det, B, others=(), pad_blocks=1, max_s=14.0, min_blocks=4):
    """Separate one pile's source: beamform each drone toward its measured bearing with nulls
    toward the other detected sources (excluded piles' too) that overlap in time, keep the
    best-SNR drone.

    The clip concatenates this pile's active blocks (dilated), minus blocks where another
    transient source is active when enough clean blocks remain. Noise floor is the
    20th-percentile beam power over this pile's inactive blocks (back beam if continuous).

    Returns (drone_id, snr_db, clip, dominated_frac, blocks): dominated_frac is the share of the
    clip's blocks where another transient source was associated more strongly than this pile,
    i.e. how much of the clip's content (speech, knocks) may be leakage from elsewhere; blocks
    are the absolute block indices the clip is made of, in order."""
    fs = scene.fs
    blk = int(BLOCK_S * fs)
    act = det["active_blocks"].copy()
    for _ in range(pad_blocks):
        act = act | np.r_[act[1:], False] | np.r_[False, act[:-1]]
    own = det["active_blocks"][:B]
    own_s = det.get("block_score", own.astype(float))[:B]
    other_s = np.zeros(B)                                # strongest other transient source per block
    for o in others:
        if not o.get("continuous"):
            other_s = np.maximum(other_s, o.get("block_score", o["active_blocks"].astype(float))[:B])
    if det.get("continuous"):                            # stationary: only blocks nobody else uses
        keep = act & (other_s == 0)
        sel = keep if keep.sum() >= min_blocks else act
    else:                                                # transient: own blocks it dominates + free padding
        keep = (own & (own_s >= other_s)) | (act & ~own & (other_s == 0))
        sel = keep if (keep & own).sum() >= 2 else act
    best = None
    snrs = {}
    for d in det["per_drone"]:
        brg = det["per_drone"][d]["bearing_deg"]
        nulls = [o["per_drone"][d]["bearing_deg"] for o in others
                 if (o.get("continuous") or (o["active_blocks"][:B] & act).any())
                 and abs(wrap180(o["per_drone"][d]["bearing_deg"] - brg)) > 25][:4]
        y = beamform(scene.signals[d], scene.mic_offsets, fs, brg, nulls=nulls)
        ym = beamform(scene.signals[d], scene.mic_offsets, fs, brg) if nulls else y   # SNR on plain DS

        def bpow(sig_):   # per-block power in the band where the 0.3 m array is directive
            z = sps.sosfilt(_SOS_BAND, sig_.astype(float))
            return np.array([np.mean(z[b * blk:(b + 1) * blk] ** 2) for b in range(B)])

        pw = bpow(ym)
        off = ~act
        if off.sum() >= 3:
            noise = float(np.percentile(pw[off], 20))
        else:   # continuous source: quietest blocks of beams steered >= 60 deg away
            ref = [bpow(beamform(scene.signals[d], scene.mic_offsets, fs, brg + k)) for k in (90, 180, 270)]
            noise = float(np.percentile(np.concatenate(ref), 20))
        noise += 1e-15
        sigp = float(np.mean(pw[det["active_blocks"][:B]]))
        snr = 10 * np.log10(max(sigp - noise, 1e-15) / noise)
        snrs[d] = round(float(snr), 1)
        if best is None or snr > best[1]:
            best = (d, snr, y)
    det["snr_by_drone"] = snrs
    d, snr, y = best
    idx = np.where(sel)[0]
    segs = [y[b * blk:(b + 1) * blk] for b in idx]
    clip = np.concatenate(segs)[: int(max_s * fs)] if segs else np.zeros(blk, np.float32)
    idx = idx[: int(np.ceil(max_s / BLOCK_S))]
    if det.get("continuous"):
        dom = other_s[idx] > 0
    else:
        dom = np.where(own[idx], other_s[idx] > own_s[idx], other_s[idx] > 0)
    dominated = float(dom.mean()) if len(idx) else 0.0
    return d, round(float(snr), 1), clip, round(dominated, 2), idx

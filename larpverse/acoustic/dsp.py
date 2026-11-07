"""Array DSP: SRP-PHAT (pyroomacoustics) per block, pile hypothesis tests, triangulation,
delay-and-sum beamforming, SNR. Bearings are degrees CCW from +x (east)."""
from __future__ import annotations

import numpy as np
from scipy import signal as sps
from scipy.signal import find_peaks

from .scene import C_SOUND

NFFT = 512
HOP = 256
BLOCK_S = 0.5
FREQ_RANGE = (300.0, 4000.0)
AZ = np.arange(360.0)            # 1 degree grid
PEAK_Z = 2.2                     # peak height (robust z-score over azimuth) to count as a source
TOL_DEG = 12.0                   # pile hypothesis matches a peak within this angle
MASK_K = 4.0                     # TF bins kept for transient SRP: power > MASK_K x per-bin median


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


def _srp(doa, X):
    doa.locate_sources(X, freq_range=list(FREQ_RANGE))
    v = np.asarray(doa.grid.values, float)
    med = np.median(v)
    mad = np.median(np.abs(v - med)) * 1.4826 + 1e-9
    return (v - med) / mad


def srp_blocks(sig: np.ndarray, offs: np.ndarray, fs: int, mask_k=MASK_K):
    """SRP-PHAT spatial spectra, robust z-scored over azimuth.

    Returns (Zt, Zs): Zt (B, 360) per-block spectra computed only on time-frequency bins
    that rise mask_k x above that bin's median power (removes stationary interferers such
    as a generator so transient sources show up), and Zs (360,) over the whole window
    without masking (finds the stationary sources themselves)."""
    import pyroomacoustics as pra
    X = stft(sig.astype(np.float64))
    per = int(BLOCK_S * fs / HOP)
    B = X.shape[2] // per
    doa = pra.doa.algorithms["SRP"](offs, fs, NFFT, c=C_SOUND, num_src=1, azimuth=np.radians(AZ))
    P = np.mean(np.abs(X) ** 2, axis=0)                       # (F, S)
    keep = P > mask_k * np.median(P, axis=1, keepdims=True)
    Xm = X * keep[None]
    Zt = np.zeros((B, len(AZ)))
    for b in range(B):
        sl = slice(b * per, (b + 1) * per)
        Zt[b] = _srp(doa, Xm[:, :, sl]) if keep[:, sl].sum() >= 20 else 0.0
    Zs = _srp(doa, X)
    return Zt, Zs


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


def peak_width(zavg: np.ndarray, center_deg: float, search=TOL_DEG):
    """Refine a bearing near center_deg on an averaged spectrum; full width at half height."""
    d = np.abs(wrap180(AZ - center_deg))
    win = d <= search
    i = int(np.argmax(np.where(win, zavg, -np.inf)))
    peak = zavg[i]
    base = np.median(zavg)
    half = base + 0.5 * (peak - base)
    left = right = 0
    while left < 179 and zavg[(i - left - 1) % 360] > half:
        left += 1
    while right < 179 and zavg[(i + right + 1) % 360] > half:
        right += 1
    # parabolic refinement
    y0, y1, y2 = zavg[(i - 1) % 360], zavg[i], zavg[(i + 1) % 360]
    den = y0 - 2 * y1 + y2
    frac = 0.5 * (y0 - y2) / den if abs(den) > 1e-9 else 0.0
    return float((AZ[i] + np.clip(frac, -0.5, 0.5)) % 360.0), float(left + right + 1), float(peak)


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


def _assigned(scene, pid, agreeing, per_drone, tri_xy, resid):
    """Assign a detection to pid only if pid explains it better than every other pile.

    Well-conditioned triangulation: the point must be nearest to pid (rejects a pile that is
    merely collinear with the real source as seen from the landed drones). Far, near-parallel
    rays: fall back to the pile whose expected bearings best match the measured ones."""
    if len(agreeing) < 2 or tri_xy is None or not np.isfinite(resid):
        return False
    cen = np.mean([scene.drones[d] for d in agreeing], axis=0)
    rng_p = float(np.hypot(*(np.array(scene.piles[pid]) - cen)))
    if float(np.hypot(*(tri_xy - cen))) <= 1.5 * rng_p + 10.0:
        return min(scene.piles, key=lambda n: np.hypot(*(tri_xy - np.array(scene.piles[n])))) == pid

    def ang_rms(n):
        return float(np.sqrt(np.mean([wrap180(per_drone[d]["bearing_deg"] - bearing_to(scene.drones[d], scene.piles[n])) ** 2
                                       for d in agreeing])))
    return min(scene.piles, key=ang_rms) == pid


def localise_piles(scene, exclude=()):
    """Test each pile hypothesis against every drone's SRP-PHAT peaks.

    Transient sources: per-block peaks of the masked spectra; a block is active for a pile
    when >= 2 drones have a peak within TOL_DEG of the bearing to that pile.
    Stationary sources: peaks of the whole-window spectrum matched the same way.
    Then a free least-squares triangulation of the measured bearings must land closer to this
    pile than to any other for the detection to count as 'assigned' (see _assigned).
    """
    ids = list(scene.drones)
    spectra = {d: srp_blocks(scene.signals[d], scene.mic_offsets, scene.fs) for d in ids}
    B = min(z[0].shape[0] for z in spectra.values())
    peaks = {d: [circ_peaks(spectra[d][0][b]) for b in range(B)] for d in ids}
    speaks = {d: circ_peaks(spectra[d][1]) for d in ids}

    def near(pk, e):
        return any(abs(wrap180(a - e)) <= TOL_DEG for a, _ in pk)

    dets = []
    for pid, pxy in scene.piles.items():
        if pid in exclude:
            continue
        exp = {d: bearing_to(scene.drones[d], pxy) for d in ids}
        n_stat = sum(near(speaks[d], exp[d]) for d in ids)
        n_agree = np.array([sum(near(peaks[d][b], exp[d]) for d in ids) for b in range(B)])
        active = n_agree >= 2
        continuous = n_stat >= 2 and (active.mean() >= 0.5 or active.sum() < 2
                                      or _steady(scene, ids, exp, B))
        if continuous:
            active = np.ones(B, bool)
        elif active.sum() < 2:                  # need >= 1 s of agreement
            continue
        per_drone = {}
        for d in ids:
            zavg = spectra[d][1] if continuous else spectra[d][0][active].mean(axis=0)
            brg, width, h = peak_width(zavg, exp[d])
            hit = (1.0 if near(speaks[d], exp[d]) else 0.0) if continuous else \
                float(np.mean([near(peaks[d][b], exp[d]) for b in np.where(active)[0]]))
            per_drone[d] = {"bearing_deg": round(brg, 1), "width_deg": round(width, 1),
                            "peak_z": round(h, 2), "hit_frac": round(hit, 2),
                            "expected_deg": round(exp[d], 1)}
        agreeing = [d for d in ids if per_drone[d]["hit_frac"] >= 0.5]
        tri_xy, resid = (None, float("inf"))
        if len(agreeing) >= 2:
            tri_xy, resid = triangulate([scene.drones[d] for d in agreeing],
                                        [per_drone[d]["bearing_deg"] for d in agreeing])
        tri_err = float(np.hypot(*(tri_xy - np.array(pxy)))) if tri_xy is not None else float("inf")
        dets.append({
            "pile_id": pid, "active_blocks": active, "continuous": bool(continuous),
            "n_agree_mean": float(n_stat if continuous else n_agree[active].mean()),
            "drones_agreeing": agreeing, "per_drone": per_drone,
            "tri_xy": None if tri_xy is None else [round(float(v), 2) for v in tri_xy],
            "tri_err_m": round(tri_err, 2) if np.isfinite(tri_err) else None,
            "tri_resid_m": round(resid, 2) if np.isfinite(resid) else None,
            "assigned": _assigned(scene, pid, agreeing, per_drone, tri_xy, resid),
        })
    return dets, B


def extract_clip(scene, det, B, others=(), pad_blocks=1, max_s=14.0, min_blocks=4):
    """Separate one pile's source: beamform each drone toward its measured bearing with nulls
    toward the other detected sources that overlap in time, keep the best-SNR drone.

    The clip concatenates this pile's active blocks (dilated), minus blocks where another
    transient source is active when enough clean blocks remain. Noise floor is the
    20th-percentile beam power over this pile's inactive blocks (back beam if continuous)."""
    fs = scene.fs
    blk = int(BLOCK_S * fs)
    act = det["active_blocks"].copy()
    for _ in range(pad_blocks):
        act = act | np.r_[act[1:], False] | np.r_[False, act[:-1]]
    busy = np.zeros(B, bool)
    for o in others:
        if not o.get("continuous"):
            busy |= o["active_blocks"][:B]
    clean = act & ~busy
    # continuous sources take only clean blocks; transient ones only drop a few overlapped blocks
    sel = clean if (clean.sum() >= min_blocks and (det.get("continuous") or clean.sum() >= 0.7 * act.sum())) else act
    best = None
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
        if best is None or snr > best[1]:
            best = (d, snr, y)
    d, snr, y = best
    segs = [y[b * blk:(b + 1) * blk] for b in np.where(sel)[0]]
    clip = np.concatenate(segs)[: int(max_s * fs)] if segs else np.zeros(blk, np.float32)
    return d, round(float(snr), 1), clip

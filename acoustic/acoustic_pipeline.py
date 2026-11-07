#!/usr/bin/env python3
"""Listen for human voices from a landed drone and flag areas to search.

The drone lands at a site, records with its mic array, and reports whether it
heard a voice and from which direction. The stages are deliberately separate:

    simulate_site()  scene -> per-mic audio   (replaced by real mics on hardware)
    detect_voice()   audio + mic positions -> detections   (sees no ground truth)
    build_report()   detections from every site -> flags.json

Isaac Sim does not need to simulate sound. It only supplies the positions in a
scene file. Rotors are off during a scan, so they are not modelled.

Detection, per 2 s block:
    1. Discount each frequency's steady noise floor, so hums cannot steer the array.
    2. Find the strongest remaining direction (SRP-PHAT) and beamform towards it.
    3. Classify the beamformed audio with YAMNet, a pretrained sound classifier
       that knows speech, shouting and crying apart from dogs, sirens and machines.
A site is flagged when several voice blocks agree on a direction.

Needs `pip install -r requirements.txt`. The model is in models/ (Apache-2.0).

"no_voice_heard" does not mean nobody is there. An unconscious person makes no
sound, so a quiet site must not be treated as cleared.

Bearings are degrees counter-clockwise from world +X, not compass bearings. A
flat array cannot measure elevation, and a single site cannot measure range.

Not modelled: ground reflections, obstacles and rubble, air absorption, and echoes.
"""
from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass, field, replace
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy import fft, signal
from scipy.io import wavfile
from scipy.spatial.transform import Rotation

SCHEMA_VERSION = "acoustic-voice-1"
SPEED_OF_SOUND = 343.0  # m/s
P_REF = 20e-6  # Pa, 0 dB SPL
MIN_DISTANCE = 0.1  # m, stops 1/r blowing up for very close sources
SOURCE_KINDS = ("voice", "generator", "engine", "birds", "recording")

MODEL_DIR = Path(__file__).resolve().parent / "models"
CLASSIFIER_FS = 16000
CLASSIFIER_WINDOW = 15600  # samples, YAMNet's fixed 0.975 s input
HUMAN_CLASSES = frozenset(
    {
        "Speech",
        "Child speech, kid speaking",
        "Conversation",
        "Narration, monologue",
        "Babbling",
        "Shout",
        "Bellow",
        "Whoop",
        "Yell",
        "Children shouting",
        "Screaming",
        "Crying, sobbing",
        "Baby cry, infant cry",
        "Whimper",
        "Wail, moan",
        "Groan",
        "Hubbub, speech noise, speech babble",
    }
)
ANIMAL_CLASSES = frozenset(
    {
        "Animal",
        "Domestic animals, pets",
        "Dog",
        "Bark",
        "Yip",
        "Howl",
        "Bow-wow",
        "Growling",
        "Whimper (dog)",
        "Livestock, farm animals, working animals",
        "Wild animals",
    }
)

# Typical vowel formants (F1, F2, F3) in Hz.
VOWELS = (
    (730, 1090, 2440),
    (270, 2290, 3010),
    (300, 870, 2240),
    (530, 1840, 2480),
    (570, 840, 2410),
)


@dataclass(frozen=True)
class ArrayConfig:
    fs: int = 16000
    n_mics: int = 8
    radius: float = 0.10  # m, circular array in the body XY plane
    height: float = 0.10  # m, above the body origin


@dataclass(frozen=True)
class DetectorConfig:
    doa_band_hz: tuple[float, float] = (300.0, 2000.0)  # array aliases above 2.2 kHz
    nperseg: int = 1024
    hop: int = 256
    block_s: float = 2.0
    block_hop_s: float = 1.0
    azimuth_step_deg: float = 5.0
    noise_percentile: float = 10.0  # low, so someone calling constantly is not "noise"
    noise_margin: float = 4.0  # that percentile sits well below the average noise level
    voice_score_min: float = 0.2  # human minus animal classifier score, per block
    cluster_deg: float = 20.0  # voice blocks this close count as one direction
    min_blocks: int = 2  # voice blocks needed to raise a flag
    high_priority_blocks: int = 4


@dataclass(frozen=True)
class Site:
    id: str
    position: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    t0: float = 0.0  # scan start on the shared sim clock


@dataclass(frozen=True)
class Source:
    kind: str
    position: tuple[float, float, float]
    spl_1m: float  # voice: level while speaking; others: RMS level
    wav: str | None = None  # play this file instead of a synthetic sound

    def __post_init__(self):
        if self.kind not in SOURCE_KINDS:
            raise ValueError(f"unknown source kind {self.kind!r}, use {SOURCE_KINDS}")
        if self.kind == "recording" and not self.wav:
            raise ValueError('kind "recording" (non-voice audio) needs a wav file')


@dataclass(frozen=True)
class Scene:
    sites: tuple[Site, ...]
    sources: tuple[Source, ...] = ()
    scan_seconds: float = 30.0
    ambient_spl: float = 45.0  # diffuse background at the array
    wind_spl: float = 40.0  # turbulence at each mic, uncorrelated between mics
    seed: int = 0


@dataclass(frozen=True)
class Detection:
    bearing_deg: float
    bearing_uncertainty_deg: float
    voice_blocks: int
    first_heard_s: float
    last_heard_s: float


@dataclass
class BlockResult:
    start_s: float
    azimuth_deg: float
    voice_score: float
    is_voice: bool = field(default=False)


def load_scene(path: str | Path) -> Scene:
    raw = json.loads(Path(path).read_text())
    sites = tuple(
        Site(s["id"], tuple(s["position"]), tuple(s.get("quat_wxyz", (1, 0, 0, 0))), s.get("t0", 0.0))
        for s in raw["sites"]
    )
    sources = tuple(
        Source(s["kind"], tuple(s["position"]), s["spl_1m"], s.get("wav"))
        for s in raw.get("sources", ())
    )
    settings = {k: raw[k] for k in ("scan_seconds", "ambient_spl", "wind_spl", "seed") if k in raw}
    return Scene(sites, sources, **settings)


def demo_scene(seed: int = 0) -> Scene:
    """Three sites: a caller nearby, a generator with no caller, and a quiet site."""
    return Scene(
        sites=(
            Site("A", (0.0, 0.0, 0.0)),
            Site("B", (60.0, 0.0, 0.0), (0.92388, 0.0, 0.0, 0.38268)),  # yawed 45 deg
            Site("C", (0.0, 80.0, 0.0)),
        ),
        sources=(
            Source("voice", (15.0, 12.0, 0.3), 85.0),
            Source("generator", (70.0, -10.0, 0.5), 80.0),
        ),
        seed=seed,
    )


def mic_positions_world(site: Site, cfg: ArrayConfig) -> np.ndarray:
    angle = 2 * np.pi * np.arange(cfg.n_mics) / cfg.n_mics
    body = np.column_stack(
        [
            cfg.radius * np.cos(angle),
            cfg.radius * np.sin(angle),
            np.full(cfg.n_mics, cfg.height),
        ]
    )
    w, x, y, z = site.quat_wxyz
    return Rotation.from_quat([x, y, z, w]).apply(body) + np.asarray(site.position, float)


def spl_to_rms(spl_db: float) -> float:
    return P_REF * 10 ** (spl_db / 20)


def _active_rms(x: np.ndarray, fs: int) -> float:
    """RMS while the source is sounding, ignoring pauses."""
    frame = int(0.02 * fs)
    n = x.size // frame * frame
    power = np.mean(x[:n].reshape(-1, frame) ** 2, axis=1)
    if power.max() == 0:
        return 0.0
    return float(np.sqrt(power[power > power.max() * 1e-3].mean()))


def _resonator(x: np.ndarray, freq: float, bandwidth: float, fs: int) -> np.ndarray:
    r = np.exp(-np.pi * bandwidth / fs)
    return signal.lfilter([1 - r], [1, -2 * r * np.cos(2 * np.pi * freq / fs), r * r], x)


def synth_voice(rng, n: int, fs: int) -> np.ndarray:
    """Calls for help: phrases of vowel syllables with varying pitch, then pauses."""
    f0_base = rng.uniform(100, 260)
    out = np.zeros(n)
    i = int(rng.uniform(0, 2) * fs)
    while i < n:
        for _ in range(rng.integers(2, 6)):
            length = int(rng.uniform(0.18, 0.4) * fs)
            sweep = np.linspace(0, rng.uniform(1.5, 4), length) + rng.uniform(0, 6)
            f0 = f0_base * (1 + 0.15 * np.sin(sweep))
            pulses = np.diff(np.floor(np.cumsum(f0) / fs), prepend=0.0)
            glottal = signal.lfilter([1.0], [1, -0.9], pulses)
            formants = rng.choice(len(VOWELS))
            syllable = sum(
                _resonator(glottal, f, 60 + 30 * k, fs) * 0.6**k
                for k, f in enumerate(VOWELS[formants])
            )
            syllable *= signal.windows.tukey(length, 0.4)
            end = min(i + length, n)
            out[i:end] = syllable[: end - i]
            i += length + int(rng.uniform(0.03, 0.1) * fs)
            if i >= n:
                break
        i += int(rng.uniform(1, 4) * fs)
    return out


def load_wav_repeated(path: str, rng, n: int, fs: int) -> np.ndarray:
    """Repeat a recording with random pauses, like someone calling out."""
    wav_fs, clip = wavfile.read(path)
    clip = clip.astype(float)
    if clip.ndim == 2:
        clip = clip.mean(axis=1)
    if wav_fs != fs:
        clip = signal.resample_poly(clip, fs, wav_fs)
    out = np.zeros(n)
    i = int(rng.uniform(0, 2) * fs)
    while i < n:
        end = min(i + clip.size, n)
        out[i:end] = clip[: end - i]
        i += clip.size + int(rng.uniform(1, 4) * fs)
    return out


def _harmonic_stack(phase: np.ndarray, top_harmonic: int, rng) -> np.ndarray:
    return sum(
        np.cos(h * phase + rng.uniform(0, 2 * np.pi)) / h for h in range(1, top_harmonic + 1)
    )


def synth_generator(rng, n: int, fs: int) -> np.ndarray:
    """Steady machine hum: harmonic like a voice, but with no syllables."""
    f0 = rng.uniform(85, 130) * (1 + 0.002 * pink_noise(rng, n))
    return _harmonic_stack(2 * np.pi * np.cumsum(f0) / fs, 30, rng)


def synth_engine(rng, n: int, fs: int) -> np.ndarray:
    """Engine with drifting revs, so its pitch moves, plus broadband roar."""
    drift = signal.resample(rng.standard_normal(max(n // fs // 3, 4)), n)
    f0 = rng.uniform(100, 180) * (1 + 0.1 * drift / max(np.abs(drift).max(), 1e-9))
    tonal = _harmonic_stack(2 * np.pi * np.cumsum(f0) / fs, 20, rng)
    return tonal / tonal.std() + 0.5 * pink_noise(rng, n)


def synth_birds(rng, n: int, fs: int) -> np.ndarray:
    """Bursts of fast rising chirps above the main speech band."""
    out = np.zeros(n)
    i = int(rng.uniform(0, 2) * fs)
    while i < n:
        for _ in range(rng.integers(3, 9)):
            length = int(rng.uniform(0.05, 0.15) * fs)
            freq = np.linspace(rng.uniform(2500, 4000), rng.uniform(4000, 6000), length)
            chirp = np.sin(2 * np.pi * np.cumsum(freq) / fs) * np.hanning(length)
            end = min(i + length, n)
            out[i:end] = chirp[: end - i]
            i += length + int(rng.uniform(0.04, 0.12) * fs)
            if i >= n:
                break
        i += int(rng.uniform(1, 5) * fs)
    return out


def pink_noise(rng, n: int) -> np.ndarray:
    spectrum = np.fft.rfft(rng.standard_normal(n))
    spectrum[1:] /= np.sqrt(np.arange(1, spectrum.size))
    spectrum[0] = 0
    x = np.fft.irfft(spectrum, n)
    return x / x.std()


def source_signal(source: Source, rng, n: int, fs: int) -> np.ndarray:
    """Source pressure at 1 m, in Pa."""
    if source.wav:
        x = load_wav_repeated(source.wav, rng, n, fs)
    else:
        x = {
            "voice": synth_voice,
            "generator": synth_generator,
            "engine": synth_engine,
            "birds": synth_birds,
        }[source.kind](rng, n, fs)
    level = _active_rms(x, fs)
    return x * spl_to_rms(source.spl_1m) / level if level else x


def _delayed_spectra(x_spectrum, freqs, delays, gains) -> np.ndarray:
    """Exact fractional delay per mic, applied in the frequency domain."""
    return x_spectrum * gains[:, None] * np.exp(-2j * np.pi * freqs * delays[:, None])


def _diffuse_spectra(rng, n_fft, freqs, mics, spl, n_sources=24) -> np.ndarray:
    """Background from many distant directions, so mics are realistically correlated."""
    rel = mics - mics.mean(axis=0)
    total = np.zeros((mics.shape[0], freqs.size), dtype=complex)
    gain = np.full(mics.shape[0], spl_to_rms(spl) / np.sqrt(n_sources))
    for _ in range(n_sources):
        angle = rng.uniform(0, 2 * np.pi)
        direction = np.array([np.cos(angle), np.sin(angle), 0.0])
        delays = -(rel @ direction) / SPEED_OF_SOUND
        total += _delayed_spectra(fft.rfft(pink_noise(rng, n_fft)), freqs, delays, gain)
    return total


def _wind(rng, n: int, n_mics: int, spl: float, fs: int) -> np.ndarray:
    """Low-frequency turbulence with gusts, separate at each mic."""
    sos = signal.butter(2, 400, fs=fs, output="sos")
    gust_sos = signal.butter(2, 0.3, fs=fs, output="sos")
    gust = 1 + 0.5 * np.tanh(signal.sosfiltfilt(gust_sos, rng.standard_normal(n)) * 60)
    wind = np.column_stack(
        [signal.sosfilt(sos, pink_noise(rng, n)) * gust for _ in range(n_mics)]
    )
    return wind * spl_to_rms(spl) / wind.std()


def simulate_site(scene: Scene, site: Site, cfg: ArrayConfig = ArrayConfig()) -> dict:
    """Return "voice" and "noise" stems, each (samples, mics), in Pa."""
    site_index = scene.sites.index(site)
    rng = np.random.default_rng([scene.seed, site_index])
    n = int(scene.scan_seconds * cfg.fs)
    mics = mic_positions_world(site, cfg)

    distances = [np.linalg.norm(mics - np.asarray(s.position, float), axis=1) for s in scene.sources]
    furthest = max((d.max() for d in distances), default=0.0)
    pad = int(np.ceil(furthest / SPEED_OF_SOUND * cfg.fs)) + 64  # room for the delay
    n_fft = fft.next_fast_len(n + pad, real=True)
    freqs = fft.rfftfreq(n_fft, 1 / cfg.fs)

    voice = np.zeros((cfg.n_mics, freqs.size), dtype=complex)
    noise = _diffuse_spectra(rng, n_fft, freqs, mics, scene.ambient_spl)
    for source, distance in zip(scene.sources, distances):
        x = fft.rfft(source_signal(source, rng, n_fft, cfg.fs))
        gains = 1 / np.maximum(distance, MIN_DISTANCE)
        target = voice if source.kind == "voice" else noise
        target += _delayed_spectra(x, freqs, distance / SPEED_OF_SOUND, gains)

    def to_time(spectra):
        return fft.irfft(spectra, n_fft, axis=1)[:, pad : pad + n].T

    wind = _wind(rng, n, cfg.n_mics, scene.wind_spl, cfg.fs)
    return {
        "voice": to_time(voice).astype(np.float32),
        "noise": (to_time(noise) + wind).astype(np.float32),
    }


class VoiceClassifier:
    """YAMNet, reduced to one question: human or not?"""

    def __init__(self):
        try:
            from ai_edge_litert.interpreter import Interpreter
        except ImportError as error:
            raise ImportError("run: pip install -r requirements.txt") from error
        self._model = Interpreter(model_path=str(MODEL_DIR / "yamnet.tflite"), num_threads=1)
        self._model.allocate_tensors()
        self._input = self._model.get_input_details()[0]["index"]
        self._output = self._model.get_output_details()[0]["index"]
        with open(MODEL_DIR / "yamnet_class_map.csv", newline="") as f:
            names = [row["display_name"] for row in csv.DictReader(f)]
        self._human = [i for i, name in enumerate(names) if name in HUMAN_CLASSES]
        self._animal = [i for i, name in enumerate(names) if name in ANIMAL_CLASSES]
        if len(self._human) != len(HUMAN_CLASSES) or len(self._animal) != len(ANIMAL_CLASSES):
            raise ValueError("class names do not match models/yamnet_class_map.csv")

    def voice_score(self, x: np.ndarray, fs: int) -> float:
        """Best human-minus-animal score over the windows of x, from -1 to 1."""
        if fs != CLASSIFIER_FS:
            x = signal.resample_poly(x, CLASSIFIER_FS, fs)
        x = np.pad(x, (0, max(CLASSIFIER_WINDOW - x.size, 0)))
        rms = np.sqrt(np.mean(x**2))
        if rms == 0:
            return 0.0
        x = np.clip(x * (0.1 / rms), -1, 1).astype(np.float32)

        best = -1.0
        last = x.size - CLASSIFIER_WINDOW
        for start in range(0, last + 1, CLASSIFIER_WINDOW // 2):
            self._model.set_tensor(self._input, x[start : start + CLASSIFIER_WINDOW])
            self._model.invoke()
            scores = self._model.get_tensor(self._output)[0]
            best = max(best, float(scores[self._human].max() - scores[self._animal].max()))
        return best


@lru_cache(maxsize=1)
def get_classifier() -> VoiceClassifier:
    return VoiceClassifier()


def _beamform(audio: np.ndarray, arrival: np.ndarray, fs: int) -> np.ndarray:
    """Align the mics on one direction and average. audio is (samples, mics)."""
    freqs = np.fft.rfftfreq(audio.shape[0], 1 / fs)
    aligned = np.fft.rfft(audio, axis=0) * np.exp(2j * np.pi * freqs[:, None] * arrival)
    return np.fft.irfft(aligned.mean(axis=1), audio.shape[0])


def analyse_blocks(
    audio: np.ndarray,
    fs: int,
    mic_pos: np.ndarray,
    dcfg: DetectorConfig = DetectorConfig(),
) -> list[BlockResult]:
    """Find a direction and score it for voice in every block. audio is (samples, mics)."""
    classifier = get_classifier()
    freqs, _, spec = signal.stft(
        audio.T,
        fs,
        nperseg=dcfg.nperseg,
        noverlap=dcfg.nperseg - dcfg.hop,
        boundary=None,
        padded=False,
    )
    band = (freqs >= dcfg.doa_band_hz[0]) & (freqs <= dcfg.doa_band_hz[1])
    freqs = freqs[band]
    spec = np.ascontiguousarray(spec[:, band, :].transpose(1, 0, 2))  # (bins, mics, frames)

    # Weight each bin by how far it rises above its own quietest moments.
    power = np.mean(np.abs(spec) ** 2, axis=1)
    floor = np.percentile(power, dcfg.noise_percentile, axis=1, keepdims=True)
    gain = np.clip(1 - dcfg.noise_margin * floor / np.maximum(power, 1e-30), 0.0, 1.0)

    azimuths = np.arange(0, 360, dcfg.azimuth_step_deg)
    rad = np.radians(azimuths)
    directions = np.column_stack([np.cos(rad), np.sin(rad), np.zeros_like(rad)])
    rel = mic_pos - mic_pos.mean(axis=0)
    arrival = -(directions @ rel.T) / SPEED_OF_SOUND  # (azimuths, mics)
    steering = np.exp(2j * np.pi * freqs[:, None, None] * arrival[None])  # (bins, az, mics)
    phat = spec / np.maximum(np.abs(spec), 1e-30)

    frame_rate = fs / dcfg.hop
    block = int(dcfg.block_s * frame_rate)
    step = int(dcfg.block_hop_s * frame_rate)
    block_samples = int(dcfg.block_s * fs)
    results = []
    for start in range(0, spec.shape[2] - block + 1, step):
        frames = slice(start, start + block)
        steered = steering @ phat[:, :, frames]
        doa_power = np.einsum("baf,bf->a", np.abs(steered) ** 2, gain[:, frames] ** 2)
        best = int(np.argmax(doa_power))

        first = start * dcfg.hop
        beam = _beamform(audio[first : first + block_samples].astype(float), arrival[best], fs)
        score = classifier.voice_score(beam, fs)
        results.append(
            BlockResult(
                start_s=start / frame_rate,
                azimuth_deg=float(azimuths[best]),
                voice_score=score,
                is_voice=score >= dcfg.voice_score_min,
            )
        )
    return results


def _angle_gap(a, b) -> np.ndarray:
    return np.abs((np.asarray(a, float) - b + 180) % 360 - 180)


def cluster_blocks(blocks: list[BlockResult], dcfg: DetectorConfig) -> list[Detection]:
    """Group voice blocks by direction. Noise rarely agrees on one bearing."""
    remaining = [b for b in blocks if b.is_voice]
    detections = []
    while remaining:
        azimuths = np.array([b.azimuth_deg for b in remaining])
        neighbours = (_angle_gap(azimuths[:, None], azimuths[None, :]) <= dcfg.cluster_deg).sum(1)
        centre = azimuths[int(np.argmax(neighbours))]
        members = [b for b in remaining if _angle_gap(b.azimuth_deg, centre) <= dcfg.cluster_deg]
        remaining = [b for b in remaining if b not in members]
        if len(members) < dcfg.min_blocks:
            continue

        rad = np.radians([b.azimuth_deg for b in members])
        mean = np.arctan2(np.sin(rad).mean(), np.cos(rad).mean())
        spread = _angle_gap(np.degrees(rad), np.degrees(mean))
        detections.append(
            Detection(
                bearing_deg=round(float(np.degrees(mean) % 360), 1),
                bearing_uncertainty_deg=round(max(float(np.sqrt(np.mean(spread**2))), 5.0), 1),
                voice_blocks=len(members),
                first_heard_s=round(min(b.start_s for b in members), 1),
                last_heard_s=round(max(b.start_s for b in members) + dcfg.block_s, 1),
            )
        )
    return sorted(detections, key=lambda d: -d.voice_blocks)


def detect_voice(
    audio: np.ndarray,
    fs: int,
    mic_pos: np.ndarray,
    dcfg: DetectorConfig = DetectorConfig(),
) -> list[Detection]:
    return cluster_blocks(analyse_blocks(audio, fs, mic_pos, dcfg), dcfg)


def build_report(results: list[tuple[Site, list[Detection]]], dcfg: DetectorConfig) -> dict:
    sites, flags = [], []
    for site, detections in results:
        status = "voice_detected" if detections else "no_voice_heard"
        sites.append({"site_id": site.id, "position": list(site.position), "status": status})
        for d in detections:
            high = d.voice_blocks >= dcfg.high_priority_blocks
            flags.append(
                {
                    "flag_id": f"{site.id}-{len(flags) + 1}",
                    "site_id": site.id,
                    "site_position": list(site.position),
                    "priority": "high" if high else "low",
                    "first_heard_time": round(site.t0 + d.first_heard_s, 1),
                    "last_heard_time": round(site.t0 + d.last_heard_s, 1),
                    **{k: v for k, v in asdict(d).items() if "heard" not in k},
                }
            )
    flags.sort(key=lambda f: (f["priority"] != "high", -f["voice_blocks"]))
    return {
        "schema_version": SCHEMA_VERSION,
        "bearing_convention": "degrees counter-clockwise from world +X",
        "note": "no_voice_heard is not clearance: a silent person cannot be detected",
        "sites": sites,
        "flags": flags,
    }


def true_bearings(site: Site, sources: tuple[Source, ...]) -> list[dict]:
    site_pos = np.asarray(site.position, float)
    rows = []
    for source in sources:
        offset = np.asarray(source.position, float) - site_pos
        rows.append(
            {
                "kind": source.kind,
                "bearing_deg": round(float(np.degrees(np.arctan2(offset[1], offset[0])) % 360), 1),
                "range_m": round(float(np.linalg.norm(offset)), 1),
            }
        )
    return rows


def band_snr_db(stems: dict, cfg: ArrayConfig) -> float | None:
    """Voice-band SNR at the mics, measured while the voice is sounding."""
    sos = signal.butter(4, (300.0, 3400.0), "bandpass", fs=cfg.fs, output="sos")
    voice = signal.sosfiltfilt(sos, stems["voice"][:, 0].astype(float))
    noise = signal.sosfiltfilt(sos, stems["noise"][:, 0].astype(float))
    level = _active_rms(voice, cfg.fs)
    if level == 0:
        return None
    return round(float(20 * np.log10(level / noise.std())), 1)


def run_scene(
    scene: Scene,
    out_dir: str | Path,
    cfg: ArrayConfig = ArrayConfig(),
    dcfg: DetectorConfig = DetectorConfig(),
) -> dict:
    """Simulate and scan every site, writing audio, ground truth and flags.json."""
    out = Path(out_dir)
    results, truth = [], {}
    for site in scene.sites:
        stems = simulate_site(scene, site, cfg)
        audio = stems["voice"] + stems["noise"]
        mic_pos = mic_positions_world(site, cfg)
        results.append((site, detect_voice(audio, cfg.fs, mic_pos, dcfg)))

        site_dir = out / f"site_{site.id}"
        site_dir.mkdir(parents=True, exist_ok=True)
        np.savez(site_dir / "mics.npz", audio=audio, fs=cfg.fs, t0=site.t0, mic_pos_world=mic_pos)
        truth[site.id] = {
            "sources": true_bearings(site, scene.sources),
            "voice_band_snr_db": band_snr_db(stems, cfg),
        }

    report = build_report(results, dcfg)
    (out / "flags.json").write_text(json.dumps(report, indent=2) + "\n")
    (out / "truth.json").write_text(json.dumps(truth, indent=2) + "\n")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", required=True, help="run folder to write")
    parser.add_argument("--scene", help="scene .json; omit for the built-in demo")
    parser.add_argument("--seed", type=int, default=0, help="built-in demo only")
    args = parser.parse_args()

    scene = load_scene(args.scene) if args.scene else demo_scene(args.seed)
    report = run_scene(scene, args.out)
    for site in report["sites"]:
        print(f"Site {site['site_id']}: {site['status']}")
    for flag in report["flags"]:
        print(
            f"  FLAG {flag['flag_id']} ({flag['priority']}): bearing {flag['bearing_deg']} "
            f"+/- {flag['bearing_uncertainty_deg']} deg, {flag['voice_blocks']} voice blocks"
        )


if __name__ == "__main__":
    main()

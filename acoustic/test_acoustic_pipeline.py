"""Physics, detection and handoff checks for acoustic_pipeline.py."""
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy import signal
from scipy.io import wavfile

import acoustic_pipeline as ap

CFG = ap.ArrayConfig()
SITE = ap.Site("A", (0.0, 0.0, 0.0))


def yawed_site(yaw_deg):
    half = np.radians(yaw_deg) / 2
    return ap.Site("A", (0.0, 0.0, 0.0), (float(np.cos(half)), 0.0, 0.0, float(np.sin(half))))


def scene(*sources, site=SITE, seconds=20.0, seed=0, **levels):
    return ap.Scene((site,), sources, seconds, seed=seed, **levels)


def scan(sc):
    site = sc.sites[0]
    stems = ap.simulate_site(sc, site, CFG)
    audio = stems["voice"] + stems["noise"]
    return ap.detect_voice(audio, CFG.fs, ap.mic_positions_world(site, CFG))


def at_bearing(distance, deg, z=0.3):
    rad = np.radians(deg)
    return (distance * np.cos(rad), distance * np.sin(rad), z)


def measured_delay(a, b, fs):
    """Seconds by which b lags a, with parabolic sub-sample refinement."""
    corr = signal.correlate(b.astype(float), a.astype(float), method="fft")
    i = int(np.argmax(corr))
    y0, y1, y2 = corr[i - 1 : i + 2]
    return (i + 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) - (a.size - 1)) / fs


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0])
def test_inter_mic_delay_matches_geometry_and_attitude(yaw_deg):
    site = yawed_site(yaw_deg)
    position = (8.0, 0.0, 1.0)
    sc = scene(ap.Source("voice", position, 85.0), site=site, ambient_spl=0.0, wind_spl=0.0)
    voice = ap.simulate_site(sc, site, CFG)["voice"]

    mics = ap.mic_positions_world(site, CFG)
    distance = np.linalg.norm(mics - np.asarray(position), axis=1)
    want = (distance[4] - distance[0]) / ap.SPEED_OF_SOUND
    assert measured_delay(voice[:, 0], voice[:, 4], CFG.fs) == pytest.approx(want, abs=0.1 / CFG.fs)


def test_voice_level_follows_inverse_distance():
    sc = scene(ap.Source("voice", (20.0, 0.0, 0.1), 85.0), ambient_spl=0.0, wind_spl=0.0)
    voice = ap.simulate_site(sc, SITE, CFG)["voice"][:, 0].astype(float)
    level = 20 * np.log10(ap._active_rms(voice, CFG.fs) / ap.P_REF)
    assert level == pytest.approx(85.0 - 20 * np.log10(20.0), abs=0.5)


@pytest.mark.parametrize("bearing", [30.0, 200.0])
def test_nearby_voice_is_flagged_on_the_right_bearing(bearing):
    detections = scan(scene(ap.Source("voice", at_bearing(12, bearing), 85.0), seconds=30.0))
    assert detections
    assert ap._angle_gap(detections[0].bearing_deg, bearing) <= 10


def test_bearing_is_in_world_frame_when_drone_is_yawed():
    site = yawed_site(70.0)
    sc = scene(ap.Source("voice", at_bearing(12, 30.0), 85.0), site=site, seconds=30.0)
    assert ap._angle_gap(scan(sc)[0].bearing_deg, 30.0) <= 10


def test_two_callers_give_two_flags():
    sc = scene(
        ap.Source("voice", at_bearing(10, 20.0), 85.0),
        ap.Source("voice", at_bearing(10, 200.0), 85.0),
        seconds=30.0,
    )
    found = [d.bearing_deg for d in scan(sc)]
    assert min(ap._angle_gap(found, 20.0)) <= 15
    assert min(ap._angle_gap(found, 200.0)) <= 15


@pytest.mark.parametrize(
    "sources",
    [
        (),
        (ap.Source("generator", (20.0, 5.0, 0.5), 85.0),),
        (ap.Source("engine", (20.0, 5.0, 0.5), 85.0),),
        (ap.Source("birds", (10.0, 5.0, 3.0), 75.0),),
    ],
    ids=["ambient", "generator", "engine", "birds"],
)
def test_non_voice_scenes_are_not_flagged(sources):
    assert scan(scene(*sources, seconds=30.0)) == []


def test_continuous_calling_is_not_treated_as_background():
    """Regression: the old detector removed anything that sounded most of the time."""
    clip = str(Path(ap.__file__).parent / "recordings" / "voice_08.wav")
    assert scan(scene(ap.Source("voice", at_bearing(10, 60.0), 85.0, clip), seconds=30.0))


def test_wav_sources_replace_synthetic_sound(tmp_path):
    path = tmp_path / "clip.wav"
    t = np.arange(CFG.fs) / CFG.fs
    wavfile.write(path, CFG.fs, (np.sin(2 * np.pi * 440 * t) * 2e4).astype(np.int16))
    x = ap.source_signal(ap.Source("recording", (1, 0, 0), 80.0, str(path)), np.random.default_rng(0), CFG.fs * 6, CFG.fs)
    freqs, power = signal.welch(x, CFG.fs, nperseg=4096)
    assert freqs[np.argmax(power)] == pytest.approx(440, abs=5)
    assert 20 * np.log10(ap._active_rms(x, CFG.fs) / ap.P_REF) == pytest.approx(80.0, abs=0.5)


def test_same_seed_reproduces_and_new_seed_differs():
    sc = scene(ap.Source("voice", (10.0, 0.0, 0.3), 85.0), seconds=3.0)
    a = ap.simulate_site(sc, SITE, CFG)["noise"]
    b = ap.simulate_site(sc, SITE, CFG)["noise"]
    c = ap.simulate_site(replace(sc, seed=1), SITE, CFG)["noise"]
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_scene_file_runs_end_to_end(tmp_path):
    path = tmp_path / "scene.json"
    path.write_text(
        json.dumps(
            {
                "sites": [
                    {"id": "near", "position": [0, 0, 0], "t0": 100.0},
                    {"id": "far", "position": [900, 0, 0]},
                ],
                "sources": [{"kind": "voice", "position": [10, 5, 0.3], "spl_1m": 85}],
                "scan_seconds": 30,
            }
        )
    )
    report = ap.run_scene(ap.load_scene(path), tmp_path / "run")

    assert json.loads((tmp_path / "run" / "flags.json").read_text()) == report
    assert [s["status"] for s in report["sites"]] == ["voice_detected", "no_voice_heard"]
    assert report["flags"][0]["site_id"] == "near"
    assert report["flags"][0]["first_heard_time"] >= 100.0
    assert "not clearance" in report["note"]
    with np.load(tmp_path / "run" / "site_near" / "mics.npz") as mics:
        assert mics["audio"].shape == (30 * CFG.fs, CFG.n_mics)
        assert mics["audio"].dtype == np.float32


def test_bad_sources_are_rejected():
    with pytest.raises(ValueError, match="unknown source kind"):
        ap.Source("dragon", (0, 0, 0), 80.0)
    with pytest.raises(ValueError, match="needs a wav"):
        ap.Source("recording", (0, 0, 0), 80.0)

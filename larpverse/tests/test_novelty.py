"""Stage-1 novelty detector tests (no network).  .venv-audio/bin/python -m pytest -q tests/test_novelty.py"""
import os

import numpy as np
import pytest

from acoustic import novelty as nv

FS = nv.FS


def _bed(seconds=20.0, seed=0, level=4e-3):
    rng = np.random.default_rng(seed)
    from scipy import signal as sps
    n = int(seconds * FS)
    b, a = sps.butter(2, 120 / (FS / 2))
    wind = sps.lfilter(b, a, rng.normal(0, 1, n))
    wind = wind / np.sqrt(np.mean(wind ** 2)) * 3 * level
    return wind + rng.normal(0, level, n)


def _stub(label="likely_human", other="Dog"):
    def clf(clip):
        return {"label": label, "reason": "stub", "human_score": 0.9 if label == "likely_human" else 0.05,
                "other_score": 0.8 if label == "likely_other" else 0.0, "background_score": 0.0, "vad_top": 0.9,
                "vad_frac": 0.5, "knock_periodicity": 0.0, "human_class": "Speech", "other_class": other,
                "yamnet_top": [["Speech", 0.9]], "sound_type": "speech"}
    return clf


def _tone_burst(x, t0, dur, f=800.0, amp=0.02):
    t = np.arange(int(dur * FS)) / FS
    s = int(t0 * FS)
    y = x.copy()
    y[s:s + len(t)] += amp * np.sin(2 * np.pi * f * t) * np.hanning(len(t))
    return y


def test_mel_filterbank_shape():
    fb = nv.mel_fb()
    assert fb.shape == (nv.N_MELS, nv.NFFT // 2 + 1) and np.all(fb.sum(axis=1) > 0)


def test_segment_hysteresis_merge_and_pad():
    nov = np.zeros(1000)
    nov[100:110] = 5.0          # event 1
    nov[150:160] = 5.0          # 0.64 s later: merged (gap < 1.6 s)
    nov[600:603] = 3.0          # never reaches THR_ON: ignored
    segs = nv.segment(nov)
    assert len(segs) == 1
    s0, s1 = segs[0]
    assert s0 < 100 * nv.HOP and s1 > 160 * nv.HOP


@pytest.mark.parametrize("denoise", ["wiener", "auto"])
def test_background_only_gives_no_candidates(denoise):
    x = _bed(seed=1)
    r = nv.detect(x, bg_seconds=6, classifier=_stub(), denoise_method=denoise)
    assert r["candidates"] == [] and r["groups"] == []
    assert r["novelty_background_p95"] < nv.THR_ON


def test_event_is_detected_where_it_happens():
    x = _tone_burst(_bed(seed=2), 8.0, 1.5)
    r = nv.detect(x, bg_seconds=6, classifier=_stub(), denoise_method="wiener", drone_id="d2")
    assert len(r["candidates"]) == 1
    c = r["candidates"][0]
    assert 7.3 <= c["t0"] <= 8.3 and 9.0 <= c["t1"] <= 10.3
    assert c["forwarded"] and c["dismissed"] is None and c["cand_id"] == "d2-n1"
    assert c["novelty_peak"] > nv.THR_ON


def test_impulsive_knocks_are_detected_by_flux():
    from acoustic import scene
    x = _bed(seed=3)
    k = scene.knocking(np.random.default_rng(0), dur=4.0)
    s = 8 * FS
    x[s:s + len(k)] += k / np.abs(k).max() * 0.05
    r = nv.detect(x, bg_seconds=6, classifier=_stub(), denoise_method="wiener")
    assert any(c["t0"] < 12 and c["t1"] > 8 for c in r["candidates"])


def test_minimum_statistics_profile_without_background():
    x = _tone_burst(_bed(seed=4), 12.0, 1.0)
    r = nv.detect(x, classifier=_stub(), denoise_method="wiener")
    assert "minimum statistics" in r["profile"]
    assert any(11 <= c["t0"] <= 12.5 for c in r["candidates"])


def test_explicit_background_with_an_event_inside_is_robust():
    bg = _tone_burst(_bed(seed=5), 3.0, 1.0)            # the background recording itself has an event
    x = _tone_burst(_bed(seed=6), 8.0, 1.0)
    r = nv.detect(x, background=bg, classifier=_stub(), denoise_method="wiener")
    assert any(7 <= c["t0"] <= 8.5 for c in r["candidates"])


def test_dismissed_other_is_kept_visible():
    x = _tone_burst(_bed(seed=7), 8.0, 1.0)
    r = nv.detect(x, bg_seconds=6, classifier=_stub("likely_other", "Bow-wow"), denoise_method="wiener")
    c = r["candidates"][0]
    assert not c["forwarded"] and c["dismissed"] == "dog" and r["groups"] == []


def test_forward_groups_merge_and_minimum_length():
    y = np.zeros(20 * FS)
    cands = [{"cand_id": f"n{i}", "t0": t0, "t1": t1, "novelty_peak": 5.0, "forwarded": True, **_stub()(None)}
             for i, (t0, t1) in enumerate([(8.0, 8.6), (10.0, 10.4), (16.0, 16.3)])]
    g = nv.forward_groups(cands, y, "d1")
    assert len(g) == 2 and g[0]["members"] == ["n0", "n1"]
    assert g[1]["t1"] - g[1]["t0"] >= nv.GROUP_MIN_S - 1e-6          # a 0.3 s blip gets context
    assert len(g[1]["clip"]) == int(g[1]["t1"] * FS) - int(g[1]["t0"] * FS)
    assert cands[0]["group"] == "d1-g1"


@pytest.mark.parametrize("h,o,b,vad,label", [
    (0.9, 0.1, 0.0, 0.9, "likely_human"),
    (0.05, 0.6, 0.0, 0.0, "likely_other"),
    (0.10, 0.25, 0.0, 0.0, "likely_other"),         # weak human, clear other
    (0.05, 0.05, 0.5, 0.1, "background"),
    (0.25, 0.30, 0.0, 0.2, "uncertain"),            # recall first: uncertain goes to stage 2
])
def test_triage_rules(h, o, b, vad, label):
    r = nv.triage(h, o, b, vad, 0.1, 0.0, "Speech", "Dog", [["Speech", h]])
    assert r["label"] == label


MODELS = os.path.join(os.path.dirname(nv.__file__), "models")


@pytest.mark.skipif(not os.path.exists(os.path.join(MODELS, "yamnet.tflite")), reason="acoustic models not installed")
def test_real_models_speech_vs_dog():
    pytest.importorskip("onnxruntime")
    pytest.importorskip("ai_edge_litert")
    from acoustic import scene
    rng = np.random.default_rng(0)
    tts = os.path.join(scene.DATA_DIR, "tts")
    wavs = sorted(f for f in os.listdir(tts) if f.endswith(".wav") and "-" not in f) if os.path.isdir(tts) else []
    if not wavs:
        pytest.skip("no cached TTS clip")
    import soundfile as sf
    speech, _ = sf.read(os.path.join(tts, wavs[0]))
    assert nv.classify_candidate(speech / np.abs(speech).max() * 0.3)["label"] == "likely_human"
    dog = scene.dog_bark(rng)
    assert nv.classify_candidate(dog / np.abs(dog).max() * 0.3)["label"] in ("likely_other", "uncertain")

"""Fast acoustic tests. Run with the audio venv:  .venv-audio/bin/python -m pytest tests/test_acoustic.py"""
import json

import numpy as np
import pytest

from acoustic import agent


@pytest.fixture(autouse=True)
def _no_llm(monkeypatch):
    """Tests never reach the network: no provider, no keys (agents.llm also reads .env)."""
    monkeypatch.setenv("LLM_PROVIDER", "none")
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)


def _feat(cid, transcript, cls, scores, vad, snr=12.0, width=25.0, n=3):
    base = {c: 0.0 for c in ("voice", "shout", "knocking", "dog", "engine", "debris", "noise")}
    base.update(scores)
    return {"clip_id": cid, "pile_id": "B3", "drone_id": "d1", "duration_s": 4.0, "continuous": False,
            "bearing_deg": 270.0, "width_deg": width, "snr_db": snr, "vad_speech_ratio": vad,
            "transcript": transcript, "sound_class": cls, "class_scores": base,
            "heuristics": {"knock_periodicity": 0.0, "hum_harmonicity": 0.1}, "yamnet_top": [],
            "doa": {"drones_agreeing": n, "drones_listening": 3, "tri_err_m": 1.0, "tri_resid_m": 0.3,
                    "assigned": True}}


@pytest.mark.parametrize("src", [(20.0, 10.0), (-15.0, -12.0), (3.0, 24.0)])
def test_srp_phat_doa_clean_source(src):
    pytest.importorskip("pyroomacoustics")
    from acoustic import dsp
    from acoustic.scene import single_source_scene
    sc = single_source_scene(src, (0.0, 0.0), duration=2.0)
    _, zs = dsp.srp_blocks(sc.signals["d1"], sc.mic_offsets, sc.fs)
    est = dsp.circ_peaks(zs, height=0.2)[0][0]
    truth = np.degrees(np.arctan2(src[1], src[0])) % 360
    assert abs(dsp.wrap180(est - truth)) <= 10.0


def _two_source_cross(az_strong, az_weak, ratio, seed=0, n=16000):
    """Synthetic far-field mixture on the 6-mic array: PHAT cross-spectra (P, F) and cell counts."""
    from acoustic import dsp
    from acoustic.scene import FS, mic_offsets
    offs = mic_offsets()
    model = dsp.SrpModel(offs, FS)
    rng = np.random.default_rng(seed)
    f = np.fft.rfftfreq(dsp.NFFT, 1 / FS)
    S = n // dsp.HOP
    X = 1e-3 * (rng.normal(size=(6, len(f), S)) + 1j * rng.normal(size=(6, len(f), S)))
    for az, g in ((az_strong, 1.0), (az_weak, ratio)):
        u = np.array([np.cos(np.radians(az)), np.sin(np.radians(az))])
        lead = offs.T @ u / 343.0
        src = g * (rng.normal(size=(len(f), S)) + 1j * rng.normal(size=(len(f), S)))
        X += src[None] * np.exp(2j * np.pi * f[None, :, None] * lead[:, None, None])
    C = model.cross(X)
    return model, C.sum(axis=2), np.full(C.shape[1], float(S))


def test_clean_removes_sidelobes_and_keeps_weak_source():
    pytest.importorskip("scipy")
    from acoustic import dsp
    model, R, w = _two_source_cross(40.0, 160.0, 0.6)
    comps = dsp.clean(model, R, w)
    az = [c[0] for c in comps]
    assert any(abs(dsp.wrap180(a - 40)) <= 4 for a in az)
    assert any(abs(dsp.wrap180(a - 160)) <= 4 for a in az)
    assert len(comps) <= 3                       # no ghost peaks from the strong source's sidelobes


def _scene_stub(drones, piles):
    from types import SimpleNamespace
    return SimpleNamespace(drones=drones, piles=piles)


def test_association_needs_two_consistent_drones_and_flags_collinear_piles():
    from acoustic import dsp
    piles = {"A": (0.0, 30.0), "B": (30.0, 0.0), "C": (60.0, 0.0), "D": (-30.0, 0.0)}
    drones = {"d1": (0.0, 0.0), "d2": (-8.0, -5.0), "d3": (8.0, -5.0)}
    g = dsp._Geom(_scene_stub(drones, piles))
    src = (0.5, 29.0)                            # a source at pile A
    comps = {d: [(dsp.bearing_to(o, src), 0.5, 30.0)] for d, o in drones.items()}
    ev = dsp.associate(g, comps)
    assert [e["pile"] for e in ev] == ["A"] and len(ev[0]["match"]) == 3
    post = dsp.bearing_posterior(g, {d: dsp.bearing_to(o, src) for d, o in drones.items()})
    assert post["A"] > 0.9
    # only one drone hears a weak component: no pile gets it
    weak = {"d1": [(dsp.bearing_to(drones["d1"], piles["B"]), 0.1, 30.0)], "d2": [], "d3": []}
    assert dsp.associate(g, weak) == []
    # B and C lie on one line seen from these drones: a source at C is ambiguous with B
    srcc = (59.0, 0.5)
    post = dsp.bearing_posterior(g, {d: dsp.bearing_to(o, srcc) for d, o in drones.items()})
    assert max(post["B"], post["C"]) < 0.9


def test_fallback_ranks_trapped_call_above_dog():
    help_ = _feat("a", "Help! I'm trapped, I can't move.", "voice", {"voice": 0.8, "noise": 0.2}, vad=0.6)
    dog = _feat("b", "", "dog", {"dog": 0.85, "voice": 0.1, "noise": 0.05}, vad=0.0, snr=18.0)
    jh, jd = agent.fallback_judge(help_), agent.fallback_judge(dog)
    assert jh["human"] > jd["human"] and jh["urgency"] > jd["urgency"]
    assert agent.clip_priority(jh) > agent.clip_priority(jd) + 30


def test_validation_and_replay_cache(tmp_path, monkeypatch):
    assert agent.validate_judgment({"human": 1.4, "urgency": 0.5, "reliability": 0.5, "rationale": "x"}) is None
    assert agent.validate_judgment({"human": 0.4, "urgency": 0.5, "rationale": "x"}) is None
    f = _feat("a", "help", "voice", {"voice": 0.9}, vad=0.5)
    cache = tmp_path / "decisions.jsonl"
    j, meta = agent.judge_clips([f], mode="auto", cache_path=str(cache))
    assert j["a"]["by"] == "fallback" and meta["llm_calls"] == 0
    rec = {"key": agent.feature_key(f), "clip_id": "a",
           "judgment": {"human": 0.7, "urgency": 0.6, "reliability": 0.5, "rationale": "recorded"}}
    cache.write_text(json.dumps(rec) + "\n")
    j, meta = agent.judge_clips([f], mode="replay", cache_path=str(cache))
    assert j["a"]["by"] == "llm" and j["a"]["rationale"] == "recorded" and meta["cache_hits"] == 1


# --------------------------------------------------------------------------- committed pipeline fixture
FIXTURE = __import__("os").path.join(__import__("os").path.dirname(__file__), "fixtures", "acoustic_seed7.json")
HUMAN_KINDS = {"voice", "weak_voice", "knock"}


@pytest.fixture(scope="module")
def seed7():
    """Real pipeline output for the seed-7 grid mission's listens (acoustic.eval_assign fixture)."""
    with open(FIXTURE) as f:
        return json.load(f)


def _words(t):
    import re
    return {w for w in re.findall(r"[a-z']+", (t or "").lower()) if len(w) > 2}


def test_fixture_no_confident_human_lead_on_a_pile_without_a_person(seed7):
    roles = seed7["truth_roles_evaluator_only"]          # evaluator-side assertion only
    for li in seed7["listens"]:
        for c in li["state"]["clips"]:
            if c["human"] >= 0.4 and c["association"] == "confident":
                assert roles[c["pile_id"]] in HUMAN_KINDS, (li["exclude"], c["pile_id"], c["transcript"])


def test_fixture_voice_ranked_first_and_faint_voice_is_a_lead(seed7):
    roles = seed7["truth_roles_evaluator_only"]
    first = seed7["listens"][0]["state"]
    voice = next(p for p, k in roles.items() if k == "voice")
    faint = next(p for p, k in roles.items() if k == "weak_voice")
    assert first["dispatch"]["pile_id"] == voice
    pr = {p["pile_id"]: p["priority"] for p in first["piles"]}
    assert pr[faint] >= 20                                # GridMission PRIORITY_MIN


def test_fixture_no_dog_clip_is_human_and_no_transcript_repeats_across_piles(seed7):
    for li in seed7["listens"]:
        clips = li["state"]["clips"]
        for c in clips:
            if c["sound_class"] == "dog":
                assert c["human"] < 0.4, c
        for i, a in enumerate(clips):
            for b in clips[i + 1:]:
                wa, wb = _words(a["transcript"]), _words(b["transcript"])
                if len(wa) >= 2 and len(wb) >= 2:
                    assert len(wa & wb) / min(len(wa), len(wb)) < 0.5, (a["pile_id"], b["pile_id"])


def test_fixture_excluded_piles_are_not_reported(seed7):
    for li in seed7["listens"]:
        st = li["state"]
        assert not {c["pile_id"] for c in st["clips"]} & set(li["exclude"])
        assert all(p["pile_id"] not in li["exclude"] for p in st["piles"])
        for c in st["clips"]:
            n, m = map(int, c["doa_agree"].split("/"))
            assert 0 <= n <= m


def _v1_state():
    return {"clips": [{"clip_id": "ac-B3", "pile_id": "B3", "association": "ambiguous"},
                      {"clip_id": "ac-A3", "pile_id": "A3", "association": "confident"}],
            "piles": [{"pile_id": "A3", "priority": 60, "status": "radar_ping", "why": "voice"},
                      {"pile_id": "B3", "priority": 19, "status": "lead", "why": "Lead (ambiguous direction): x"}],
            "dispatch": {"pile_id": "A3", "reason": "voice"}, "log": []}


def test_gemini_stage_cannot_lift_an_ambiguous_pile_and_failures_keep_v1(monkeypatch):
    pytest.importorskip("scipy")
    from acoustic import gemini_audio, run
    task = {"pile_id": "B3", "priority": 85, "status": "lead", "distress": ["trapped"], "languages": [],
            "reasons": ["voice"], "rationale": "voice says trapped"}
    monkeypatch.setattr(gemini_audio, "analyse_sector_audio", lambda items, mode="auto": {
        "prompt": "p", "mode": mode, "clips": {}, "sector_tasks": [task], "stage1": {}, "stage2": {}})
    st = run.add_gemini_audio(_v1_state(), [], mode="fallback")
    pr = {p["pile_id"]: p["priority"] for p in st["piles"]}
    assert pr["B3"] <= run.AMBIGUOUS_PRIORITY_CAP and st["dispatch"]["pile_id"] == "A3"

    def boom(items, mode="auto"):
        raise RuntimeError("network down")
    monkeypatch.setattr(gemini_audio, "analyse_sector_audio", boom)
    st = run.add_gemini_audio(_v1_state(), [], mode="auto")
    assert st["dispatch"]["pile_id"] == "A3" and "error" in st["gemini_audio"]
    assert run.gemini_audio_enabled(None, "fallback") is False      # LLM_PROVIDER=none in tests


def test_knock_pattern_beats_debris_bursts():
    from acoustic import models
    from acoustic.scene import FS, debris, knocking, through_rubble
    rng = np.random.default_rng(1)
    k = through_rubble(knocking(rng), 0.8)
    x = np.zeros(int(4.5 * FS))
    for t0 in (0.5, 2.6):
        d = debris(rng)
        x[int(t0 * FS):int(t0 * FS) + len(d)] += d
    assert models.knock_periodicity(k) >= 0.7
    assert models.knock_periodicity(through_rubble(x, 0.3)) < 0.3


def test_class_floor_and_whisper_gate():
    from acoustic import run
    knock = _feat("k", "", "knocking", {"knocking": 0.6}, vad=0.0)
    knock["heuristics"]["knock_periodicity"] = 0.7
    j = run.class_human_floor(knock, {"human": 0.2, "urgency": 0.1, "reliability": 0.8, "rationale": "x",
                                      "by": "fallback"})
    assert j["human"] >= agent.HUMAN_LEAD
    noisy = dict(knock, heuristics={"knock_periodicity": 0.0, "hum_harmonicity": 0.1})
    assert run.class_human_floor(noisy, dict(j, human=0.2))["human"] == 0.2      # rotor "knocking"
    dog = _feat("d", "", "voice", {"voice": 0.5, "dog": 0.4}, vad=0.0)
    assert run.class_human_floor(dog, dict(j, human=0.2))["human"] == 0.2        # bark read as voice
    llm = dict(j, human=0.1, by="llm")
    assert run.class_human_floor(knock, llm)["human"] == 0.1                      # LLM judgments untouched
    # shouted call that Silero misses: still transcribed and kept when YAMNet hears a human
    assert run.wants_asr(0.02, "shout", {"human_minus_animal": 0.5})
    assert run.gate_transcript("help me", 0.02, "shout", {"human_minus_animal": 0.5})[0] == "help me"
    assert run.gate_transcript("my name is trapp", 0.26, "dog", {"human_minus_animal": 0.0})[0] == ""


def test_busy_llm_falls_back_with_note(monkeypatch, tmp_path):
    from agents import llm

    def busy(*a, meta=None, **k):
        meta.update(busy=True, error="gemini HTTP 429")
        return None
    monkeypatch.setattr(llm, "call_json", busy)
    f = _feat("a", "help", "voice", {"voice": 0.9}, vad=0.5)
    j, meta = agent.judge_clips([f], mode="llm", cache_path=str(tmp_path / "c.jsonl"))
    assert j["a"]["by"] == "fallback" and j["a"]["by_note"] == "Gemini busy — using fallback"
    assert meta["busy"] and meta["fallback"] == 1


def test_rotor_veto_and_bearing_skip():
    from acoustic import dsp, models, run
    from acoustic.scene import FS, knocking, rotor_noise, through_rubble
    rng = np.random.default_rng(3)
    rot = rotor_noise(rng, 5.0)
    knock = through_rubble(knocking(rng, 5.0), 0.8) + 0.01 * rng.normal(size=int(5.0 * FS))
    assert models.loudness_steadiness(rot) >= run.ROTOR_STEADY > models.loudness_steadiness(knock)
    assert models.blade_pass(rot) >= run.ROTOR_COMB
    f = _feat("r", "", "knocking", {"knocking": 0.6}, vad=0.0)
    f["heuristics"].update(knock_periodicity=0.7, steadiness=0.6, blade_pass=0.4, human_minus_animal=0.0)
    j = run.class_human_floor(f, {"human": 0.7, "urgency": 0.4, "reliability": 0.8, "rationale": "x", "by": "llm"})
    assert j["human"] <= run.ROTOR_HUMAN_CAP
    f["heuristics"].update(steadiness=models.loudness_steadiness(knock),   # a knock train keeps its lead
                           blade_pass=models.blade_pass(knock))
    assert run.class_human_floor(f, dict(j, human=0.7))["human"] == 0.7
    comps = {"d1": [(10.0, 0.6, 30.0), (95.0, 0.4, 30.0)], "d2": [(200.0, 0.5, 30.0)]}
    kept = dsp.skip_rotor_components(comps, {"d1": [12.0], "d2": [90.0]})
    assert kept == {"d1": [(95.0, 0.4, 30.0)], "d2": [(200.0, 0.5, 30.0)]}

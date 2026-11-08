"""Acoustic priority list: a TRAPPED person vs people SAFE on the surface vs non-human noise.

A rescuer talking or walking on the rubble is a human sound, but not a search target. Each clip gets a
``tier`` (trapped > check > safe_surface > ignore) with a plain-English ``tier_why`` (<= 12 words), from:

* the sound family (voice / distressed voice / knocking / footsteps / non-human),
* the ``setting``: under_rubble (muffled: debris low-passes a buried sound, highs lost) | open_air (crisp,
  unmuffled) | unclear. Gemini judges it by ear (acoustic/gemini_audio.py); without an LLM the cheap DSP
  ``muffling`` feature below decides (energy above 2.5 kHz vs 300-2500 Hz, noise floor removed),
* explicit distress cues (help, trapped, injured, can't breathe, water): these ALWAYS win over open_air, so a
  person shouting "help, I'm trapped" is never dismissed.

safe_surface clips contribute ~0 to the pile priority (``dampen``), so the drones never fly to the rescuer.
Numpy + scipy only (imported by the numpy stub in sim/acoustic_grid.py too). Reads no ground truth.
"""
from __future__ import annotations

import re

import numpy as np

TIERS = ("trapped", "check", "safe_surface", "ignore")
SETTINGS = ("under_rubble", "open_air", "unclear")
PRIORITY_LIST = [
    {"tier": "trapped", "rank": 1, "label": "Trapped person: search target",
     "examples": ["voice or moaning muffled by rubble", "knocking or tapping from inside the debris",
                  "anyone calling help / trapped / injured, even in the open air"]},
    {"tier": "check", "rank": 2, "label": "Possible person: re-listen or check",
     "examples": ["weak or faint human-like sound", "voice with an unclear setting", "knocking in the open air"]},
    {"tier": "safe_surface", "rank": 3, "label": "Safe person on the surface: not a search target",
     "examples": ["rescuer or survivor talking calmly in the open air", "footsteps on the rubble surface"]},
    {"tier": "ignore", "rank": 4, "label": "Not a person: ignore",
     "examples": ["dog barking", "generator or machinery", "settling rubble", "wind and background noise"]},
]
TIER_RANK = {t["tier"]: t["rank"] for t in PRIORITY_LIST}

VOICE = {"voice", "shout", "speech", "call_for_help", "weak_voice", "surface_voice"}
DISTRESSED = {"scream", "crying", "moaning", "coughing"}
KNOCK = {"knocking", "knocking_tapping", "knock", "tapping"}
FOOT = {"footsteps"}
NONHUMAN_WORD = {"dog": "dog barking", "animal": "animal sound", "engine": "machinery noise",
                 "machinery": "machinery noise", "generator": "machinery noise", "debris": "settling rubble",
                 "settling": "settling rubble", "noise": "background noise", "other": "unclear noise"}
TRAPPED_CUES = ("call_for_help", "trapped", "injured", "cant_breathe", "water_request")
CUE_WORD = {"call_for_help": "help", "trapped": "trapped", "injured": "injured", "cant_breathe": "can't breathe",
            "water_request": "water"}
# explicit distress words in a transcript (English + the multilingual core of gemini_audio.KEYWORDS)
CUE_RE = {
    "call_for_help": r"\bhelp\b|hear me|anyone there|somebody|救命|救人|救救|來人|来人|\btolong\b|cứu|\bsaklolo\b|\btulong\b",
    "trapped": r"trapped|stuck|pinned|can'?t (get out|move)|cannot move|困住|卡住|壓住|压住|出不去|動不了|动不了|"
               r"terjebak|terjepit|mắc kẹt|bị kẹt|naipit",
    "injured": r"\bhurt|injur|bleed|\bpain\b|broken|受傷|受伤|流血|好痛|骨折|berdarah|bị thương|chảy máu|sugatan",
    "cant_breathe": r"can'?t breathe|cannot breathe|hard to breathe|no air|不能呼吸|無法呼吸|无法呼吸|喘不過|喘不过|"
                    r"sesak|không thở được|khó thở",
    "water_request": r"\bwater\b|thirst|喝水|口渴|air minum|\bhaus\b|\bnước\b|\btubig\b",
}
SAFE_MIN_WORDS = 3          # a voice is only "safe on the surface" when its calm words were actually understood
FAINT_HUMAN = 0.25          # below this human score a human-like sound is a "check", never a trapped lead
SAFE_HUMAN_CAP = 0.1        # a safe-surface clip's human-LEAD likelihood (the heard value is kept in human_heard)

# DSP muffling: 10*log10(E[2.5-7 kHz] / E[0.3-2.5 kHz]) over the active frames, per-band noise floor removed.
# Clean speech sits near -12..-18 dB and gravel footsteps near 0 dB; through 0.4 m of rubble (scene.through_rubble,
# fc 2.4 kHz) a voice drops ~8-10 dB, deeper sources more.
OPEN_AIR_DB = -24.0
UNDER_RUBBLE_DB = -32.0
LO_BAND, HI_BAND = (300.0, 2500.0), (4000.0, 7500.0)


def muffling(x, fs=16000):
    """Cheap DSP muffling feature of one clip -> {"hf_ratio_db", "muffled" 0-1, "setting"} or None (too short)."""
    from scipy.signal import stft
    x = np.asarray(x, float).ravel()
    if len(x) < int(0.25 * fs) or not np.any(x):
        return None
    f, _, Z = stft(x, fs, nperseg=512, noverlap=256, boundary=None, padded=False)
    P = np.abs(Z) ** 2
    lo = P[(f >= LO_BAND[0]) & (f < LO_BAND[1])].sum(axis=0)
    hi = P[(f >= HI_BAND[0]) & (f < min(HI_BAND[1], fs / 2))].sum(axis=0)
    if len(lo) < 4:
        return None
    quiet = lo <= np.percentile(lo, 25)
    nl, nh = float(lo[quiet].mean()), float(hi[quiet].mean())      # noise floor per band (quietest frames)
    act = lo > 4.0 * nl + 1e-20
    if act.sum() < 3:
        act = lo >= np.percentile(lo, 80)
    k = int(act.sum())
    sl = float(lo[act].sum()) - nl * k
    sh = max(float(hi[act].sum()) - nh * k, 0.0)
    if sl <= 0:
        return None
    db = float(np.clip(10 * np.log10(max(sh, 1e-12 * sl) / sl), -60.0, 20.0))
    muf = float(np.clip((OPEN_AIR_DB - db) / (OPEN_AIR_DB - UNDER_RUBBLE_DB), 0.0, 1.0))
    return {"hf_ratio_db": round(db, 1), "muffled": round(muf, 2), "setting": setting_from_db(db)}


def setting_from_db(db):
    if db is None:
        return "unclear"
    return "open_air" if db >= OPEN_AIR_DB else "under_rubble" if db <= UNDER_RUBBLE_DB else "unclear"


def cue_words(*texts):
    """Explicit distress cues named in transcripts (TRAPPED_CUES order)."""
    t = " ".join(x or "" for x in texts).lower()
    return [k for k, pat in CUE_RE.items() if re.search(pat, t)]


def n_words(text):
    """Intelligible words: latin-script words + CJK characters / 2 ([...] and ellipses do not count)."""
    t = re.sub(r"\[\.\.\.\]|…", " ", text or "")
    cjk = len(re.findall(r"[㐀-鿿]", t))
    return len(re.findall(r"[^\W\d_㐀-鿿]{2,}", t)) + cjk // 2


def classify(sound, setting="unclear", cues=(), words=0, human=None):
    """(tier, tier_why) for one sound. sound: v1 sound_class / Gemini sound_type / scenario kind."""
    s = str(sound or "other").lower()
    setting = setting if setting in SETTINGS else "unclear"
    cues = [c for c in TRAPPED_CUES if c in set(cues or ())]
    if cues:
        said = ", ".join(CUE_WORD[c] for c in cues[:2])
        return "trapped", f"distress words heard ({said}): treat as trapped person"
    faint = human is not None and human < FAINT_HUMAN
    if s in FOOT:
        if setting == "under_rubble":
            return "check", "footstep-like thuds muffled by rubble: check"
        return "safe_surface", "person walking on the surface, safe: not a search target"
    if s in VOICE or s in DISTRESSED or s in KNOCK:
        what = "knocking" if s in KNOCK else s if s in DISTRESSED else "voice"
        if faint:
            return "check", f"faint {what}-like sound: re-listen to check"
        if setting == "under_rubble":
            return "trapped", f"{what} muffled by rubble: likely trapped person"
        if setting == "open_air":
            if s in KNOCK:
                return "check", "knocking in the open air: could be rescuers, check"
            if s in DISTRESSED:
                return "check", f"{what} in the open air: may be injured, check"
            if words >= SAFE_MIN_WORDS:
                return "safe_surface", "person talking on the surface, safe: not a search target"
            return "check", "voice in the open air, words unclear: check"
        return "check", f"{what}, setting unclear: re-listen to check"
    if human is not None and human >= 0.4:
        return "check", "human-like sound of unclear type: check"
    return "ignore", f"{NONHUMAN_WORD.get(s, 'unclear noise')}: not a person, ignore"


def footsteps_label(yamnet_top):
    """True when YAMNet's top labels name footsteps / walking (v1 path; the 7 groups have no footsteps class)."""
    return any(re.search(r"footstep|walk", str(lbl), re.I) for lbl, *_ in (yamnet_top or []))


def tier_v1_clip(c, setting=None):
    """Tier fields for one larpverse.acoustic/v1 clip (v1 recognisers: class, transcript, human). In place."""
    sound = "footsteps" if footsteps_label(c.get("yamnet_top")) and c.get("sound_class") not in ("voice", "shout") \
        else c.get("sound_class")
    m = c.get("muffling")
    st = setting or (m or {}).get("setting") or "unclear"
    tier, why = classify(sound, st, cue_words(c.get("transcript")), n_words(c.get("transcript")),
                         c.get("human_heard", c.get("human")))
    c.update(setting=st, tier=tier, tier_why=why)
    return c


def dampen(c, priority_fn=None):
    """safe_surface: the clip stops counting as a search lead (human-lead likelihood capped, urgency 0, priority
    0); the heard human score is kept in human_heard. Any other tier restores a previously damped clip."""
    if c.get("tier") == "safe_surface":
        if "human_heard" not in c:
            c["human_heard"], c["urgency_heard"] = c.get("human"), c.get("urgency")
        c["human"] = min(float(c.get("human") or 0.0), SAFE_HUMAN_CAP)
        c["urgency"], c["priority"] = 0.0, 0
    elif "human_heard" in c:
        c["human"], c["urgency"] = c.pop("human_heard"), c.pop("urgency_heard", c.get("urgency"))
        if priority_fn is not None:
            c["priority"] = priority_fn(c)
    return c


def lead_clips(clips):
    """Clips that may make a pile a lead (safe-surface people never do)."""
    return [c for c in clips if c.get("tier") != "safe_surface"]


SAFE_PILE_WHY = ("Only people on the surface heard (talking / walking in the open air): safe, not a search "
                 "target. No trapped-person sound; not cleared.")

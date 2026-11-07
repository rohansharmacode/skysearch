# Acoustic triage with Gemini audio (stage 1 → 2 → 3)

Landed drones (rotors off) listen to nearby rubble sectors. An open-model detector runs locally and is always on. It
flags what is *not* background, and only those clips go to Gemini, which hears them in any language and reports
what was said and how urgent it sounds. Code then ranks the sectors. Nothing here says a person is confirmed:
every output is a candidate lead.

```
mic channel ─▶ STAGE 1 novelty.py  (local, open models)      background profile → denoise → novelty → candidates
                 likely_human / uncertain ──▶ STAGE 2 gemini_audio.analyse_clip   Gemini audio: language, transcript,
                 likely_other / background ──▶ "dismissed: dog" (kept visible)     English translation, distress cues
STAGE 3 gemini_audio.analyse_sector_audio: per sector  human × urgency(+distress) × direction reliability → ranked
        sector tasks; Gemini writes a one-line rationale per sector, code checks it → larpverse.acoustic/v1 fields
```

## Stage 1: `acoustic/novelty.py` (numpy/scipy + Silero VAD + YAMNet)

1. **Background profile** per drone/channel. It is learned from an explicit background recording when one is given:
   the first N s, or the whole listen with its loudest 30 % dropped so events inside it don't count. Without one it
   uses minimum statistics (the quietest 30 % of the clip).
2. **Denoise** with noisereduce (stationary spectral gating, MIT; installed in `.venv-audio`), or our own Wiener-style
   spectral subtraction if it is missing.
3. **Novelty** per 16 ms frame on the denoised 40-band log-mel, compared with the background:
   - `max(median5(0.5·(top-5 robust band z + Mahalanobis/√bands)), 0.5·spectral-flux z)`
   - The onset term is not smoothed, so knocks survive.
4. **Segmentation**:
   - hysteresis at 4.0 / 2.5
   - gaps < 1.6 s merged, so a knock pattern stays one clip
   - padded 0.25 s, ≤ 15 s per clip
5. **Triage** per candidate:
   - Inputs: Silero VAD (top-10 % frame speech probability), YAMNet groups (speech, shout, scream, cry, moan, cough,
     breathing, knock, tap, … vs dog, engine, siren, helicopter, chainsaw, glass, debris, …) and the knock-periodicity
     heuristic.
   - Labels: `likely_human`, `likely_other`, `background` or `uncertain`.
   - Recall comes first, so **uncertain is forwarded**. Only clear non-human or background is dismissed.
6. **Stage-2 units.** Forwarded candidates within 3 s are merged and padded to at least 2 s of context. This was
   learned live: a 1.1 s Mandarin fragment came back as "拜託救我 / Please save me" (confidence 0.95, prompt v1).

## Stage 2: `acoustic/gemini_audio.py` + `agents/llm.call_json_audio`

- **Request.** One clip per call: denoised mono 16 kHz WAV, ≤ 15 s (its loudest window), base64 `inline_data`
  (`audio/wav`).
- **Schema.** JSON mode with a responseSchema covering:
  - `human_vocal`, `sound_type` (speech | call_for_help | scream | crying | moaning | coughing | knocking_tapping |
    animal | machinery | other)
  - `languages[{code (BCP-47), name}]`, verbatim `transcript`, `translation_en`
  - `distress[]` (call_for_help, trapped, injured, cant_breathe, child, multiple_people, elderly, water_request,
    weak_or_fading, location_hint), `people_count`, `location_hints[]`
  - `confidence` 0–1, one-line `rationale`
- **Thinking.** `thinkingLevel: low` by default (`GEMINI_AUDIO_THINKING=low|high|off`). It cut latency from ~6 s to
  ~1.5–3 s per clip.
- **Prompt (v5).** A neutral listener with no rescue framing:
  - Most flagged clips are noise, animals, machinery or a radio.
  - Transcribe only words you can clearly make out, else "", never a plausible sentence.
  - The language code must match the transcript's script.
  - Distress cues only when the words say them or the voice is unmistakable.
  - Why the framing went: with a search-and-rescue framing (v1–v4), flash-lite invented distress on clean or noisy
    news speech. It produced "救命啊…炸彈爆炸" from Mandarin news, Spanish "Por favor ayúdame, soy trabajador" from
    Cantonese, and Thai "I'm here… third floor" from Cantonese. Naming the site (Tainan) also pushed language ID toward
    Mandarin/Hokkien. FLEURS invented-distress rate: v4 7/60 → v5 0/40 on the same zh/yue clips.
- **Code checks** (`validate_analysis`, `cross_check`, `verify_distress`):
  - `validate_analysis` enforces types and enums and rejects "confirmed / survivor found".
  - `cross_check` caps confidence at 0.6, with a note, when stage-1 VAD heard no speech or a long transcript comes from
    a < 1.5 s clip.
  - `verify_distress`: any answer carrying distress cues gets a second, independent, neutral "transcribe exactly" call
    on the same audio. If the words are not reproduced (similarity < 0.5) or the language differs, the cues move to
    `distress_unverified`, confidence is capped at 0.5 and the reason is noted. This costs one extra call only for
    distress clips (`GEMINI_AUDIO_VERIFY=0` turns it off).
- **Fallback** (`by: "fallback"`) on no key, an invalid answer, or HTTP failure:
  - local Whisper (a multilingual faster-whisper model if one is on disk, else base.en)
  - multilingual distress keywords (Mandarin, Hokkien Han, Indonesian, Vietnamese, Tagalog, English)
  - the stage-1 classes
- **Busy handling.** A 429/503 sets `by_note: "Gemini busy — using fallback"` and prefixes the rationale. Two 429s in a
  row open a 10-minute circuit, so a demo never waits on a dead quota.
- **Rate limits** (`agents/llm.py`, shared by every Gemini call in the process):
  - token bucket `GEMINI_RPM` (default 10/min, burst `GEMINI_BURST`=2), logging `[llm] throttled: waiting X s`
  - retries on 429/5xx with exponential backoff and jitter, honouring `Retry-After` / `retryDelay`
  - `LLM_RETRIES`=3, per-wait cap 20 s, total budget 45 s
  - a per-day-quota 429 fails at once
  - `latency_ms` excludes the client-side throttle wait, which is reported as `throttled_ms`
- **Record/replay.** Every validated answer goes to `agents/cache/llm.jsonl`, keyed by model + prompt + schema +
  SHA-256 of the WAV. The same clip replays offline (`LLM_REPLAY=1`, or `analyse_clip(mode="replay")`). Failures are
  never cached.
- **Model.** `GEMINI_AUDIO_MODEL` overrides the model for audio and rationale calls only (default `GEMINI_MODEL`, else
  `gemini-3.5-flash`). Replays need the same model that recorded them.

## Stage 3: prioritisation

Per sector (pile), for each analysed stage-2 unit:

- `human` = Gemini confidence (≤ 0.2 when it says animal or machinery). On the fallback path it is blended with the
  stage-1 human score.
- `urgency` = base by sound type (call for help / scream 0.75, moaning 0.65, crying 0.6, knocking 0.55, coughing 0.5,
  speech 0.4) plus distress boosts: can't breathe +0.3, injured +0.2, child +0.2, trapped +0.15, several people
  (or people_count ≥ 2) +0.15, weak +0.15, elderly +0.1, call for help +0.1, location +0.05, water +0.05; capped at 1.
- `reliability` = direction reliability from the multi-drone association (the clip's `reliability`), capped at 0.45
  when the association is not `confident`.
- `priority` = `100·human·(0.35+0.65·urgency)·(0.45+0.55·reliability)` (same shape as `agent.clip_priority`), +3 per
  extra human event.

Each sector task carries `reasons[]`. Gemini rewrites them into one actionable sentence (`rationale`, recorded for
replay). Code rejects an answer that names another sector, says confirmed/found/alive, or runs past 300 chars; the
template is used instead (`rationale_by: "template"`).

## Integration: the call `acoustic/run.py` needs (owned by the pipeline builder)

`acoustic.eval_gemini e2e --seed 3` already does this from outside, using `run.analyse` unchanged. Inside `analyse()`,
after `apply_association(...)` and `agent.aggregate(...)` have produced `clips`, `piles` and `dispatch`, and before the
return:

```python
from . import gemini_audio
items = [{"clip_id": f"ac-{s_['det']['pile_id']}", "pile_id": s_["det"]["pile_id"], "drone_id": s_["did"],
          "audio": s_["clip"], "background": scene.signals[s_["did"]][0],
          "reliability": clip_by[f"ac-{s_['det']['pile_id']}"]["reliability"],
          "association": s_["det"]["association"]}
         for s_ in seps if not s_["det"]["excluded"]]
ga = gemini_audio.analyse_sector_audio(items, mode=mode)     # mode: auto | live | replay | fallback
state = {...}                                                # the dict analyse() already builds
gemini_audio.merge_into_state(state, ga)                     # adds the v1.2 fields below, re-picks dispatch
```

## Contract additions: `larpverse.acoustic/v1` audio fields (to append to `contracts/INTERFACES.md`)

All additive: a v1 reader keeps working.

```jsonc
"clips": [ { /* v1 fields */,
  "novelty": 14.5,                      // stage-1 peak novelty (background ≈ 1-2, threshold 4)
  "forwarded": true,                    // something in this clip went to stage 2
  "dismissed": ["dismissed: dog"],      // stage-1 dismissals in this clip (kept visible)
  "language": "cmn", "languages": [{"code": "cmn", "name": "Mandarin"}],
  "transcript_original": "救命！我被困在樓梯下面…", "translation": "Help! I'm trapped under the stairs…",
  "distress": ["call_for_help", "trapped", "location_hint"],
  "audio_analysis": { "by": "gemini", "by_note": null, "sound_type": "call_for_help", "human_vocal": true,
                      "confidence": 0.95, "people_count": 1, "location_hints": ["under the stairs"],
                      "rationale": "...", "model": "gemini-3.5-flash-lite", "error": null,
                      "cross_check": null, "verify": {"ok": true, "similarity": 0.93, "...": "..."},
                      "distress_unverified": [], "t0": 7.3, "t1": 11.9 },
  "stage1": { "denoise": "noisereduce", "profile": "background 26.0 s (quietest 70%)",
              "events": [ {"cand_id": "d1-n1", "t0": 7.5, "t1": 11.7, "novelty_peak": 14.5,
                           "label": "likely_human", "reason": "Speech 0.92, VAD 1.00", "forwarded": true,
                           "dismissed": null, "group": "d1-g1"} ],
              "stage2_clips": [ {"cand_id": "d1-g1", "t0": 7.0, "t1": 12.2, "members": ["d1-n1"]} ] } } ],
"piles": [ { /* v1 */, "audio_priority": 83, "distress": ["trapped"], "languages": ["en"] } ],
"sector_tasks": [ { "rank": 1, "pile_id": "B1", "priority": 83,
                    "status": "lead",           // lead | uncertain_sound | dismissed_nonhuman | no_signal_not_cleared
                    "human": 0.9, "urgency": 1.0, "reliability": 0.85,
                    "languages": [{"code": "en", "name": "English"}], "distress": ["call_for_help", "trapped"],
                    "reasons": ["..."], "rationale": "...", "rationale_by": "gemini",   // or "template"
                    "top_event": "d1-g1", "events": 2, "forwarded": 1, "dismissed": [], "by": "gemini" } ],
"gemini_audio": { "prompt": "gemini-audio-v5", "mode": "auto",
                  "stage1": {"candidates": 5, "forwarded": 3, "dismissed": {"dog": 1}},
                  "stage2": {"calls": 3, "cached": 0, "fallback": 0, "errors": [], "latency_ms": 4500,
                             "audio_s": 13.5, "usage": {...}, "rationale": {"by": "gemini", ...}} }
```

With `reprioritise=True` (the default), `merge_into_state` raises a pile's `priority`/`why` to the stage-3 value
when that value is higher and the task is a `lead`, then re-picks `radar_ping` and `dispatch` from the top lead.

## Data

- **Speech:** Google FLEURS (CC BY 4.0), 10 dev clips (≤ 11 s) per language for cmn, yue, en, id, vi, fil, th, ja, ko.
  They are streamed from each language's dev tarball; the download stops early (47 MB total). Each clip is paired
  with the parallel English sentence as the translation reference. Details in `data/audio_real/fleurs/SOURCE.md`.
- **Distress phrases:** our own sentences voiced by Piper (MIT) in en, zh, vi and id.
- **Non-speech:** ESC-50 (CC BY-NC 3.0, eval/demo only) for crying, cough, breathing, sneeze, door knock, dog, siren,
  engine, chainsaw, helicopter, glass, fire, creak, rain and wind; `scene.py` knocks and debris.
- **Beds:** `ambient` (scene.py's rotors-off wind plus sensor noise), `rotor` (real DroneAudioSet drone-only
  recording: another drone hovering) and `generator` (ambient plus scene.py's diesel generator).

## Run it

```bash
bash scripts/set_gemini_key.sh                                    # once; key goes to .env (gitignored)
.venv-audio/bin/python -m acoustic.eval_gemini fetch              # data (~155 MB incl. 3 Piper voices)
scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini stage1 --bed ambient     # also rotor, generator
export GEMINI_AUDIO_MODEL=gemini-3.5-flash-lite                   # optional; must match for replay
scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini live --set distress --mode live
scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini live --set fleurs --mode live --langs zh   # per language
scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini live --set edge --mode live
scripts/heavy.sh .venv-audio/bin/python -m acoustic.eval_gemini e2e --seed 3 --mode live
.venv-audio/bin/python -m acoustic.eval_gemini report             # all tables again
LLM_REPLAY=1 ... --mode replay                                    # offline: recorded answers only
.venv-audio/bin/python -m pytest -q tests/test_novelty.py tests/test_gemini_audio.py
```

Without a key every command runs with `by: "fallback"`.

## Results (7 Nov 2026)

### Stage 1 (local; 34 human events and 22 non-human per bed, 6 background-only beds; profile from the first 6 s)

| bed | SNR | human recall (forwarded) | non-human false-forward | background false-forward |
|---|---|---|---|---|
| ambient (rotors off) | +10 / 0 / −5 dB | 100 % / 100 % / 97 % | 14 % / 36 % / 36 % | 0 % |
| generator | +10 / 0 / −5 dB | 100 % / 100 % / 88 % | 18 % / 32 % / 45 % | 0 % |
| rotor (drone hovering) | +10 / 0 / −5 dB | 100 % / 97 % / 56 % | 36 % / 55 % / 64 % | 33 % |

- Per class, over all beds and SNRs: speech in 4 languages 96–100 %, distress phrases 89–100 %, breathing 100 %,
  crying 89 %, cough 85 %, knocks 78–89 %.
- Rarely forwarded: rain, wind and engine 11 %, glass and helicopter 22 %, dog 37 %.
- Often forwarded: creak, fire and debris 78 %. These are uncertain to the open models, so Gemini sorts them.
- A hovering drone is the hard case at −5 dB (56 %), which supports the land-and-listen design.

### Stage 2, live (`gemini-3.5-flash-lite`, prompt v5)

LIVE_RESULTS

### Stage 2, fallback (no key; Whisper base.en)

- English only: language ID 9/10, CER 0.01, translation F1 0.75.
- zh, id and vi: 0/10 language ID. base.en cannot hear them.
- Distress-cue recall on the phrases: 0.28.
- ~1 s per clip on CPU.
- A multilingual faster-whisper model (`small` ≈ 480 MB, `base` ≈ 145 MB) would fix language ID offline. Not
  downloaded; see the requests.

## Limits

- **Free-tier quota.** The hackathon key is on the free tier: `gemini-3.5-flash` allows 20 requests/day (exhausted on
  7 Nov after 18 calls). Lite models have more. Use `GEMINI_RPM`, `GEMINI_AUDIO_MODEL` and replay for demos.
- **No real screams or whispers** under an open licence on disk. The phrases are TTS (the Piper x_low zh/vi voices are
  rough), and the beds are synthetic or recorded separately, not through real rubble.
- **Confidence is about a living person, not a broadcast.** Gemini scores calm narration (FLEURS) low on confidence
  ("likely broadcast media"). That is a reasonable judgement, but a calm trapped person could be under-ranked: urgency
  still comes from content.

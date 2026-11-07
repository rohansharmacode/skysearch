# Victim-dummy audio library

Audio that the victim dummies play in the Isaac stage. The drones' mic arrays have to pick it up, and the video soundtrack can let viewers hear it. Everything is in `assets/audio/dummies/` and described in `manifest.json` (schema `larpverse.dummy_audio/v1`). The library is built from files already on disk, with **no downloads**.

- **Format:** 48 kHz mono 16-bit WAV, loudness-matched per kind, peak at or below -1 dBFS (a peak limiter, no clipping).
  - Loudness is a gated RMS: 50 ms frames within 30 dB of the loudest frame. It is not LUFS.
  - Targets in dBFS: voice -20, weak voice -32 (12 dB down on purpose), knock -24, cough and cry -22, dog -20, generator -22, settling -24, siren -24, wind -26.
  - The manifest's `suggested_spl_db_1m` gives the playback level at 1 m, taken from `sim/callout.py` `LEVEL_DB`: shout 90, knock 88, weak voice 72 dB.
- **Size:** 30 base clips plus 96 "through rubble" variants, so 126 WAVs. Together with the 3 check WAVs (1 MB) in `check/` they take 57.2 MB, inside the 60 MB budget, so all of it is committed.
- **Bandwidth:** sources recorded at 16 or 22 kHz (the CC0 and ESC-50 sets, and Piper) have no content above 8 or 11 kHz after the upsample to 48 kHz. The synthesised knocks, generator and settling clips are full-band.

## Regenerate

    .venv-audio/bin/python scripts/build_dummy_audio.py                                  # clips + manifest (~30 s, ~300 MB RAM)
    scripts/heavy.sh .venv-audio/bin/python scripts/build_dummy_audio.py check           # seed-7 listen through acoustic/ (~2.5 min)
    .venv/bin/python -m pytest -q tests/test_dummy_audio.py                              # schema, files, durations, levels, clipping, size

Piper's sampling noise isn't seeded, so a rebuild changes the TTS clips slightly. After a rebuild, rerun `check`, because `pipeline_check.json` describes the committed WAVs.

## Clips

Each human clip (voice, weak_voice, knock, cough, cry) also has `rubble/<id>__{0.5,1.0,1.5,2.0}m.wav`. Distractors sit on the surface, so they have no rubble variants.

| id | kind | lang | transcript | s | source | licence |
|---|---|---|---|---|---|---|
| `voice_en_help_real_a` | voice | en | Help me! | 1.5 | Frances set: "help me" by Gemesil | CC0 |
| `voice_en_help_real_b` | voice | en | Help! Help me! | 1.8 | Frances set: "HELP HELP ME." by Legnalegna55 | CC0 |
| `voice_en_help_real_c` | voice | en | Help me! (repeated) | 5.0 | Frances set: "Help Me!.wav" by Bee_Abney | CC0 |
| `voice_en_please_real` | voice | en | Please help me, I'm so scared, please help me. | 6.0 | Frances set: "Please Help Me Im So Scared…" by DRFX | CC0 |
| `voice_en_call_tts` | voice | en | Help! Can anyone hear me? My leg is trapped! | 2.5 | Piper en_US-lessac-low | Piper MIT + voice card* |
| `voice_en_here_tts` | voice | en | I'm here! I'm here! Over here! | 2.2 | Piper en_US-lessac-low | Piper MIT + voice card* |
| `voice_zh_call_tts` | voice | zh-TW | 救命啊！有沒有人聽得到？我的腳被壓住了！ (Help! Can anyone hear me? My leg is pinned!) | 3.4 | Piper zh_CN-huayan-x_low | Piper MIT + voice card* |
| `voice_zh_here_tts` | voice | zh-TW | 我在這裡！我在這裡啦！ (I'm here! I'm right here!) | 1.8 | Piper zh_CN-huayan-x_low | Piper MIT + voice card* |
| `voice_id_call_tts` | voice | id | Tolong! Ada yang bisa dengar saya? Kaki saya terjepit! | 3.3 | Piper id_ID-news_tts-medium | Piper MIT + voice card* |
| `voice_id_here_tts` | voice | id | Saya di sini! Di sini! | 1.9 | Piper id_ID-news_tts-medium | Piper MIT + voice card* |
| `voice_vi_call_tts` | voice | vi | Cứu tôi với! Có ai nghe thấy tôi không? Chân tôi bị kẹt! | 4.6 | Piper vi_VN-vivos-x_low | Piper MIT + voice card* |
| `voice_vi_here_tts` | voice | vi | Tôi ở đây! Ở đây! | 1.4 | Piper vi_VN-vivos-x_low | Piper MIT + voice card* |
| `weak_en_tts` | weak_voice | en | hello... is anyone there... water... | 7.9 | Piper en_US-lessac-low, **processed** | Piper MIT + voice card* |
| `weak_zh_tts` | weak_voice | zh-TW | 有人嗎……我在這……水…… (Anyone there... I'm here... water...) | 6.7 | Piper zh_CN-huayan-x_low, **processed** | Piper MIT + voice card* |
| `weak_id_tts` | weak_voice | id | halo... ada orang... air... | 7.9 | Piper id_ID-news_tts-medium, **processed** | Piper MIT + voice card* |
| `weak_vi_tts` | weak_voice | vi | có ai không... nước... cho tôi nước... | 8.3 | Piper vi_VN-vivos-x_low, **processed** | Piper MIT + voice card* |
| `weak_en_real_processed` | weak_voice | en | help me... help me... | 5.5 | Frances set: "help me" by Gemesil, **processed** | CC0 |
| `knock_concrete_3x` | knock | - | 3 groups of 3 knocks | 7.0 | synthesis: Kirchhoff plate modes of a 3.0 × 2.4 × 0.15 m slab, plus a contact thud | CC0 (generated) |
| `knock_concrete_3x_once` | knock | - | one group of 3 (the reply to the call-out) | 1.8 | synthesis (same model) | CC0 (generated) |
| `knock_pipe_3x` | knock | - | 3 groups of 3 knocks | 7.0 | synthesis: free-free beam modes of a 1.5 m DN50 steel pipe, half-sine contact of 0.6 ms | CC0 (generated) |
| `knock_pipe_3x_once` | knock | - | one group of 3 | 1.8 | synthesis (same model) | CC0 (generated) |
| `knock_door_esc50` | knock | - | wood door knock (variety) | 3.0 | ESC-50 1-101336-A-30 (Freesound 101336) | CC BY-NC 3.0 |
| `cough_esc50` | cough | - | | 3.7 | ESC-50 2-108017-A-24 (Freesound 108017) | CC BY-NC 3.0 |
| `cry_esc50` | cry | - | infant cry (the only cry source on disk; not used in any profile) | 3.9 | ESC-50 3-151080-A-20 (Freesound 151080) | CC BY-NC 3.0 |
| `dog_bark` | dog | - | | 4.3 | Frances set: "Dogs Barking" by darranambler | CC0 |
| `generator_60hz` | generator | - | 1800 rpm, 60 Hz mains (Taiwan), seamless loop | 12.0 | synthesis | CC0 (generated) |
| `generator_50hz` | generator | - | 1500 rpm, 50 Hz (as `acoustic/scene.py`), loop | 12.0 | synthesis | CC0 (generated) |
| `settling_creak_trickle` | settling | - | creak, crack + thump, gravel trickle, rumble | 10.0 | synthesis | CC0 (generated) |
| `wind_real` | wind | - | loop-crossfaded | 12.0 | Frances set: "Wind 2.mp3" by FunWithSound | CC0 |
| `siren_distant` | siren | - | | 12.0 | Frances set: "Distant Ambulance Siren" by brunoboselli | CC0 |

**Attribution:**
- The "Frances set" is F. Valisalo's CC0 collection, `data/audio_real/frances/manifest.csv`. Each clip's Freesound URL is in the manifest's `attribution` and `origin_url`.
- ESC-50: K. J. Piczak, "ESC: Dataset for Environmental Sound Classification", ACM Multimedia 2015, CC BY-NC 3.0. Use it only in this non-commercial demo, and keep the credit in the video description.
- \*Piper's code is MIT, but each voice's licence comes from its training data. The model cards aren't on disk, so this is unverified.
  - From memory: lessac is Blizzard 2013 (research or non-commercial) and vivos is CC BY-NC-SA 4.0.
  - Check `rhasspy/piper-voices` MODEL_CARD files before any use beyond the hackathon.

**Honest labels** (each one is also in the manifest's `processing` and `notes`):
- **TTS shouts aren't real shouts.** Calm Piper speech was processed towards a raised voice: pitch up 10 % by resampling, a +5 dB presence boost at 2.5 kHz, and soft saturation. Real CC0 shouts are used wherever a phrase exists in the Frances set; all of those are English.
- **zh:** the Mandarin uses Taiwanese wording (有沒有人, 腳, 啦) in Traditional characters. The only Mandarin voice on disk is a Mainland female voice with **weak tones**: the model's phoneme map has no tone symbols, so Piper logs "Missing phoneme 1..5". Treat it as a placeholder until a native Taiwanese Mandarin speaker records it.
- **Texts:** the zh, id and vi texts were written for this demo and haven't had a native-speaker check, the same status as `sim/guidance.REVIEW`.
- **Weak voices are processed.** The steps are:
  - slow TTS, or the real CC0 "help me";
  - pauses of 1.5–2.4 s with faint inhales;
  - energy falling 6 dB across the clip;
  - pitch and tempo falling 6 %;
  - breathy noise at -14 dB;
  - a 4 kHz low-pass;
  - levelled at -32 dBFS.
- **TODO, not faked:**
  - **Taiwanese Hokkien** (`voice_nan_call`, `voice_nan_here`) needs a native recording. There's no open voice. The manifest's `todo` holds draft Han/Tâi-lô text, marked unverified.
  - **Groan/moan:** there's no source. ESC-50 has no groan class, and a YAMNet scan of the DroneAudioSet crying/humansounds programmes found no Groan or Wail/moan frame (max 0.26). It needs a CC0 recording or one allowed Freesound download.

### Physical models behind the synthesised clips

These are the standard formulas. The dimensions and damping values are assumptions, written in the code.
- **Pipe:** Euler-Bernoulli free-free beam, f_n = (β_nL)²/(2πL²)·√(EI/ρA). For a DN50 steel pipe (OD 60.3 mm, 3.9 mm wall) 1.5 m long, that gives modes at 160, 440, 863, 1427 and 2131 Hz.
  - Loss factor η = 0.04. That's a pipe clamped by rubble; a free pipe at 0.001–0.01 rings like a bell.
  - Each mode is split by 0.4 % so the pair beats. Excitation is a half-sine contact of 0.6 ms, plus contact noise.
- **Concrete:** a simply supported Kirchhoff plate, f_mn = (π/2)·√(D/ρh)·((m/a)² + (n/b)²) with D = Eh³/12(1-ν²). For a 3.0 × 2.4 × 0.15 m slab, f₁₁ = 70 Hz.
  - Loss factor η = 0.2, for a cracked chunk wedged in rubble. Intact concrete at 0.01–0.02 rings, and that doesn't read as a knock.
  - A band-limited contact thud dominates, as it does in real knocks on concrete.
- **Rhythm:** three knocks 0.30 s apart, then the next group about 2.6 s later. That's what "knock three times" asks for.
- **Generator:** a 4-cylinder 4-stroke at synchronous speed, so it fires at the mains frequency.
  - The harmonic series runs to about 2.4 kHz. There's a crank order at half the firing rate, combustion bursts with 30 % cycle-to-cycle variation, a component at twice mains (magnetostriction), and exhaust rumble.
  - Taiwan's grid is 60 Hz, so `generator_60hz` is the one used for the take.

### "Through rubble" model (ESTIMATE)

Loss A(f, d) = d·(3 + 6·f/kHz) dB, capped at 70 dB, where d is the burial depth in m. It's a 1023-tap linear-phase FIR, followed by a small-void reverb: RT60 0.22 s, low-passed, and +8 dB direct-to-reverberant by energy.
- **Source:** the 3 dB/m broadband term is the value already used by `acoustic/scene.py` and `sim/callout.py` (`RUBBLE_DB_PER_M`).
- **Not measured:** the frequency slope is a heuristic for porous, granular rubble, where losses rise with frequency as in porous-ground propagation models such as Attenborough's. I found no measured rubble attenuation-versus-frequency data on disk.
- **Effect:** speech stays recognisable at 0.5 m, and is a dull murmur by 2 m. For a voice, gated RMS drops about 1.5, 5, 7.5 and 9.5 dB at 0.5, 1.0, 1.5 and 2.0 m; most of a voice's energy is below 500 Hz, where the loss is small.
- **Level:** variants are **not** renormalised, so the loss stays in the file. Isaac plays them at the same gain as the dry clip.

## Dummy profiles (manifest `profiles`, `hidden_dummies`, `ambient`)

Prim convention: `/World/Dummies/<dummy_id>`. A buried dummy plays the rubble variant that matches its `burial_depth_m`.

| dummy | pile | plays (idle) | after the call-out | depth |
|---|---|---|---|---|
| A3 | A3 | cycles zh call → real "Help! Help me!" → en call, every 9 ± 2 s | "我在這裡！" / "I'm here!", 1.5–3.5 s after, twice | 0.5 m acoustic path (the scenario's radar depth is 0.89 m: the voice escapes through a void, as `scene.py` uses 0.4 m) |
| B1 | B1 | weak en "hello... is anyone there... water..." / weak zh, every 25 ± 5 s | none (irregular_apneic: no voluntary reply, `sim/callout.RESPONSE_MODEL`) | 1.0 m |
| B2 | B2 | concrete 3×3 knocks every 20 ± 3 s | one 3-knock group 1–4 s after, repeated 3× every 4 s | 1.0 m |
| A2 | A2 | silent (unconscious, radar only) | none | 1.5 m |
| C2 | C2 | dog bout every 15 ± 5 s (and barks at the loudspeaker) | bark 0.5–2 s after | surface |
| C1 | C1 | `generator_60hz`, continuous | none | surface |
| B3 | B3 | settling every 30 ± 10 s | none | surface |
| **H1_C3_vi** (hidden) | C3 | Vietnamese "Cứu tôi với!…" / "Tôi ở đây!", every 8 s | "Tôi ở đây!" | 0.5 m |
| **H2_A1_id** (hidden, alternative) | A1 | Indonesian "Tolong!…" / "Saya di sini!", every 9 s | "Saya di sini!" | 1.0 m |
| ambient | none | `wind_real` non-spatial bed at gain 0.25; `siren_distant` beyond the site edge every 45 ± 10 s | none | none |

The hidden dummies sit in C3 and A1, the two seed-7 sectors with no sound and no lead. They start with `active = false`. Someone flips them on live during the take, which is the unscripted change.

**Schedule semantics** (`schedule` objects):
- `loop`: play the next playlist item every `period_s ± jitter_s`, starting at `start_s`.
- `continuous`: loop the file with no gap.
- `after_callout`: after the last segment of a loudspeaker call-out ends, wait a random `latency_s`, then play `repeats` times every `period_s`. During that window it overrides `idle`.
- `order: cycle` steps through the playlist.

## Isaac: attaching audio to dummy prims (for the Isaac builder to implement)

One `UsdMedia.SpatialAudio` prim per active clip, as a child of the dummy prim. The SpatialAudio attributes exist in `usd-core` (checked here: `filePath`, `auralMode`, `playbackMode`, `startTime`, `endTime`, `mediaOffset`, `gain`). Whether Kit's audio player (`omni.usd.audio`) honours them in the livestream isn't verified here; please confirm in `FROM_ISAAC.md`.

```python
import json
from pxr import Sdf, Usd, UsdGeom, UsdMedia

man = json.load(open("assets/audio/dummies/manifest.json", encoding="utf-8"))
clips = {c["id"]: c for c in man["clips"]}

def attach(stage, prof, xyz, active=True):
    root = UsdGeom.Xform.Define(stage, prof["prim"])
    root.AddTranslateOp().Set(xyz)                      # dummy position (from the scenario / your site)
    p = root.GetPrim()
    p.CreateAttribute("larpverse:active", Sdf.ValueTypeNames.Bool).Set(active)
    p.CreateAttribute("larpverse:schedule", Sdf.ValueTypeNames.String).Set(
        json.dumps({"idle": prof["idle"], "after_callout": prof["after_callout"]}))
    for i, cid in enumerate((prof["idle"] or {}).get("playlist", [])):
        a = UsdMedia.SpatialAudio.Define(stage, f"{prof['prim']}/Audio_{i}")
        a.CreateFilePathAttr().Set(Sdf.AssetPath(f"assets/audio/dummies/{clips[cid]['file']}"))
        a.CreateAuralModeAttr().Set(UsdMedia.Tokens.spatial)
        a.CreatePlaybackModeAttr().Set(UsdMedia.Tokens.onceFromStartToEnd)  # the scheduler retriggers it
        a.CreateGainAttr().Set(1.0 if active else 0.0)

for prof in man["profiles"]:
    attach(stage, prof, xyz_of(prof["pile"]), active=True)
for h in man["hidden_dummies"]:
    attach(stage, h, xyz_of(h["pile"]), active=False)   # flipped live during the take
```

- **Scheduling:** a small per-frame scheduler (a Kit update callback) reads `larpverse:schedule` and `larpverse:active`. To fire a clip it sets that child's `startTime`/`endTime` to now … now + `duration_s`, or calls the audio player's play.
  - For a `continuous` schedule (the generator), use `playbackMode = loopFromStage` and skip the scheduler.
  - The call-out trigger is the loudspeaker broadcast end time from `sim/callout.plan_callout`.
- **Isaac as the source of truth:** the acoustic sensor reads, per dummy, the prim's world position, `larpverse:active`, and the clip currently playing. It feeds them to the scene synthesis exactly as `scripts/build_dummy_audio.py check` does: the clip resampled to 16 kHz, gain set on the dry parent so the rubble loss stays, then pyroomacoustics into the 6-mic arrays. See Request 1.
- **Gain:** the files are loudness-matched. `suggested_spl_db_1m` gives the relative level, e.g. a knock at 88 dB SPL is 2 dB below a shout at 90. In Isaac, set the speaker gain to 10^((SPL - 90)/20) relative to the voice. Weak voices are already 12 dB down in the file, so leave their gain at 1.

## Video soundtrack: "what drone d2 hears"

`check` writes a reference mix in `assets/audio/dummies/check/` at 16 kHz, the pipeline's rate:
- `d2_raw_16k.wav`: mic 0 of drone d2, all sources plus wind and sensor noise.
- `beam_A3_d2_16k.wav`: the LCMV beam that `acoustic/dsp.extract_clip` steers at A3, with nulls on every other source.
- `beam_B2_d3_16k.wav`: the knock beam.

For the take:
1. Bed: the d2 raw mic, ducked to -18 dB under the narration.
2. When the dashboard draws a yellow cone (human-audio clip, `scores.human >= 0.4`), cross-fade in that clip's beam signal over 0.5 s at about -6 dB. That's the separated signal saved by `acoustic.run` (`acoustic/data/clips/seed<seed>_<pile>.wav`, or the `audio` item handed to the Gemini stage). Hold it while the cone is shown.
3. Optional: a 200 ms "radio" band-pass (300–3400 Hz) on the beam, so viewers hear it as a drone channel rather than studio audio.
4. Caption the language and the transcript next to the beam audio. Captions come from the dashboard state, never from truth.

The beam is what the system really hears, and it's muffled and noisy. That's the honest version. Don't mix the dry dummy clip into the soundtrack as if it were the drone's audio. If the dry clip is used for the viewer's sake, label it "source audio (not the drone's microphone)".

## Pipeline check (`pipeline_check.json`)

`scripts/build_dummy_audio.py check` places the library clips at the seed-7 truth positions, so this harness is an observation generator, like `acoustic/scene.build_scene`.
- It uses the same pyroomacoustics free field, 1 m levels (`scene.KINDS` × the scenario's `level_db`) and wind/sensor noise as `build_scene`.
- It calls the **unchanged** `acoustic.run.analyse` with the deterministic agent, no LLM and no Gemini.
- The drones sit in a ring around B2, so every pile is within about 33 m. No acoustic code was changed: `acoustic.scene.Scene` is a public dataclass, so external clips need no code change.

Results on the committed WAVs (PASS rule: a human source gets human ≥ 0.4; a knock is class `knocking` with human ≥ 0.4; a distractor gets human < 0.4):

| listen | A3 voice (zh+en, 0.5 m) | B1 weak (1.0 m) | B2 knock (concrete, 1.0 m) | C2 dog | C1 generator | B3 settling | C3 hidden vi |
|---|---|---|---|---|---|---|---|
| baseline | PASS 0.99 voice | PASS 0.99 voice | PASS knocking 0.78 | PASS 0.02 dog | PASS 0.01 engine | PASS 0.08 debris | n/a |
| + hidden H1 at C3 | PASS 1.00 | not detected | FAIL: voice 0.47, *ambiguous* | PASS 0.01 | PASS 0.01 engine | FAIL: voice 0.45, *ambiguous* | PASS 0.99 voice, C3 goes from 8 to priority 56 (lead) |
| dog barking over the knocks | PASS 0.97 | PASS 0.90 | PASS knocking 0.60 | PASS 0.00 | FAIL: voice 0.45, *ambiguous* | PASS 0.07 | n/a |

**Reading it:**
- Clean listen: voices are flagged human, the knock is a knock, and the dog, generator and settling are dismissed.
- The hidden Vietnamese caller in an untasked sector is heard at once and ranked second (56, just under A3).
- With a seventh talker, or a dog over the knocks, the separation leaks a voice into a neighbouring beam. Every one of those false flags is `association: ambiguous`, so `run.py` caps its priority at 19, below dispatch. It never sends a drone, but it does draw a weak lead.
- **Run-to-run variance:** earlier builds with different TTS noise swapped which run leaked, and twice read the ringing **pipe** knock as `dog`. That's why B2 uses the concrete knock. Earlier builds also missed or kept B1 depending on the weak-voice processing (it's now lighter).
- **Transcripts:** Whisper `base.en` produces garbled text on the zh/en mix, as expected. The multilingual reading is the Gemini stage's job, which isn't run here.

## Requests

1. **Acoustic owner (`acoustic/scene.py`, `build_scene`):** add an optional `sources=[{xy, clip (16 kHz float array) or path, level_db, kind, depth_m}]` argument. Isaac's live dummy state could then drive `listen_and_rank` directly, with no harness copy of the room/ambient code. `scripts/build_dummy_audio.py check` (`check()`) shows the exact mapping. Today, external clips mean building `Scene` by hand. That works, but it duplicates about 30 lines of `build_scene`.
2. **Acoustic owner:** the cross-talk cases above (a voice leaking into B2/B3/C1 beams when a seventh source is active) are worth a look before the live take, since the unscripted hidden dummy creates exactly that situation.
3. **Orchestrator / user:** a native Taiwanese Hokkien recording (the 2 phrases in the manifest `todo`) and, ideally, native zh-TW, id and vi recordings to replace the TTS; one CC0 groan recording.
4. **Isaac builder:** confirm that `UsdMedia.SpatialAudio` plays in the livestream and can be retriggered from Python, and whether stream audio can be captured (`FROM_ISAAC.md`).

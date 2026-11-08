# From the Isaac Sim session (newest first)

### 2026-11-08 09:30: FINAL live take recorded (final_h), plus why takes miss A1

**Final take:** `take/larpverse_live_take_FINAL_60s.mp4` on branch `larpverse-rehearsals`. It is 60 s cut from `final/final_h.mp4` (5.0–65.0 s), one continuous take with no cuts inside, layout B, grid seed 7, reviewed every 2 s.

| Video (cut) | What happens |
|---|---|
| 0–19 s | Launch from the van, LiDAR map of A2, three radar pods down |
| 19–27 s | Live button (GO+18 s): "Live change (unscripted)", STAGE EVENT t130 H2_A1_id |
| 28–35 s | Pod d1 at A2 feels 8.6 breaths/min, trace on the panel |
| 37–43 s | "New voice heard near A1" (56/100, 2/2 agree). Pod-mic quick listen 3 at t295 |
| 44–51 s | "Plan changed": A2 cut short after 7 scans, dispatch to A1 at t440 |
| 52–60 s | A1 mapped (95 %), radar pods going down |

**Command** (box `larpverse-l40s-nb`, runner `/home/ubuntu/live2/nb_runY.sh`):
`export LARPVERSE_WHISPER_DEVICE=cuda; N=final_h PS=18 SS=80 YMODE=button LOOK="--grid-look 0 --drone-style blender --take-open wide --town isaac/assets/town_kit --live-asr multi --sky 600 --sun 2600 --fog-intensity 0.08 --fog-start 120" EXTRA="" bash nb_runY.sh`
on larpverse-isaac d45998a plus 3e1633c (sensor_fx).

**What we learned from eight takes (reh7, final_a–h):**
1. **`LARPVERSE_WHISPER_DEVICE=cuda` is mandatory.** `acoustic/models.py` defaults to cpu. On the CPU the pre-flight listen takes 65 s instead of 9 s, so no pod-mic listen finishes during A2 and the change is never heard (final_a). Check the log line `acoustic pre-flight PASS … warm listen X s`: X must be under about 15 s.
2. **Success is about 5 in 9, run to run.** With Whisper on CUDA:
   - **Went to A1:** reh6, reh7, b, c, e, h.
   - **Went to B1:** reh6b, d, f, g.

   B1's own victim is often audible first. In g, the A2 radar fixed early (dispatch t263) before any listen heard A1. In f, B1 was flagged NEW and the banner said "next pile → B1 (… unscripted live input)", which is wrong: B1 was not the live input. Worth fixing on your side: only attribute a re-plan to the live input when the NEW lead is within reach of the changed dummy.
3. **In every success the NEW flag fires on B1/B2, not A1.** A1 is first heard in the same listen (priority 56–57, never heard before), so it ranks first and the gate goes there. The panel's "voice near A1 that they had not heard before" is true. The log's "new human-like call at B1" is the trigger.
4. **`--cam-grade 1` is dropped.** reh7 came out sepia and washed out.
5. **3e1633c sensor_fx:** in simple style, only LiDAR rays that land on rubble (heightmap > 0.3 m) are drawn. Rays cast in transit streaked down the low opening camera like rain (final_b 9.5–11 s).

**Product video:** `showcase/` on `claude/project-thread-exloke`. Its collapse-site clips (NCKU scan textures) are replaced with grid-town footage from the same take, and its live-change scene now shows the hidden voice and re-plan instead of "operator adds a zone".

### 2026-11-08 04:05: rehearsals 5–6b: the unscripted change now works on camera (larpverse-isaac cd1a4cf)

**Result.** In rehearsal 6 the whole story fits inside 60 s (`reh6_layoutB.mp4` on release `larpverse-rehearsal1`, reviewed every 2 s):

| Video | What happens |
|---|---|
| 24 s | The teammate presses the live button. STAGE EVENT t137 |
| 26–34 s | Panel: "Live change (unscripted)" |
| 34–41 s | Panel: "New voice heard near A1". The pod mics heard it at t265. Re-plan: "A1 (56) now ranks first" |
| 42–48 s | Panel: "Plan changed / Flying to A1". Dispatch at t439 |
| 50–57 s | The drones map A1 and lower their pods |

The town kit is on for this run (streets, cars, damaged buildings). Its log says it is not the cause of the lag. "Behind" grows by about 0.8 s per rescan with or without the town, because each scan's DSP runs at ×24.

**What broke on the way, and the fixes.**
1. **The banner named the wrong pile.** reh5 said "PLAN CHANGED: A2 → B3" but flew to A1. reh5b said "A1 → B1", where A1 was only a guess by name order before any listen. Fixes:
   - the banner names the pile the gate ranks first (`gate_target`);
   - there is no "plan changed" claim when nothing was ranked before the input.
2. **The change was missed.** In reh5b the voice switched on at t265 and the pile ended before a pod-mic listen with it returned. Fixes:
   - after a live stage change the take pace is capped at ×10 until a pod-mic listen submitted after the change reports new evidence (at most 3 listens), or until the pile ends (`quick_pace`);
   - `QUICK_STRONG` is 50 (was 60);
   - a pile counts as new if it was silent from no further than max(d + 5 m, 1.6·d). In 6b, idle drones landed nearer A1 than the first pods, so A1 could never count as new;
   - `live_cmd.json` is stat'ed every 0.2 s (the button used to wait up to 1 s, which is 24 sim s at ×24);
   - the cap is applied after every state drain too. In reh6 it only averaged ×17.
3. **The gate listen held the clock 10.2 s.** A pod-mic listen was still queued in the worker. `listen_pace` now counts it, and uses 1.3× the pre-flight latency for the first gate listen.
4. **The panel showed a breathing rate on empty scans.** At reh6 75 s it said "Pod d1 feels a rhythm: 30.8/min" on scans the mission logged as "ranges no detection". A scan now counts only with ranges or breathing_hz. Otherwise the panel says "nothing strong enough to count in this scan" and draws no trace. (Please check: your radar vitals block is computed on empty scans too.)
5. **"Plan changed" was worded wrong.** At reh6 42 s it said "planned to search A2 next" while the drones were searching A2. It now says "cut the search at A2 short and now fly to A1".
6. **ASR (`--live-asr multi`) let video outros through.** The 10 s quick listens produced "Terima kasih kerana menonton!" ("thank you for watching", Malay) on B3. Fixes:
   - Malay/Indonesian, Vietnamese and Chinese outros are now filtered;
   - when the translation is dropped, the words are dropped too.
   A1's Indonesian words still don't survive the filters. `LARPVERSE_ASR_DEBUG=<file.jsonl>` dumps every raw decode so they can be tuned.
7. **The LiDAR read as rain** (10–15 s, 46 s). Simple style now draws a short downward fan: 8 rays per drone, each at most 18 m.
8. **Stale state at the start.** A left-over live.json made the panel open with a stale "Live change" that ate the real one's 8 s. The runner now clears it. The panel also re-arms on a new event and ignores events stamped after the mission clock.

**Still open.**
- The margin is thin. 6b pressed about 1 s later than reh6 (event t178), heard nothing new before A2 converged at t366, and the full listen went to B1 (83, boosted by its call-out answer; A1 57). A1's own call-out answers read as "already making such sounds before the call-out", because its idle loop is continuous. Rehearsal 7 / 7b test the fixes above with the button at GO+18 s and GO+24 s.
- Optional `--cam-grade 1`: contrast and saturation ×1.12, a faint warm gain, vignette and grain. Testing in reh7.

**Take command (current).** ISAAC_BRIEF §4 plus:
`--live-pace clear --comms 0 --hud off --overlay-style simple --live-asr multi --grid-look 0 --drone-style blender --take-open wide --town isaac/assets/town_kit --sky 600 --sun 2600 --fog-intensity 0.08 --fog-start 120`, env `LARPVERSE_WHISPER_DEVICE=cuda`, layout B (Isaac 1280×1080 + `isaac/hud/panel.html` 640×1080). Clear `dashboard/data/{live,hud,live_cmd,state,motion}.json` before the take. **Button: press once while A2's radar pods are down (the panel shows "Feeling for breathing"), about 18–22 s into the video.**

### 2026-11-08 03:45: rehearsals 1–4b, glitch list, fixes (larpverse-isaac 17df038, your 12577d6 merged)

**Rehearsal footage.** Release `larpverse-rehearsal1`. (Code state: larpverse-isaac e1eaf4d.) The most representative is `reh4b_layoutB.mp4` (81 s, look (a) + Blender drone, layout B), which I reviewed frame by frame every 2 s. Rehearsal 5 with the fixes below is running now; its MP4 lands in the same release.

**What each rehearsal showed** (grid, seed 7, real acoustic pipeline, `--live-pace clear --comms 0 --hud off --overlay-style simple`):

| Run | Live change | Result |
|---|---|---|
| reh1–2 | Y key through the WebRTC viewer | Never reached Isaac (keys don't reach carb). reh2: no A2 ping by t349 |
| reh3 | CLI activate at t242 | A2 needed 9 radar scans; ping only at t589 |
| reh4a | CLI activate at t182 (rescans) | Listen t497: A1 49 vs B1 47, so the fleet went to A1, but only by 2 points, and the panel showed base.en's "Oh long, other young bees are better…" |
| reh4b | **New live button** at video 20 s, event t121 | Button works. Listen t541: A1 56 (voice, human 0.98, **no transcript**) vs A3 **63** (answered the call-out, AVPU V), so the fleet went to **A3**. The plan didn't change on screen. Listen phase = 35 s of video |

**Glitch list (reh4b frames) and status.**
1. **Opening:** 8 s on a still van, with the clock at 0:01 for 6 s. Fixed: pre-roll 2.5 s with `--live-pace clear`.
2. **Ping beacon:** a 14 m glowing column 0.3–0.5 m wide dominated every listen shot. Fixed in `--overlay-style simple`: 0.06 m radius, glow 2, 7 m tall.
3. **Listen too long:** the pace was capped at ×4 for the whole 177 sim s (44 s of screen time), so the dispatch fell outside the take. Fixed: `LiveController.listen_pace` runs the listen at ×17 and slows only while the prefetched listen is still computing. That's about 15 s of video for a 15 s pipeline.
4. **A1's words never came through.** The GPU diagnostic gave Whisper's own language id as zh / th / hu / km at p 0.16–0.79 on the eSpeak Indonesian. Translation with the auto-detected language hallucinated every time ("Thank you for watching"). Forcing `id` read the separated A1 clip as "Ada yang bisa dengar saya, kaki saya terjepit" (lp −0.76, but no_speech 0.76, so your keep rule dropped it). Fixed behind `--live-asr multi`:
   - each voice clip is decoded once per language a Taiwan rescue team plans for (`LARPVERSE_ASR_LANGS`, default zh, en, id, vi);
   - Whisper small, beam 5, temperature fallback, no_repeat_ngram_size 3;
   - a segment is dropped at compression ratio ≥ 2.0, avg_logprob < −0.9, repetition loops, or stock subtitle/YouTube phrases. Without these filters 'vi' won on A1 with a YouTube outro at lp −0.39, and A3 decoded to "Kill me…" (e1eaf4d);
   - the no-speech gate applies only below VAD 0.8;
   - the best surviving language must beat the runner-up by 0.10, otherwise no words are shown;
   - Whisper then translates with the source language forced, under the same filters;
   - measured on the separated clips: A1 → id "Ada yang bisa dengar saya, kaki saya terjepit.", B1 → en, A3 → no transcript.
   - The English text becomes the clip transcript (the judge's keywords are English). The language and the original words go to `live.json` `heard`, and the panel shows "“Help! …” / spoken in Indonesian: “Tolong! …” · machine-translated".
   - Only the speech-to-text step of `recognise` is swapped, inside the Isaac worker. VAD, YAMNet, gates, scores and ranking are yours, unchanged.
   - This replaces your planned "voice-like call (language not identified)" for the take. If the decode isn't confident, nothing is shown, as before.
5. **Whisper on CUDA failed** (libcublas.so.12 not found). Fixed: `audio_env()` puts `.venv-audio`'s pip `nvidia/*/lib` on `LD_LIBRARY_PATH`. Measured: 10 s of audio in 1.7 s.
6. **Where a listen's 16.5 s go** (`--profile-listen`, L40S, before your 12577d6 latency work): separate 9.4 s, DoA 4.3 s, scene 1.9 s, Whisper 0.9–1.6 s. A 10 s window costs about 3.5 s, but it never heard A1, because its loop started 9 s in. Quick listens now squeeze the source stagger into their window.
7. **Quick listens never confirmed** (none in reh4b): two in a row outlast a 3-pod dwell. Now one clear listen confirms (priority ≥ 60 with ≥ 2 drones' bearings agreeing); weaker leads still need two.
8. **Not explained yet:** the rescans t125–364 took about 21 s of video, not the planned 10 at ×24. The take log now prints `phase X at t=.. wall=..s xSPEED behind=..s`; rehearsal 5 will show whether the mission thread is the bottleneck.
9. **Look:**
   - haze and low contrast (darkest 1 % of pixels ≈ 90/255);
   - the rubble reads as stacked flat slabs close up;
   - the flat desert has no context.
   Rehearsal 5 tries `--sky 600 --sun 2600 --fog-intensity 0.08 --fog-start 120`. `--town isaac/assets/town_kit` (1c897f0, off by default) adds streets, cracked asphalt, parked cars, poles and wires, and 2–3-storey damaged buildings with no colliders. The RC review: "a big step up in context; cars and buildings blocky up close". Stills are `town_*.jpg` on `larpverse-lookcheck`.
10. **Panel:** the body went near-empty for a moment at about 25 s (caption transition). 'Plan changed' now uses your banner: "Before the new voice, the drones planned to search A3 next. They heard it themselves and now go to A1 instead."

**Your paused live-agent WIP.** 17df038 merges your 12577d6 into larpverse-isaac. Conflicts were in `isaac/live.py` (submit, preflight, prefetch, `_handle_quick`), `isaac/run_mission.py` (runner.target) and `acoustic/models.py` (transcribe). Both sides are kept. Please merge larpverse-isaac before you commit the WIP on trigger / redirect timing / garbled transcript, since items 3, 4 and 7 above overlap with it. The live button is `isaac/hud/live_button.html` → `POST /api/live` in `scripts/serve.py` → `live_cmd.json`.

**Take command (proposed; final after rehearsal 5).** ISAAC_BRIEF §4 take command plus:
`--live-pace clear --comms 0 --hud off --overlay-style simple --live-asr multi --grid-look 0 --drone-style blender --take-open wide [--town isaac/assets/town_kit]`, env `LARPVERSE_WHISPER_DEVICE=cuda`. The teammate opens `/isaac/hud/live_button.html` and presses once while A2's pods are down.

### 2026-11-08 02:50: look check on the L40S, and fixes (9cc8c5e)

**Stills.** Variants (a)–(d) rendered on the Nebius L40S; the stills are on release `larpverse-lookcheck`:
- (a) the previous grid look;
- (b) grid-look + Blender drone + wide opening;
- (c) (b) at take-render quality;
- (d) (c) + `--cinema 1` with the kloofendal sky.

**Verdict.**
- **(a)** has the most natural light and the best contrast.
- **(b) and (c)** have a strong blue cast. The grid look switched the sun to white and set a blue-grey fog colour and gain, all under the blue evening sky.
- **(d)** is washed out and tan. The film grade lifted the blacks, the haze was warm, and the brighter sky with sun 1600 overexposed the frame.
- The `[cinema]` log says 31 settings applied. `raytracing/inscattering` is unknown to this Kit build.

**Fixes in 9cc8c5e.**
- `--grid-look 1` keeps photoreal's sky, sun strength and haze. The sun is slightly less orange (1.0, 0.86, 0.70), and the grade is mild (saturation 0.95, contrast 1.08). It still re-materials the rubble with the textured concrete and brick.
- `--cinema 1` is now lens and sensor only: shutter blur 0.2, chromatic aberration 0.004, flare, grain 0.018 and vignette 0.05. No grade, haze or sky changes.
- A quick variant (e) on 9cc8c5e follows the rehearsal. Note that at the survey distance the drones are only about 100 px wide; we'll judge that on the rehearsal footage.

**Rehearsal 1 is running.**
- Real acoustic pipeline: the preflight passed, and a warm listen takes 18.9 s.
- Settings: grid, seed 7, `--live-pace clear --comms 0 --hud off --overlay-style simple`, layout B with `panel.html`.
- Y is pressed during A2's rescans, so it tests the A3-vs-A1 question above with the real pipeline.

### 2026-11-08 02:35: merged 04a4d34. ⚠ A Y press after A2's first dwell doesn't redirect to A1 (stub acoustics)

**Merge.** larpverse-sensing 04a4d34 is merged into larpverse-isaac. The only conflict was a log string in `sim/grid_mission.py`; we took your version, so `sim/`, `tests/` and `contracts/` are identical to yours. `tests/test_live.py`: 11 passed. The rehearsal take (with the real acoustic pipeline if `--preflight` passes, otherwise `--allow-stub-acoustic`) starts now on this commit.

**⚠ The live change, measured on the merged code.** Dry rehearsals: seed 7, grid, `--acoustic stub --allow-stub-acoustic --comms 0 --live-pace clear`, with Y (`activate:hidden2`) at four times:

| Y at | Where in the story | What happens |
|---|---|---|
| t 100 | A2's first 3-pod dwell | Pod-mic quick listen hears A1 at t 125: priority 79, confirmed twice. The A2 dwell ends at t 130 with 1 of 3 ranges, so **no A2 ping**. Fleet to **A1** at t 130 |
| t 200 | A2 rescans | A2 ping (t 275, vitals t 364). Listen t 399–541: **A3 76, A1 71**. Fleet to **A3** |
| t 300 | A2 rescans | Same: A3 76, A1 71. Fleet to **A3** |
| t 420 | During the listen | Same. Fleet to **A3** |

- **After the first dwell the change has no visible effect.**
  - A3's scenario voice outranks the A1 dummy. A3: "Help! Can anyone hear me? My leg is trapped.", human 0.86, urgency 0.94.
  - At the listen, the dummy's stub clip is a partial transcript: "… Ada yang bisa dengar … Kaki saya terjepit!", urgency 0.88, priority 71. The same dummy scores 79 at the pod-mic quick listen.
  - A3 is also where the fleet goes with no trigger (SEED7_STORY: A2 → A3 → …). Judges would see no adaptation, which is the 25% criterion.
- **We haven't touched `sim/`. Options, your call:**
  - (a) make the hidden voice reliably clear at the listen positions, e.g. the dummy's depth or clip choice (a stage/scenario parameter, not planner logic);
  - (b) press Y in A2's first dwell: video 16.9–20.7 s at the clear pace. A1 is then reliable, but there's no A2 ping before the change;
  - (c) the operator draws a zone around A1 as the change.
- The real pipeline may score differently. The rehearsal reports what it did.

**`--live-pace clear` re-timed for SmoothFlight.** It is now `deploy=6,survey=7,recon_mapping=6,recon_listening=10,landing=6,radar=12,relocate=24,rescan=24,acoustic=17,dispatch=5,return=12,battery_swap=40,done=8`.

The old values gave 76 s with the new flight timings: the survey alone now takes t 20–65. The new values give 56.3 s. Times are measured on the dry-run frames:

| Video s | What |
|---|---|
| 0–4 | Van pre-roll |
| 7.2 | Survey of A2 starts |
| 16.8 | Pods down at A2 |
| 21–30.7 | Rescans at ×24 |
| 30.7–41.1 | Listen |
| 41.1 | Dispatch |
| 45.9 | Survey of the next pile |
| 54.5 | Pods down there |

Panel captions (each held ≥ 5 s): Launch 4.1, Mapping 9.2, Feeling for breathing 14.2, Breathing-like signal found 27.1, Live change 32.1, Listening 37.1, the re-plan 42.1, Searching the new area 51.4.

**Panel.**
- It says **"Plan changed"** only when the dispatch goes to the changed pile, or follows an operator zone. Otherwise it says "The swarm decides", so it never claims an adaptation that didn't happen.
- The decision rows add **"Next best option: A3 (priority 72)"**, the runner-up taken from `state.decisions`. That makes the change visible on screen.
- The steps bar no longer marks Listen as done when the change comes before the listen.

### 2026-11-08 01:35: plain-language perception panel, slower "clear" take pacing, Blender drone

**`isaac/hud/panel.html` (new): a right-hand perception column for OBS layout B, written for people with no robotics background.**
- One short caption per story step, each held for at least 5 s: Launch, Map, Radar, Listen, Live change, Re-plan, New area.
- Below it, a plain list of what the drones are sensing right now. Every number comes from the run's own files:
  - LiDAR points and coverage come from `hud.json`.
  - The radar breathing trace is `live.json` `last_scan.wave`. Its rate and range are `vitals.rate_bpm` and `rate_ci`; before vitals exist it says "first estimate".
  - Heard speech comes from `last_listen`. It is translated with `assets/audio/dummies/manifest.json` (`transcript_en`).
  - The depth line says "depth not measured yet" while `depth_source` is `nominal`.
- Pings are always "possible person (simulated)", never "found".
- URL: `http://localhost:8765/isaac/hud/panel.html?state=../../dashboard/data/state.json`. It scales to any width from 640 (OBS browser source 640×1080).
- We suggest it for the 640×1080 right column instead of `dashboard/?layout=take`, which is too dense to read in a one-minute take. Your call.

**`--live-pace clear` (new preset).** With `DEFAULT_PACE` the seed-7 take is 47.5 s. The Map step gets 3.5 s on screen and the dispatch to A1 gets 1 s, which is less than a caption needs. The x30 pod relocations also read as drones jumping, not flying.
- `clear` = `deploy=5,survey=4,recon_mapping=6,recon_listening=10,landing=4,radar=12,relocate=20,rescan=20,acoustic=14,dispatch=3,return=12,battery_swap=40,done=8`.
- That gives 53.7 s in total, still under a minute:
  - survey A2: 5.3 s;
  - the four relocate/rescan cycles: 12 s (x20);
  - the listen: 10.9 s, with Y at t 400 about 4.9 s in, which leaves ~6 s before the re-plan;
  - dispatch to A1: 2.0 s;
  - A1 survey: 4.5 s.
- Any explicit pace string still works as before.

**Grid look (`isaac/grid_look.py`, on by default with `--photoreal 1` on the grid; `--grid-look 0` = the old look).** Our older grid renders read as a toy: near-white MDL concrete "cards", saturated orange brick, an orange sun and haze. Now:
- The irregular rubble pieces and the debris stones use the dusty baked textures of the Blender facade kit, with box-projected UVs and 8 tints: concrete about 0.29 linear albedo, broken concrete about 0.37, brick about (0.28, 0.20, 0.17).
- The sun is a dusty afternoon white (1.0, 0.91, 0.80) instead of (1.0, 0.80, 0.60).
- The grade is a real-camera look: saturation 0.85, contrast 1.05, neutral haze, AO, fine grain and a light vignette. Exposure is unchanged.
- Geometry, poses and colliders are unchanged, so the LiDAR and landing logic see exactly what they saw before.
- Not GPU-checked yet; stills of the old look vs this one come from the rehearsal box tonight.

**`--drone-style blender`:** a Blender-built heavy X8 (2.2 m wheelbase, about 126k triangles, PBR textures, `isaac/assets/drone/heavy_sar.usdc`). Spinning props use your `set_rpm` path. It carries your winch pod (Hang/Cable/Pod under its drum), so PodViz lowers, sways and rests it exactly as on your heavy model. It is off by default; your heavy model stays the default (note: `--photoreal 1` sets `--drone-style mesh` unless you pass another style).

### 2026-11-08 01:15: merged your full brief (b31fe88) into larpverse-isaac (c3c148b); dry-run findings

**Merge.** `larpverse-isaac` now has everything up to b31fe88 (live loop, director-take, HUD, van, dummies, heavy drone, PoseTrack motion, comms). Conflicts were only in `drone_viz.py` and `run_mission.py`:
- **run_mission:** resolved on *your* structure (PoseTrack, mission thread, `--live`, `director-take`, `hud.py`). Our extras sit on top and are off unless asked for:
  - `--camera story`;
  - `--overlay-style simple`;
  - `--line-scale` (renamed from `--viz-scale`; see below);
  - `isaac/sensor_fx.py` (LiDAR beams, radar pulses, sound domes, belief hotspot; `--fx 0` turns it off).
  - Our own Hermite smoothing (`--smooth`) is retired in favour of your PoseTrack.
- **drone_viz:** your heavy model is the default. `--drone-style mesh` (photoreal) keeps our smooth DJI-class body (`_build_parts_mesh`). Your integrated winch pods are used whenever the model carries one.

**Stream fixes from our on-box capture (live3, 1080p, 0 dropped frames), now in run_mission:**
- `--viz kit` crashed with "--viz-scale: invalid float value: 'kit'" (argparse prefix match). Renamed our flag to `--line-scale`, so your `--viz kit` works again.
- On the Lab route the stream was the whole Kit editor at 1440×900, with 739 frames dropped ("resolution differs from 1920x1080"). New `--stream-res 1920x1080` (default) adds `--/app/window/hideUi=true --/app/window/width=1920 --/app/window/height=1080` to `--kit_args` whenever `--livestream` is on, which gives a full-frame viewport.
- The Lab route starts stepping as soon as Kit is up, about 27 s before the stream connects. New `--hold-file PATH` renders the opening frame and holds t at 0 (before your pre-roll) until the file exists. Arm OBS, then `touch` the file.
- `--stop-t T` freezes the picture at sim t (ends dry and recorded runs).

**Your dry rehearsal (§6.2), identical on larpverse-sensing b31fe88 and on our merge:**
- It does **not** redirect to A1. At t 343 a portal phone report ("I'm trapped near B2", RSSI fit ±3.43 m) queues a re-plan to B2. The Y-stand-in at t 400 is then heard, but dispatch at t 485 is A2 → B2. Your brief expects A1 at t 471. It's the comms integration (aca080c, on by default). With the new `run_mission --comms 0` passthrough, the same rehearsal dispatches **A2 → A1 at t 485**. Should the take run with `--comms 0`, or is B2 intended?
- `tests/test_live.py::test_live_dryrun_with_trigger` fails on b31fe88 (`assert ([])`), likely the same cause.
- `tests/test_isaac_assets.py::test_heavy_parts_follow_the_bom` fails because `data/hardware_bom.json` isn't in the tree. `docs/HARDWARE.md` is missing too.
- Every run logs `isaac.motion failed (KeyError: 'no flight-motion trajectory for d1'); falling back to Hermite interpolation`. Is that expected until SmoothFlight lands?

**P1 (instanceable debris):** noted. We'll measure the survey-shot fps on the GPU first (the rehearsal running now) and do it if the fps is under 30.

**GPU rehearsal of the brief (self-test, stream/HUD/audio, Y take, collapse fps):** running now on the L40S from a clean checkout of b31fe88. Results follow in the next entry.

### 2026-11-08 00:40: rehearsal results (take1, live1) and the fixes now on larpverse-isaac (753999a, d01dcc6)

**Stream and capture (your Q1):**
- `--livestream 2` streams on the **Isaac Lab route** (`./isaaclab.sh -p run_mission.py ...`; Kit WebRTC on :49100, `/viewer` via nginx).
- On the **Kit launcher route** (`python.sh ... --launcher kit`) `--livestream` is ignored, and the Lab route writes **no PNGs** with `--record`. So: Lab route for the live take, Kit route for recorded backups.
- serve.py plus a private `brev port-forward ... -p 8765:8765` tunnel: the judge HUD, the dashboard and `/api/tasking` all work on Rohan's PC. Nothing is public.
- OBS capture, stream fps and audio are not measured yet: the viewer needs Rohan's own Brev sign-in in a browser on his PC.

**take1 glitch list (recorded, seed 7, director camera, 1080p30, rec-speed 4, 0.45 s per frame for 2 cams):**
- The story ran end to end: T1a beyond radar reach at t 87, zone OP-T1b added at t 116, dispatch to T1b at t 144.
- Look: too bright (14–28% clipped at f/9); rubble white with a blue cast; the fallen facade read as a clean grid; standing blocks had blurry blue scan facades; motion blur too strong.
- Clarity: the camera framed the tower, so the pods, the new zone and the dispatch happened off-centre. Zone outlines and dispatch arcs were invisible at that distance, LiDAR beams faint, radar rings small, drones too small.
- `heat 0` is expected: no radar lead on this seed, so the belief hotspot stays empty.

**Fixed (dry-run tested, `tests/test_isaac_dryrun.py` 8/8):**
- **Smooth heavy-multirotor motion** (`--smooth 1`, default). Monotone cubic paths one step behind the newest state (no overshoot, so no dips into the rubble). Tilt from acceleration plus drag, so the drone leans in and flares when braking.
- **`--camera story`:** one continuous state-driven shot.
  - It opens wide on the approach from staging and closes in on the active zone.
  - It widens for the listening domes and frames the drones' zone suggestion and the operator's new zone.
  - It holds on the zones while the drones fly home to swap batteries, then frames the dispatch flight in.
  - Critically damped springs and a slow orbit. Your `director-take` can replace it later; same interface.
- **`--viz-scale 4`:** zone and dispatch lines, rings, LiDAR beams and sound domes 4× wider for the wide shot.
- **Look:**
  - f/12;
  - darker, warmer rubble;
  - irregular, dirty fallen-tower facade (broken slabs, torn openings, 3 tints);
  - warm tint on the scan facade photos;
  - motion blur 0.15.
- **Drone hardware** (from your hardware list):
  - OS1-128 LiDAR on a nose bracket pitched 45°;
  - winch drum, and no radar panel in pod mode;
  - compute box, mesh radio with 2 antennas, blue Wi-Fi antenna;
  - red and green nav lights and a blinking white strobe.
- **Judge HUD:**
  - DECISION row (latest dispatch and why);
  - breathing condition on pings (`vitals.condition_text`);
  - green dispatch in the legend;
  - the correct step when opened late (fixes "Step 4 of 6" at done).
  - `isaac/hud/burn.js` burns the HUD into a recorded take, for the backup.
- **take2** rehearsal with all of this is rendering now (`--camera story --viz-scale 4 --lidar-profile heavy`).

### 2026-11-08 00:05: answers to ISAAC_QUICK.md, and what already exists for the single take (please don't rebuild it)

**Already on larpverse-isaac and dry-run tested.** Please build on these rather than duplicating them:
- **Live unscripted change:** `run_mission --tasking auto --out $L/dashboard/data`. The dashboard's "Add as zone" re-tasks the swarm mid-take: 5–30 s after the click in seeds 1–8, and ~28 s on seed 7 (zone added at t 116, dispatch at t 144). `isaac/tools/add_zone.py` gives the same write for rehearsals.
- **Dummies and acoustics in 3D** (`isaac/sensor_fx.py`, `--fx 1 --show-truth`):
  - grey half-sunk manikins at `truth.victims`;
  - teal wireframe sound domes from each sounding one while the fleet listens (grey for non-human);
  - LiDAR beams;
  - radar pulses around each scanning pod;
  - the belief hotspot.
  Your `--live` truth-from-Isaac and SpatialAudio can attach to the same `/World/SensorFX/Dummies/<id>` prims.
- **Perception on screen:** `isaac/hud/judge.html`, an OBS browser source over the stream that reads the same state.json. It shows:
  - step and caption, clock with the speed-up shown;
  - LiDAR, radar (last scan or ping), voices heard with an English gloss, non-human sounds, the drones' zone suggestion, zone status;
  - a mini-map;
  - an "operator added a zone" banner, then a "swarm adapts" banner.
  Screens: media/isaac/hud/ (next push). An in-Isaac `isaac/hud.py` would duplicate this. If you still want one, the radar breathing trace is the missing piece: add `scans[-1]` phase samples to state and judge.html will plot them.
- **Runbook:** `docs/LIVE_TAKE.md` (box, serve.py, SSH or `brev port-forward` tunnel, OBS scene, seed 7 story timeline).

**Your questions:**
1. Livestream: `--livestream 2` works on the L40S, and the Brev viewer `<brev URL>/viewer` loads behind the Brev sign-in. OBS capture at 1080p30 and audio in the stream are being tested now. Results to follow, with the rehearsal take and its glitch list.
2. Frame time (debris on/off, photoreal on/off, RTX modes): measuring now. For reference, the photoreal recorded run has been ~55 s for a short look test. Note that real-time is now `RealTimePathTracing` (Kit's default, DLAA plus ray reconstruction with `--rt-quality 1`).
3. Glitch list: comes with the rehearsal.
4. The core was not built by us: it is phupha-yao's `3d_model` generator, from screenshots of the NCKU scan. The Sketchfab page shows no download and no licence. The public-showing variant is `--urban-terrain mdl --no-scan-textures`.
5. Merging `3d_model` into sensing is phupha-yao's and Rohan's call. Our Isaac path only needs `collapse_site/collapse_site.usda` (`--site-usd`).

**The final take is still held** until READY TO RE-RECORD.

### 2026-11-07 23:45: your `--site collapse` merged into larpverse-isaac, live zones on the Isaac stage, single-take effects

**Merge (3b86342):** larpverse-sensing is merged into larpverse-isaac. Conflicts were resolved like this:
- `--site collapse` is now **your** scenario (`sim/site_collapse.py`, native `GridMission(site=...)`).
- Our compact 8-sector scenario is renamed `--site collapse-core`. It is used by `isaac/eval_collapse.py` and the case study.
- The map is loaded once, through `collapse_site.add_to_stage`, the GPU-tested path. Your `scene.build(site_usd=...)` path is kept but is not used for `--site collapse`.
- `--no-scan-textures` is honoured on our path.
- We kept our ribbon widths (`LINE_W 0.03`) and the urban-map dressing in `photoreal.dress`.

**New run_mission flags:**
- `--tasking auto|PATH|default`, passed to `GridMission(tasking=...)`;
- `--acoustic auto|stub|real`;
- `--fx 1` (below).

Tests: 185 passed and 5 skipped. Two things for you:
1. **Crash (please apply on larpverse-sensing):** `GridMission._finish_pile` raised `TypeError: unsupported format string passed to NoneType.__format__` when a sector had 0 pod placements, because `cov['radar_reach_m']` is None. Repro: seed 7 with an operator zone on T2b. On larpverse-isaac we guarded that one log line and changed no decision logic. During a live take this crash would end the take.
2. `tests/test_site_weiguan.py::test_grid_default_unchanged` fails on origin/larpverse-sensing itself, because the grid public md5 changed.

**Live operator zone for the judges' take** (the single continuous take, with "operator adds a new search zone" as the unscripted change):
- Run with `run_mission --tasking auto --out dashboard/data`. The dashboard's "Add as zone" (`scripts/serve.py` POST `/api/tasking`) then re-tasks the drones mid-take.
- `isaac/tools/add_zone.py` does the same file write for rehearsals: `--init T1a` before launch, and `--at 116` adds the drones' first sound suggestion.
- CPU rehearsal, seed 7: T1a survey ends t 46, pods and radar end t 88 with no ping, listening ends t 114 with voices near T1b and CN, the zone is added at ~116, and the swarm is dispatched to T1b at ~144.
- The re-task lands 5–30 s after the click in every seed we tried (1–8). The sector must have pod placements, otherwise the crash above.

**Sensor effects (`isaac/sensor_fx.py`, `--fx 1`):** these are display only, have no colliders and are never read by the mission.
- Purple LiDAR beams from each surveying drone to its newest returns.
- Orange radar pulses over the rubble around each scanning pod, out to ~3 m.
- Teal wireframe sound domes from each sounding dummy while the fleet listens (grey for non-human sounds).
- The localiser belief drawn as a glowing hotspot.
- With `--show-truth`: grey half-sunk dummies where the scenario hid its invented people. They replace the white capsules and are labelled simulated (R2-016).

**Look:**
- AI-refined scan textures: SD x4 upscaler, colour-locked to the original (`--scan-ai`).
- Kit's default RealTimePathTracing, with DLAA and ray reconstruction (`--rt-quality 1`).
- Facade v2 matched to the 2016 photos: thick grey floor slabs and dark crushed storeys.
- Excavators, a crane and fire trucks.

**Licence (your question):** we did not build the core. It comes from branch `3d_model` (phupha-yao's generator, from screenshots of the NCKU photogrammetry model). The Sketchfab model page (NCKU Cyber City lab) shows no download and no licence, so we have not used the mesh itself. For a public showing we still need NCKU's permission or the procedural variant (`--urban-terrain mdl --no-scan-textures`).

**Re-record** is still held until READY TO RE-RECORD.

### 2026-11-07 22:05: collapse site path-traced cut, and a Weiguan 2016 case-study simulation

**Path-traced collapse windows (034d94b, release `larpverse-collapse-pt`):** all 7 cameras, 1920×1080, 16 spp, `--replay` of the seed 9 CPU story (`python -m isaac.collapse_site --seed 9`), one process on the L40S, ~0.5 s per camera-frame (about 15 min for ~1,650 camera-frames). Review:
- **Read well:** `director`, `wide`, `map` (excellent ending shot, whole site north-up), `chase:d1` during deploy (operator and drones lifting off) and when landed on a slab, `mid` when the ping beacon and rings are up.
- **Weak on this map:** `fpv:d2` and `top` close-ups. The scan-photo texture is blobby within ~20 m, and the first-person view passes close to wreck faces. The cut uses director/chase there instead.
- Our 58 s cut opens with a case-study card (reported 2016 figures with sources, and a line saying the site is a reconstruction and the people, sounds, radar and drone data are simulated, per R2-016), then runs the mission with burned-in sector IDs, ping tags and a mission clock that only moves forward (no repeated action across shots).

**Case-study simulation (new, CPU only):** `isaac/eval_collapse.py` (evaluator: reads truth only after each run) runs the unchanged `GridMission` on `--site collapse` over 30 seeds × {acoustic, nearest, raster} × {legacy, literature} radar, plus an independent-start variant. Results: `docs/EVAL_COLLAPSE.md`; write-up with the 2016 figures and sources: `docs/CASE_STUDY_WEIGUAN.md`; figure: `docs/img/case_weiguan_timeline.png` (`python -m isaac.case_timeline`).
- Every sector is pinged, flagged or marked not cleared within 9.5–12.5 min of launch, against 52–60 h to the last live rescue and a 7–7.5 day search in 2016.
- With literature-grade radar, radar pings 3 of 305 invented people on this map (tall wrecks). The sectors flagged by listening hold 66% of them (74% with optimistic radar), against 1–30% for radar-only orders.
- Caveats are in the doc: about 10 invented people per seed, not 155; 7.2 of 8 sectors are occupied, so flagging is easy; `--acoustic stub` is an upper bound.
- `RASTER` for this map is set inside the evaluator (serpentine Z8 Z2 Z4 Z6 Z5 Z3 Z1 Z7 from the staging street); `sim/` is untouched.

### 2026-11-07 21:10: brief follow-ups (11eb199), and a regression you should know about

**Regression fixed: path-traced lines were back to 0.5 m tubes.** Your merge took `photoreal.dress()` from 68036c8, which sets `LINE_W = 0.006` but leaves `overlay.LINE_MESH` off. In RTX and path tracing, BasisCurves ignore that width and render as ~0.5 m yellow tubes (pt1 contact sheet). `dress()` now turns `LINE_MESH` on again (6–12 cm flat ribbons, as verified in pt2 at 85f7109). Please keep that line when you merge.

**New cameras for the shot list (§5.4, review T4):** `--rec-cams … ,mid,map`
- `mid` (or `mid:A2` to stay on one pile): 36 m out, 25° down, 35 mm lens on the 20.955 mm aperture (33° horizontal), on the active pile, with a slow ±6° drift for parallax. It sits WSW of the pile, so it's cross-lit by the SW sun. Use it for landing and radar (shots 5–6).
- `map`: straight down at 80 m, north up, east right, lens fitted to the site bounds (grid: 9.7 mm, whole site in frame). Sites too big for a 7 mm lens at 80 m get a higher camera instead (collapse site: 205 m). This is the ending shot that mirrors the dashboard (shot 9).
- `wide` and `map` use `isaac.site.bounds(sc)`, so they fit the collapse site. The grid keeps the exact orbit we recorded in lv2/pt2.

**Burn-in helper:** `photoreal.project(row, P)` (numpy only) turns world points into pixels for one `cameras.jsonl` row. It uses square pixels, the horizontal aperture fitted to the width and the principal point at the centre, which is how Kit renders a USD camera. It returns `(uv, depth)`; depth ≤ 0 means behind the camera. `scripts/cut_video.py` can import it, or copy it, to place pile/zone IDs and ping tags.

**Checks (no GPU):** offline with usd-core, every rig's target projects to (960, 540) at 1920×1080; `mid` is 36.0 m out at 25.0° down; `map` has all four bound corners in frame, east to the right, north up. `run_mission.py --dry-run --max-steps 60` PASS, and `tests/test_isaac_dryrun.py` 6 passed.

**Re-record command when you post READY** (L40S real-time first, then a path-traced pass of the chosen windows):
`--seed 7 --photoreal 1 --overlay usd --show-truth --record OUT --rec-cams director,wide,mid,map,chase:d1,fpv:d2,top --rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-warp 4 --rec-stop-t 380 --frames --out OUT_dash`, gated on `timeline.csv` (A2 ping before t 110, at least one dispatch).

**Media:** the 60 s path-traced draft cut of pt2 (A2 insufficient evidence → A3 ping → dispatch to B2, dashboard inserts) is at `larpverse/media/isaac/pt2/larpverse_grid_pathtraced_60s.mp4`. Its A3 shots are hazy; the haze settings from 3adca99 address that for the re-record.

**Licence variant (§5.8), status:** not a texture swap. `buildtex.py` derives the whole library from the scan photo (roof crops, texsynth exemplars), and `terrain_lowres.py` composites the photo into the core terrain tiles. So a scan-free core means re-running the generator with the photo weight at zero, plus non-photo roof and facade textures. Proposal: `collapse_site_noscan.usda`, a small override layer on `collapse_site.usda` that re-points the core terrain tiles and `scan_roofs` to regenerated textures. Tell us if pp would rather do it in the generator.

### 2026-11-07 21:00: ISAAC_BRIEF.md pass (branch `larpverse-isaac`, rebased on `larpverse-sensing` 451feeb)

**Branch:** `larpverse-isaac` now starts from `larpverse-sensing`, because your tree already has every GPU-verified `isaac/` change: ribbons, Recorder, `--rec-windows`, `timeline.csv` and path tracing. On top of that it carries only the GPU media (lv2/pt2 timelines, dashboard state, contact sheets) plus the brief items below. Delivered as a merge commit (3adca99, parents: the old `larpverse-isaac` tip 277b520 and sensing 451feeb) whose tree equals the rebased result, so there was no force-push and no history was lost.

**Done from §5 (no GPU needed; dry run PASS, `--help` lists the flags):**
- **5, scene look:**
  - `--fog-intensity` 0.35 → **0.18**, `--fog-end` 450 → **700 m**, `--fog-start` stays 70 m (≥ the 60 m you asked for). The values are logged as `[photoreal] haze: …`.
  - `--fpv-tilt` default **40°** down (was ~19°).
  - Chase rig at **6 m behind, 3 m up** (`--chase-back`, `--chase-up`).
  - `--veg` now defaults to **auto**: off on `collapse` / `weiguan` / `mesh:` sites, on for the grid. `--veg 0/1` forces it.
  - Flow dust is untouched; your `--look pretty` decides.
- **6, identification overlays:** the Recorder writes **`cameras.jsonl`** next to `timeline.csv`. One line per saved frame and camera: `frame, cam, width, height, world` (USD row-major 4×4; camera looks down local −Z, +Y up), `focal_mm, h_aperture_mm, v_aperture_mm, clip`. That's enough to project pile/zone IDs and ping tags in `scripts/cut_video.py`. Failures only log once and never stop a recording.
- **Held:** no re-record until you post **READY TO RE-RECORD**. Both GPUs are off: the L40S is stopped and the RTX PRO 6000 box is deleted.

**Answers to §6:**
1. **Merge `3d_model`?** Yes. Merge only `collapse_site/` (generator, `collapse_site.usda`, `geo/*.usdc`, textures, previews) into `larpverse-sensing` at `<repo>/collapse_site/`. The branch is based on the old `main` (292551d), so don't take its root `README.md` / `.gitignore`. The demo should run from `larpverse-sensing`/`larpverse-isaac` with `--site collapse --site-usd <repo>/collapse_site/collapse_site.usda`.
2. **0.125 m heightfield as `.npz`?** Yes, regenerated from the `3d_model` generator (`prep.py` → `outer_layout.py` → `heightfield.py`) and committed as **`larpverse/media/isaac/collapse_heightfield_0p125m.npz`** (16.5 MB). It holds `H` float32 2561×2561 (terrain z in metres, −0.26…5.97; buildings and debris are separate USD prims and not in H), `res=0.125`, `extent=160`. Row 0 is y=+160 (north), column 0 is x=−160 (west). `col = (x+160)/0.125`, `row = (160−y)/0.125`, in the site frame (x east, y north).
3. **Original Weiguan blocks A–I?** Not labelled in the model. The scanned core in `collapse_site/generator/sitecfg.py` names `CORE_BUILDINGS` by position (NW_warehouse, N_site_office, N_damaged_hall, N_shed, NE_hut, E_tank, **Block6_remnant**, W_long_shed, S_warehouse, SE_red_roof, SE_white, SE_tower, SE_green_roof) plus the two fallen 16-storey towers (`SLABS`: W_tower, E_tower). Only `Block6_remnant` carries a block number. I wouldn't map A–I onto these without a source.
4. **Frame time on the L40S, collapse site + 3 drones + recording, debris on/off:** not measured yet. It needs `--site collapse` (🟡 on your side) and a GPU run. I'll measure it in the first collapse-site self-test after READY. From the forest and earthquake runs: real-time RTX with 3–5 cams at 1080p was ~2 s per recorded frame on the L40S, and path tracing at 16 spp was <1 s/frame per camera on an RTX PRO 6000.

**Licence (§5.8):** noted. For any public showing I'll render a procedural-texture variant of the scanned core (no scan-photo albedo).

## 2026-11-07 18:20 (branch `larpverse-isaac`)

**Setup:** Isaac Sim 6.0.1 on a Brev L40S (headless), driven from Rohan's desktop.

**Dry run:** PASS. `python isaac/run_mission.py --dry-run --max-steps 60`: three drones move, the LiDAR gets 42,160 points on A2, and `state.json` is written. `pytest tests`: 22 passed, 3 skipped.

**What we added.** Everything is opt-in, and runs without the new flags are unchanged.
- `isaac/photoreal.py` (new), `--photoreal 1`:
  - Each scenario box under `/World/Rubble/<pile>/<kind>_NNN` is swapped for an irregular concrete piece of the same size and pose. Pieces are slabs with bent rebar, blocks or rocks, built by `isaac/rubble.py` with a triangle collider. The box itself is set inactive, so the LiDAR and landing cells see the same geometry.
  - Visual-only debris per pile and a gravel patch under each pile.
  - NVIDIA MDL materials: concrete, brick, gravel, dirt, asphalt and rusted steel.
  - A dusk HDRI sky, a low sun, haze, dry shrubs and trees, and an operator figure at staging.
- Recording with `--record DIR` (PNG frames per camera, virtual clock), plus:
  - `--rec-cams director,chase:d1,fpv:d2,top`
  - `--rec-res 1920x1080`, `--rec-fps 30`
  - `--rec-speed` (sim seconds per video second)
  - `--rec-stop-t`
  - `--rtx realtime|pathtracing --spp N`
- `run_mission.py`: about 30 lines of hooks for the above. The flag is named `--photoreal`, not `--look`, so it won't clash with your coming `--look pretty`. Feel free to have `--look pretty` call `photoreal.dress()`.

**Not changed:** `sim/` decision logic, the `state.json` contract, and truth access. The ghosts are still only shown with `--show-truth`.

**Next from us:** GPU bring-up (checklist steps 3–9), then footage for the shot list. Large files will go to a GitHub Release.


### 2026-11-07 18:25: first GPU bring-up (Isaac Sim 6.0.1 + Isaac Lab 3.0, Brev 1x L40S 48 GB)

- **Version / GPU:** Isaac Sim `6.0.1-rc.7` (Brev Isaac launchable, Isaac Lab 3.0.0), NVIDIA L40S, driver 595. Run with
  `/workspace/isaaclab/_isaac_sim/python.sh isaac/run_mission.py ...` from `larpverse/` (OMNI_KIT_ACCEPT_EULA=YES). scipy installed.
- **Test command:** `--seed 7 --headless --radar dsp --show-truth --photoreal 1 --max-steps 40 --record out/lv_test --rec-cams director,wide,chase:d1,fpv:d2,top --rec-res 1280x720 --rec-every 15`. Exit 0.

| Checklist step | Result |
|---|---|
| Stage built (~600 pieces) | PASS: `stage built: 621 rubble pieces` |
| Photoreal dressing | PASS: HDRI sky + sun 24°; crushed-concrete ground (4,572 faces); 621 irregular rubble pieces (183 with rebar); 34,200 debris stones; 70 grass/shrub clumps + 12 trees; operator model |
| LiDAR points > 0 | PASS: 17,440 at step 20, 30,048 at step 40; heightmap rebuilt from 10,359 points; 16 landing patches at A2 |
| Drones moving | PASS: d1 62.9 m, d2 67.6 m, d3 63.0 m by t=40 (survey → landing) |
| PNGs for every camera | PASS: 13 each for director, wide, chase_d1, fpv_d2, top |

- **Contact sheet:** `media/preview/isaac_lv_test_contact_sheet.png` (frame 10 of each camera).
- **What looks off:** (1) light-blue speckles all over the ground, apparently the LiDAR hit points drawn as an overlay. They read as noise in the wide shots; consider hiding them in `director`/`wide`, or thinning them out. (2) A haze band in the upper-left of `fpv_d2`. (3) One `[Error] Failed to resolve extension dependencies` line at startup, harmless (the run carries on).
- **Full recording:** started 18:23 on the same L40S: `--rec-res 1920x1080 --rec-fps 30 --rec-speed 6 --rec-warp 4 --rec-stop-t 250 --frames --out out/lv1_dash`. Clips go to `media/isaac/` when done.


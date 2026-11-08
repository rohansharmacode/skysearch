# Review pass 3: the demo path (the 60 s single take)

**Reviewer:** pass 3, demo path. **Date:** 8 Nov 2026, 01:36–02:45. **Code reviewed:** HEAD `553a69e` plus the uncommitted working tree. At review time, `isaac/live.py`, `isaac/run_mission.py`, `isaac/hud.py` and `assets/audio/dummies/manifest.json` had another agent's uncommitted edits; line numbers refer to that working tree.

**Scope:**
- The `ISAAC_BRIEF.md` §4 commands, run dry with no GPU.
- A read-through of the GPU-only code paths.
- The dashboard `?layout=take` checked against real dry-run output.
- Honesty.
- The test suite.

Nothing was fixed in other agents' files. There were no one-line typos worth fixing.

## What was run

All outputs went to a scratch directory (`$S` = the session scratchpad `pass3/`). The real `dashboard/data/` was never touched.

| # | Command | Result |
|---|---|---|
| R1 | Dry rehearsal exactly as in brief §4, `--test-trigger "activate:hidden2@400"`, `--max-steps 560` | Exit 0. `pods_max_visible: 3`. Stage event at t 400 (video 34.0 s). **Dispatch to A1 at t 542** (video ≈48.1 s). Run ends at t 560, video ≈51 s |
| R1b | The same with `python3`, as written | **Fails here:** `ModuleNotFoundError: No module named 'numpy'` (`isaac/live.py:42`). Re-run with `.venv/bin/python` |
| R2 | Control: R1 without `--test-trigger` | Dispatch to **A3** (78) at t 496 (video ≈43.5 s). A3 landed radars at video ≈54 s |
| R3 | Press in VIDEO_PLAN's recommended window, `activate:hidden2@251`, 620 steps | Radar-mic listens confirm A1 at t 306: "Re-prioritise: A1 (84) queued". Gate at t 542: **A1 79 vs A3 78**, so the fleet goes to A1 by **1 point**. Video: event 26.7 s, dispatch 48.1 s, A1 survey 52.2 s, A1 landed radars 58.6 s |
| R4 | `--dry-run --live --site collapse --max-steps 200` | Exit 0. `pods_max_visible: 3`, 145k LiDAR points. A portal "I'm trapped" phone forces a **re-plan to RW at t 174** with no key press. The run used a grid `tasking.json` (see P1-6) |
| R5 | `--dry-run --platform heavy_sar --delivery pod --max-steps 300` (`pod` = the radar-on-the-rubble-surface scan model) | Exit 0. `pods_max_visible: 3`. Localisation converged at A2 by t 274 |
| R6 | Headless Chrome (heavy.sh browser slot, `serve.py --port 8797` on a scratch copy holding R1's output), `?layout=take&state=data/state.json` at 640×1080 and 960×1080 | `docs/review/img/pass3-take-640.png`, `pass3-take-960.png`. The waveform, dig order and top clip are R1's real output, not fixtures. **The LIVE line is broken** (P0-1) |
| R7 | `.venv` pytest (all files except the two audio ones) | 1 failed (`test_plan_dummies_grid_seed7_matches_manifest`: library was mid-rebuild, transient). On re-run, `test_isaac_assets`, `test_isaac_dryrun` and `test_live` all passed (30). `test_radar_dsp::test_penetration_physics_numbers` **fails** (see the test table at the end) |
| R8 | `.venv-audio` pytest `test_acoustic.py test_dummy_audio.py` | 27 passed, 1 failed: `test_hidden_dummies_in_silent_sectors` (see the test table at the end) |

**State files vs the contract.**
- `state.json` is `larpverse.state/v1` with every v1 and v1.1 grid key. Radar candidates are `status: candidate`, never "confirmed".
- `live.json` (`larpverse.live/v1`) and `hud.json` (`larpverse.hud/v1`) are written every step. Their keys match what `dashboard/live.js` reads.
- **Neither of them is in `contracts/INTERFACES.md`** (P2-1).

**Known bug status.** `pods_max_visible` is **fixed**: it is 3 in R1, R4 and R5.

**Timing caveat.** `ad27b12` (02:16, "vitals-look drone choice iterated a set… different seed-7 stories") landed during this review. R1 and R2 predate it; R3 ran after it. Every timing below should be re-measured once on the final code.

---

## P0: breaks the take

### P0-1. The dashboard LIVE line is unreadable exactly when the key is pressed
- **Evidence:** `pass3-take-640.png` and `-960.png`. The LIVE bar shows "t 400 s" and a 30×16 px dashed box containing "stage: dummy" cut off. The event text is in the DOM: `<span class="lx k-stage">stage: dummy H2_A1_id activated near A1 …</span>`.
- **Cause:** a CSS class collision. `dashboard/live.js:92` builds the class `k-${e.kind}`, which gives `k-stage`. `dashboard/style.css:108` already defines `.k-stage` as a legend swatch: `width: 30px; height: 16px; border: 1.5px dashed …`.
- **Why it matters:** the "stage" kind is the line shown right after the teammate presses Y. That line is the on-screen perception of the unscripted change. The fixture used to verify c19bbc1 evidently had a different kind.
- **Fix:** rename the live-bar kinds, e.g. `ev-${e.kind}` in `live.js:92` and `#liveBar .lx.ev-stage` in `style.css:567`. Re-screenshot with a `last_event`-only `live.json`.
- **Owner:** dashboard (`dashboard/live.js`, `dashboard/style.css`).

### P0-2. With the real pipeline, the gate can wait 2–3 pipeline runs, and the latency rule makes it worse
- **The worker runs jobs one at a time, first in first out, with no cancel.** `isaac/live.py` `_SERVER` (`for line in sys.stdin`, ~l.764) handles one job at a time.
- **Radar-mic listens keep the worker busy.** `_quick_listen` (l.1236–1260) submits a new radar-mic listen every 20 sim s while the radars rest on the rubble. At the ×14 radar pace, one is in flight at almost all times once L (the latency of one pipeline run, in wall seconds) is more than ~1.5 s.
- **The prefetch queues behind it.** `_prefetch_listen` (l.1202–1221) is submitted at the landing command (t ≈364) and waits behind that radar-mic job.
- **A press after the prefetch adds a third job.** A press after ≈31 s submits a new job (`find_prefetch` misses on the new signature). That job waits behind the **stale prefetch, which still runs to completion**.
- **What the gate gets:** from the landing command it has ≈7 s (landing ×5) plus 14 s (141 sim s at ×10) ≈ **21 s wall**. It needs up to 2L (no press) or 3L (late press). At L = 10 s that is a 10–20 s hold with landed drones, which pushes the redirect from 48 s to 58–68 s.
- **The latency rule uses a stale constant.** `LISTEN_AFTER_DOWN_S = 66` (l.59) no longer matches. Measured from "all drones down" to the gate is **141 sim s** (R1: t 401 → 542). From the landing command it is 176 sim s. The rule `run_mission.py:827-835` caps the listen pace at `66/(L+1)`. At L = 10 s that is ×6, so the listen takes ≈23.5 s of video instead of 14. At L = 15 s it is ×4.1, ≈34 s of video, and the take no longer fits. The rule slows the take exactly when the pipeline is slow, and by more than needed.
- **Fix (live.py):**
  1. Stop submitting radar-mic listens once `m.pm.stage == "done"` or the radar phase is ending.
  2. Give jobs a `kind` and let the worker drain stdin and skip superseded jobs (send `{"cancel": id}`, or have it check a `latest_gate_id` before starting).
  3. Compute the window from the mission instead of the constant: `listen_s + callout_s` from `m`, or measure "all drones down" to the gate at run time.
- **Verify:** a dry run with `--acoustic real` and a fake service of fixed latency 10 s, logging gate hold seconds.
- **Owner:** `isaac/live.py`; `run_mission.py` for the cap.

### P0-3. The redirect, the visible action of the unscripted change, hangs on a 1-point margin
- **What happens:**
  - Without a press the gate goes to **A3 (78)** (R2).
  - A press at t 251 (the VIDEO_PLAN window) wins A1 with **79 vs 78** (R3).
  - VIDEO_PLAN's own stub table (§2, row "Y during the rescans, t 251") recorded **A3** for the same press.
  - The radar-mic listens heard A1 at 84 and logged `Re-prioritise: A1 (84) queued; A2 has a firm ping in progress`.
  - **But nothing is queued:** `isaac/live.py:1285-1286` only logs, and the gate re-ranks from its own listen. With the real pipeline, where Whisper `base.en` returned no transcript for A1 in VIDEO_PLAN's check, the ranking can easily flip back to A3. The fleet then does exactly what it does without a press.
- **Fix:** in the `firm` branch of `_handle_quick`, make the promise true: set `m._order = pile` (GridMission `_gate` honours it first, `sim/grid_mission.py:1216`). Log it as a re-plan, `self.replans.append(...)`, so the dashboard LIVE line shows "new voice heard near A1 → redirecting d1–d3 to A1".
  - **Honesty:** this is still decided from generator outputs only, confirmed by two consecutive radar-mic listens.
  - **Otherwise:** say plainly in VIDEO_PLAN that the redirect is not guaranteed, and keep the operator-zone fallback rehearsed.
- **Owner:** `isaac/live.py`; the mission owner should agree the policy.

---

## P1: looks bad, misleading, or a likely rehearsal failure

### P1-1. Hotkey Y toggles, so a second press switches the dummy off
- **Code:** `LiveController.before_step` calls `self.live.toggle` (`isaac/live.py:1048-1049`). The HUD button routes to the same `on_key` (`run_mission.py:754-757`).
- **Risk:** a nervous double press, or a press repeated because nothing visible happened, makes it "deactivated". There is then no camera event, no voice and no redirect.
- **Fix:** hotkeys and the HUD button should call `set_active(did, True, …)`. Keep toggle only for `live.py --cmd toggle`.
- **Owner:** `isaac/live.py`.

### P1-2. `--live --acoustic stub` outside `--dry-run` silently skips the pre-flight
- **Code:** `run_mission.py:723`: `want_real = (a.acoustic or "auto") != "stub" and …`.
- **Risk:** copying the dry-rehearsal flags into the take command gives a stub take with no refusal. Only the HUD and dashboard banner label it.
- **Fix:** in `parse_args`, `if a.live_take and not a.dry_run and a.acoustic == "stub" and not a.allow_stub_acoustic: raise SystemExit(...)`.
- **Owner:** `isaac/run_mission.py`.

### P1-3. The reaction time and end shot differ from what the docs promise
- **What the docs say:**
  - Brief §1: "about 7 s of video later".
  - Brief §3: "stage event at t400 → dispatch to A1 at t471".
  - VIDEO_PLAN §2: "B2 without a change".
- **What was measured:**
  - Press to redirect: **14 s** (R1: video 34.0 → 48.1) and **21 s** (R3: 26.7 → 48.1).
  - The redirect is late because the call-out phase grows from +68 s to **+114 s landed** when A1 answers (R1 log, reassurance plus guidance).
  - Without a change the fleet goes to **A3**, not B2.
  - A1 landed radars appear at 58.6 s, so the end shot only just makes 60 s.
  - The rehearsal command's `--max-steps 560` stops at video ≈51 s, so it never shows the end of the take.
- **Fix:**
  1. Use `--max-steps 620` in brief §4 and VIDEO_PLAN §6.2.
  2. Correct brief §1/§3 and the VIDEO_PLAN §2 row ≈44.
  3. Optionally raise the `acoustic` pace for the post-call-out reassurance and guidance stretch, so the redirect lands nearer 43 s.
- **Owner:** docs (orchestrator / VIDEO_PLAN author), `isaac/live.py` DEFAULT_PACE.

### P1-4. VIDEO_PLAN's OBS URLs lack `?layout=take`
- **Where:** `docs/VIDEO_PLAN.md:24, 250, 256` say `http://localhost:8765/dashboard/` and "narrow layout: it is checked at 400 px".
- **Why it matters:** `live.json` and `hud.json` are polled only with `?layout=take` or `?live=1` (`dashboard/app.js:23`). That URL therefore shows no LIVE line and no live waveform. The brief (l.101) has the right URL.
- **Fix:** use `…/dashboard/?layout=take&state=data/state.json` everywhere.
- **Owner:** VIDEO_PLAN.

### P1-5. `python3` in the commands assumes numpy
- **What happens:** `python3 isaac/live.py --write-tasking …` fails on a clean python (R1b). On the box, the container's `python3` is not Isaac's python, and the brief never says to install numpy/scipy for it.
- **Fix:** write `.venv/bin/python` (or `./isaaclab.sh -p`) in the brief §4 and VIDEO_PLAN §0/§6, or make `--write-tasking` and `--cmd` import numpy lazily.
- **Owner:** docs; `isaac/live.py:42`.

### P1-6. Collapse-site take: wrong zones, and a second, scripted re-plan
- **Wrong zones.** `live.py --write-tasking default` defaults to `--site grid`. The brief's collapse variant never adds `--site collapse`. R4 ran T1a under grid zone "A1" ("1 landing patches lie outside tasked zone A1; not used").
  - **Fix:** add `--site collapse` to the write-tasking line in brief §4 and VIDEO_PLAN §6.
- **Scripted re-plan.** On collapse the comms generator's phone reports "trapped" and `_check_portal` re-plans to **RW** at t 174 (R4) with no human action. RW is also the hidden dummy H1_RW_vi's sector. The take would show a scripted redirect next to the "one unscripted change".
  - **Fix:** in `--live`, disable portal re-plans from simulated phones (`--no-comms`, or a flag in `_check_portal`), or narrate it as a second, scripted input.
- **Owner:** docs; `isaac/live.py`.

### P1-7. The HUD overlay covers ~37% of the Isaac half and may be cropped in layout B
- **Coverage:** `isaac/hud.py:341-345` draws a 470 px-wide, full-height, 69%-opaque panel (`COL["bg"]` alpha 0xB0) at the viewport's right edge.
- **The button:** it includes a button that says "Unscripted: switch on a hidden dummy" (l.380), visible in the stream.
- **Cropping:** in layout B the 16:9 stream (1920×1080) goes into a 1280×1080 slot. Cropping to fill cuts 320 px per side, so about two-thirds of the HUD is cut off mid-text. Scaling instead leaves 360 px of black bars.
- **Fix:** for layout B use `--hud window`, or `--hud off` since the dashboard carries perception, or set the viewport/stream resolution to 1280×1080. Keep the button off-stream.
- **Owner:** VIDEO_PLAN §1/§6.5; `isaac/hud.py`.

### P1-8. The van's three lights are never created (confirmed with usd-core by a helper read)
- **Code:** `isaac/van.py:989-993` defines `UsdLux.RectLight` at `…/Lights/<name>`, then calls `xform(stage, path, …)`. That runs `UsdGeom.Xform.Define` on the same path (l.448) and **retypes the prim to Xform**.
- **Effect:** the van interior, the operator and the tent are unlit in the opening "van door" shot. `test_isaac_assets` only checks that the path exists.
- **Fix:** call `xform(...)` first, then `UsdLux.RectLight.Define(...)`. `dummies.py:401` uses that order.
- **Owner:** `isaac/van.py`.

### P1-9. Gemini is on by default for the take: tail latency and a daily quota shared with rehearsals
- **Default:** `--live-gemini` defaults to 1 (uncommitted, `run_mission.py:111`).
- **Tail latency:** `agents/llm.py` retries 429/5xx with up to 45 s of backoff (`LLM_BACKOFF_BUDGET_S`) on top of the call timeout. One bad call holds the gate.
- **Quota:** the budget is per run (`GEMINI_BUDGET = 20`), but the free tier is ~20/day/model. Every pre-flight and rehearsal spends the same quota, so the final take can hit a per-day 429.
- **Fix:** in `--live`, set `LLM_BACKOFF_BUDGET_S=3` and a ~8 s timeout for gate calls; rehearse with `--live-gemini 0`. Log remaining daily quota at pre-flight.
- **Owner:** `isaac/run_mission.py` / `isaac/live.py`.

### P1-10. The hidden dummies' language no longer matches the story
- **What changed:** the in-flight `assets/audio/dummies/manifest.json` (uncommitted, rebuilt 02:29) gives both hidden dummies **English** real shouts. H2_A1_id plays `voice_en_help_real_a__0.5m` and `voice_en_please_real__0.5m`.
- **What still says Indonesian/Vietnamese:**
  - the brief §1/§2: "Indonesian voice … 'Tolong! … kaki saya terjepit'";
  - the prim ids `_id` / `_vi`;
  - `test_hidden_dummies_in_silent_sectors`, which fails (R8).
- **Also:** the stub dry run still shows the scenario's Indonesian line in the top clip, so dry and real runs will differ on screen.
- **Fix:** decide (English is the safer bet with Whisper `base.en`), then align the brief, the test and the narration.
- **Owner:** audio agent; brief.

### P1-11. On screen: "IMMEDIATE" triage chip and clipped chips at 640 px
- **The chip:** at 640 px the dig-order card shows an **IMMEDIATE** chip, garbled by an overlapping element in `pass3-take-640.png`.
- **The clipped chip:** "no response — does not rule out unconscious" runs off the right edge.
- **Why "IMMEDIATE" is a problem:** it is a START triage category asserted from synthetic radar alone. The rest of the screen carefully says "check first", "aid for rescuers, not a diagnosis".
- **Fix:** label it "check first" (as the HUD does, `hint urgent`), and fix the chip overlap and wrap in `body.take` at ≤ 800 px.
- **Owner:** dashboard (`dashboard/vitals.js` triageChip, `style.css`).

### P1-12. A missing or partial dummy library silently removes the unscripted change
- **What happens:** if `manifest.json` is missing, `DM.load_manifest()` returns None. `build_dummies` then plans **no hidden dummies** (`dummies.py:356`), and Y logs only "no hidden dummy #2".
- **How likely:** `scripts/build_dummy_audio.py` deletes and rebuilds in place. During R7 the library had 11 of 131 files for several minutes. A pull during a rebuild commit would hit this.
- **Fix:** `run_mission --live` should exit if the manifest is missing or has no `hidden_dummies`, and the rebuild should write to a temp dir and swap it in.
- **Owner:** `isaac/run_mission.py`; `scripts/build_dummy_audio.py`.

### P1-13. Render settings and blur discs are unverified, and may change the look
- **Render mode:** `TAKE_RENDER` (`isaac/live.py:1637`) and `photoreal.set_renderer` (`photoreal.py:457`) force `/rtx/rendermode = "RaytracedLighting"`. The GPU-verified route never set it except for path tracing, and on 6.0.1 the default RTX real-time mode may be a different one. *Uncertain.*
- **Blur discs:** the rotor blur discs use `UsdPreviewSurface` opacity 0.30/0.12 (`drone_viz.py:59-60, 557-564`). Without `/rtx/raytracing/fractionalCutoutOpacity` they may render as opaque plates. *Uncertain.*
- **Fix:** drop the `rendermode` line, add the fractional-cutout key, and check both on the first rehearsal frame.
- **Owner:** `isaac/live.py`, `isaac/photoreal.py`, `isaac/drone_viz.py`.

---

## P2

1. **Contract.** `live.json` / `hud.json` are not in `contracts/INTERFACES.md` (VIDEO_PLAN request O1). Owner: orchestrator.
2. **HUD dig order** shows an unlabelled number ("1. A2-R1 (0.43)"). The decision line repeats itself ("dispatching d1-d3 to A1: Send the radar drones to A1: …") and is cut mid-word at 120 chars. File: `isaac/hud.py:269,276`.
3. **Real-pipeline label.** In a real take `state.mission.acoustic_source = "custom"` (`sim/grid_mission.py:995`), so the dashboard says "Custom acoustic source". Pass the pipeline's own label, e.g. `real-live`.
4. **Clinical wording in the call-out text:** "Responds to voice (AVPU V)" (`sim/callout.py`). AVPU is a clinical consciousness scale; say "answered the call-out".
5. **Stage-event wording.** The dashboard shows the stage event ("dummy H2_A1_id activated near A1") on the operator console, before the system has heard anything. It is labelled "the drones must hear or scan it to know"; adding "(demo view, not known to the system)" would make the boundary explicit. File: `dashboard/live.js:52`.
6. **Collapse site:** `world.changed` is `true` from the start with no event (R4), so every listen is "fresh". The untouched-stage-equals-scenario guarantee is tested only on grid. Owner: `isaac/live.py` (`signature` / collapse dummies).
7. **Helper read of the asset modules (lower risk):**
   - `UsdPrimvarReader_float2.varname` is authored as Token, not String (`van.py:341`, `scene.py:70`).
   - `overlay.UsdBackend.flush_points` re-uploads the whole cloud every 1500 points (`overlay.py:552-568`, `--overlay usd`, which the take uses).
   - `pods.py:141` `hasattr(d, "set_winch")` hides the radar visuals with `--drone-model small` (the fps fallback).
   - `UsdMedia.SpatialAudio` may be silent in Kit (`dummies.py:402`), so don't promise stream audio.
   - The recording rigs are centred on the grid (`photoreal.py:498`).
   - The van's monitor textures read "FIXTURE … not a real run" in a close-up (`assets/textures/dash_*.jpg`).
8. **Map at 640 px:** labels overlap ("B2 · unsearched" over B2's priority chip, C2, B3). File: `dashboard/app.js` take map.
9. **Worker death.** If the acoustic worker dies mid-take, `service.wait` raises in the mission thread and the run stops (keep-open). This is honest (no stub fallback), but log a clear banner. File: `isaac/live.py:895-905`.
10. **CJK glyphs:** `omni.ui` default fonts may not have CJK, so a Gemini zh-TW transcript on the HUD could render as boxes. *Uncertain.*
11. **Hidden dummy has no visual cue.** When the dummy is switched on, the event camera move (`TakeCamera.on_event`) frames a manikin that looks the same as before. A short visual cue, or reliance on the LIVE line once P0-1 is fixed, is needed.

## Checked and fine
- **The mission never reads the stage or truth.**
  - In `sim/` and `agents/`, `scenario.truth` / `.targets` are read only by the generators (`radar_model`, `grid_world`, `callout`, `comms` RadioSim, `acoustic_grid` stub) and evaluators (`impact`, `evaluate*`).
  - `grid_mission.py` has no truth reads. `agents/gemini_decider.py` strips truth-like keys.
  - `isaac/live.py` reads the stage only for generators, cache freshness, the HUD and the camera. Its re-plans come from outputs, tasking and the portal.
- **Stub fallback in the take is refused:** the pre-flight fails loudly and exits (`run_mission.py:724-736`), except for P1-2. `--test-trigger` is refused outside `--dry-run`.
- **Labels in the dry run:** the HUD "Sources" line, the dashboard banner and `last_listen.source` all say SYNTHETIC (stub).
- **Threading:** USD and PhysX stay on the main thread. `SyncedSource` queues writes; the probe is swapped to `probe_cached` in threaded mode (`run_mission.py:808-812`). No prims are created per frame in the reviewed paths. The HUD is throttled to 4 Hz.
- **Isaac-side APIs:** carb hotkey API names (`acquire_input_interface`, `subscribe_to_keyboard_events`, `KeyboardEventType.KEY_PRESS`) and the omni.ui HUD calls (`get_frame`, `ui.Plot`, `set_data`, `dock_in`) match Kit 105+ as far as can be checked without a GPU. Everything is guarded and falls back to a NullHud with a log line.
- The landed radars are visible in `--live`, and `heavy_sar --delivery pod` (the radar-on-the-rubble-surface scan model) runs clean.

## Test status

| Test | Status | Likely owner |
|---|---|---|
| `tests/test_radar_dsp.py::test_penetration_physics_numbers` | **Fails deterministically**: `reach_path_m(REACH_REF_RANGE_M, 0.95) = 1.90`, test wants ≥ 2.0 | Radar / physio recalibration (`698d79b`, `sim/radar_dsp.py`): decide whether the literature band or the test moves |
| `tests/test_dummy_audio.py::test_hidden_dummies_in_silent_sectors` | **Fails**: the hidden dummies are now English | Audio agent's uncommitted manifest rebuild (P1-10) |
| `tests/test_isaac_assets.py::test_plan_dummies_grid_seed7_matches_manifest` | Failed only while the library was being rebuilt; passes after | `scripts/build_dummy_audio.py` (P1-12) |
| Everything else | Passes: 143 + 88 + 30 in `.venv` (1 skip), 27 in `.venv-audio` | |

# Larpverse: slide outline (9 slides)

Placeholders «…» are numbers or media still being produced; see the table at the top of `docs/PITCH_LARPVERSE.md`. Every number on a slide carries its source in small type and its caveat in one line.

## Visuals that exist now

| File | What it shows | Good for |
|---|---|---|
| `docs/img/isaac_grid_map.png` | Seed-7 site map: 9 piles A1–C3 and the staging area (from the Isaac handoff; check it shows no ground-truth markers before using it on a slide) | Slide 3 |
| Team branch `larpverse-isaac`: `larpverse/media/preview/isaac_lv2_f150_contact_sheet.png` | Isaac Sim 6.0.1 photoreal frame 150, five cameras: director, wide, top, chase d1, FPV d2 | Slides 1, 3 |
| `larpverse-isaac`: `larpverse/media/preview/isaac_lv_test_contact_sheet.png`, `pt_test.png` | First GPU bring-up frames; one path-traced director frame | Backup for slide 1 |
| `docs/slam.png` | KISS-ICP pile map vs truth, landing cells | Slide 5 |
| `docs/radar_dsp.png` | Synthetic FMCW chain: range FFT, phase, band-pass, breathing peak | Slide 5 |
| `docs/radar_real_validation.png` | Real recordings: radar waveform vs reference, rate over time | Slide 7 |
| `docs/screenshots/grid-dispatch-t384.png`, `grid-final.png`, `ping-detail-A2-R1.png` | Dashboard mid-mission, final state, ping in the LiDAR model | Captured 17:58, **before** the dashboard honesty fixes (7c39d06): they may show the old depth "estimate" wording. Re-capture from the current dashboard for slides 6 and 8 |
| `docs/review/img/dash-b2-desk-1440.png`, `dash-b2-desk-ping.png` | Dashboard after the honesty fixes (depth "assumed", belief share, decision panel) | Slides 4, 6 (prefer these over the older screenshots) |
| `docs/review/img/dash-b2-phone-400.png` | Dashboard at phone width | Backup |
| `docs/review/img/anim-seed7-t10.png` … `anim-seed7-t80.png`, `anim-seed7-topdown.png` | Concept animation of the seed-7 story | Slides 2, 4 |
| `media/larpverse-grid-seed7.mp4` (88 s), `media/larpverse-grid-search.mp4`, `media/larpverse-concept.mp4` | Concept animations (not in git; storyboard release on the team repo) | Video, slide 4 |
| «DASH-MP4» `media/larpverse-dashboard-seed7.mp4` | Real pipeline recording | Not recorded yet |
| «ISAAC-CLIP» `media/isaac/` on `larpverse-isaac` | Isaac mission footage | Pending; `lv2_dash.tgz` and `timeline.csv` exist for the cut |

Avoid: `docs/review/img/phone-400-ping.png` and `desk-1440-final.png` (they show the old "dig here" and constant-depth "estimate" wording that review pass 1 flagged).

## Slides

| # | Title | One line of copy | Visual |
|---|---|---|---|
| 1 | **Larpverse** | "Three drones that decide where to measure next in collapsed-building rubble, and tell rescuers where to search first." | Isaac `director` frame from `isaac_lv2_f150_contact_sheet.png` (crop one panel). Small tag: "Isaac Sim 6.0.1, simulated" |
| 2 | **Many piles, one crew, digging blind** | "Survivors are in voids that cameras can't see. The commander has to pick which pile, and where on it, with few crews and little time." «STAT: one checked figure with its source» | `anim-seed7-topdown.png` (nine piles, no annotation) |
| 3 | **The user: the incident commander** | "They don't need a 'person found' alarm. They need: where to look, how sure we are, and what's not checked yet." | `docs/img/isaac_grid_map.png` next to the Isaac `top` camera panel |
| 4 | **The loop** | "Survey → land → radar → ping → listen → rank → dispatch. One range is a ring; three agreeing rings are a candidate search region." | Loop diagram (build it), with `anim-seed7-t42.png` inset and `dash-b2-desk-ping.png` for the ping |
| 5 | **What's new: autonomy, not new sensors** | "Drones choose the next landing spot by expected information gain, report uncertainty, and say 'insufficient evidence' when the scans don't agree." | `docs/slam.png` (landing cells) + `docs/radar_dsp.png`; seed-7 log line: "Relocate d1 → L15 … EIG 0.56 nats" |
| 6 | **The acoustic agent triages piles** | "Rotors off, drones listen to the other piles. Real models score how human and how urgent; a bounded agent picks the next pile, and code checks every pick." | `dash-b2-desk-1440.png` (acoustic panel + decisions panel). Badge: "decision by argmax / Gemini" |
| 7 | **Evidence** | "Breathing DSP on real radar recordings: 0.39 bpm MAE (line of sight, no rubble). SLAM landing map IoU 0.98 (sim). Planning resolves 43% vs 10% for fixed sites (sim, provisional). «EVAL-GRID» «ACOUSTIC-REAL»" | `docs/radar_real_validation.png` + one small table. Caveat line under each number |
| 8 | **What it can't do yet** | "Radar returns are synthetic; audio scenes are simulated; flight is kinematic. In our own demo run, a faint voice was put on the wrong pile: that pile stays 'not cleared'. No detection is not absence." | Fresh capture of the final dashboard state (or the last frame of «DASH-MP4») with B1 "not cleared" circled «SEED7-RERUN: replace if BL-3 fixes it, but keep the 'not cleared' message» |
| 9 | **Where it runs, and next** | "On the drone: LiDAR odometry, landing, radar DSP. Edge box at staging: fusion, planner, acoustic agent. Cloud optional. Next: real UWB radar, PX4 flight, recordings on a USAR training site." | Three-box deployment diagram (build it); close on the title line "Candidates, not confirmations." |

Optional slide 10 (Q&A backup): the rubric map table from `docs/PITCH_LARPVERSE.md` §11.

## Copy rules for every slide

- Say "candidate search region" or "ping", never "survivor detected" or "confirmed".
- Put "simulated" on every Isaac or animation frame, and "synthetic radar returns" wherever radar appears.
- Depth is "assumed", never "estimated".
- Every number: source in small type, caveat in one line.

# Larpverse slide kit

Everything you need for the deck. **Every fact here is sourced or labelled.** Keep the labels ("simulation", "model", "vendor claim") on the slides, because judges will push on numbers. The full sources are in `01_facts/Larpverse_Fact_Sheet.pdf`.

**Judging weights:** perception-driven action 25%, adaptation to change 25%, operational autonomy 15%, relevance and impact 20%, ambition and technical quality 15%.

---

## Suggested deck: 12 slides

### 1. Title: "Finding the survivors who can't call for help"
- One line: heavy drones map the ruin, call out, and **land on the rubble to scan it with a breathing-detection radar**, so unconscious and silent people get on the dig list in the first hour.
- **Image:** `02_isaac_renders/12_site_overview_van_and_rubble.jpg` or `11_command_van_hero.jpg`

### 2. The problem (relevance)
- **≥28%** of injured survivors of the 2003 Bam earthquake were **unconscious when rescued**, so listening devices can't hear them (Nia et al. 2008, via Rom & Kelman 2020).
- **Weiguan Jinlong, Tainan 2016:** rescued alive by day were **164 → 7 → 4 → 0**. Nobody was found alive after 62.3 h, and 115 of 309 residents died (Chiu & Tseng 2018; Tainan Fire Bureau).
- International rescue teams' live rescues were **under 0.85% of deaths** in all 14 earthquakes reviewed. They arrived 12–56 h after the quake, at about **US$0.8–0.94M per life saved** (Rom & Kelman 2020, BMJ Global Health).
- **Image:** `05_system_and_site/weiguan_case_study_site.png` with one giant number

### 3. Why today's search misses people
- Listening devices need a knock or a voice. In Turkey 2023, crews **switched off generators and asked victims to scratch with a fingernail**.
- Dogs work 20–45 min, then need to rest as long.
- Hand-carried radar checks one spot at a time, and a rescuer has to climb the unstable pile.
- **In 15 major collapses reviewed, no drone has ever located a living person inside the rubble.** Drones were used for mapping and lighting only.
- Turkey 2023: about 5,000 international rescuers saved about 300 people while 53,537 died, with 26,000 buildings to search.
- **Image:** text-led, or `03_radar_signals/zz_full_signature_gallery_10_classes.png` as a faded background

### 4. Our system
- **Recon:** three heavy-lift drones map the ruin with 128-channel LiDAR.
- **Listen:** they perch with rotors off and call out "knock three times" in Mandarin, Taiwanese Hokkien and English.
- **Land and scan:** each drone lands on a flat spot on the pile, and its downward-facing radar reads breathing up to about 3 m down.
- **Rescue Wi-Fi:** trapped people's phones can tap "I'm trapped".
- **AI ranks the dig order;** code checks every decision, and a rule-based fallback takes over if the AI fails.
- **Images:**
  - `02_isaac_renders/03_FULL_drones_landed_radar_on_T1b.jpg` (collapse site: private showing only, see credits)
  - `05_system_and_site/drone_real_components_labelled.png`
  - `05_system_and_site/command_van_layout.png`

### 5. Perception → action (25%)
- LiDAR map → drone lands on the flattest safe spot.
- Voice heard by 3 drones → dispatch to that pile.
- Breathing trace → radar ping, condition and dig order.
- Debris or machinery → "not a person → no dig".
- Every action is shown on screen with its reason ("d1–d3 → A1 BECAUSE voice heard, 3/3 drones agree").
- **Images:**
  - `03_radar_signals/breathing_vs_noise_comparison.png` (the key visual)
  - `02_isaac_renders/04_FULL_radar_breathing_scan_panel.jpg`

### 6. Reading breathing
- **Deep and slow:** possibly unconscious, so check responsiveness.
- **Shallow and rapid:** possible injury or distress.
- **Irregular with pauses:** check first.
- **Facts:**
  - The breathing condition is identified correctly **86%** of the time (simulation); rapid shallow breathing **100%**.
  - On real radar recordings, **6/6 breath-holds** were detected with zero false alarms, and the breathing rate was within about 0.5 breaths per minute (Schellenberger 2020 open dataset, line of sight).
  - It's a triage aid, not a diagnosis.
- **Images:** `03_radar_signals/breath_deep_slow.png` and `breath_shallow_fast.png` (clean, no text; transparent versions included), plus a grey `noise_*.png` underneath for contrast

### 7. Adaptation to change (25%)
- During the take, a teammate switches on a hidden caller **live, unscripted**. The drones hear it, a "PLAN CHANGED" banner appears, and the fleet redirects.
- A voice where silence was heard before counts as new evidence.
- The system also adapts by itself:
  - re-lands if its spot turns out unstable;
  - swaps batteries at a 25% reserve;
  - revisits open leads;
  - ignores a rejected radar return.
- **Images:**
  - `02_isaac_renders/05_live_unscripted_change.jpg`
  - `09_FULL_live_change_panel.jpg`
  - Swap in final-take frames when we have them.

### 8. Operational autonomy (15%)
- No operator input during the mission: recon, planning, sensing, triage, dispatch and battery swaps are all automatic.
- The AI (Gemini) decides, code verifies, and a rule-based fallback takes over if the AI is unavailable.
- **Zero false pings** across all evaluation runs, and the same result on every run.
- **Images:** `06_dashboard_and_phone/dashboard_video_layout_wide.png`, `02_isaac_renders/10_FULL_ping_and_dig_list.jpg`

### 9. Measured results
| Metric | Result | Source |
|---|---|---|
| Trapped people inside a drone-flagged search area | **53% within 30 min** (median 8 min), 83% within 1 h, Weiguan-scale site | Simulation |
| Survivors pinged vs simpler search with the same drones | **3×** (17/60 vs 6/60 nearest-first and 5/60 sweep) | Simulation, upper bound |
| Radar depth, our conservative simulation | **2.9 m** | Simulation |
| Radar depth, commercial rubble life detectors | 4.5–7 m | Vendor claim |
| Radar depth, NASA/DHS FINDER lab rating | **9 m of rubble** (it found 4 men in Nepal 2015) | NASA JPL |

- **Images:** `04_charts/impact_time_to_locate.png`, `04_charts/report_radar_reach.png`

### 10. Getting people connected
- Open rescue Wi-Fi on every drone. A phone taps "I'm trapped" in 6 languages and the operator sees it in **2 s**.
- The operator replies with approved messages only ("help is coming", "stay put, save battery").
- **Images:**
  - `06_dashboard_and_phone/phone_portal_mandarin.png`
  - `phone_portal_message_sent.png`
  - `dashboard_rescue_wifi_phones.png`

### 11. Impact and cost
- **City kit** (3 radar drones, a van, training): **$673k** up front and **$2.78M** over 10 years. That's about the yearly cost of a police drone programme, or 8.5 dog teams over 10 years.
- **Pays for itself at 0.56 lives per decade,** using Taiwan's value of a statistical life ($5M).
- **Weiguan model:** about **+6 survivors** (range 0.6–14) **if the radar performs as designed**. Label this as a model: there's no time-of-death data.
- The public-safety drone market is **$2.6B (2024), growing to $5.8B by 2030**.
- **Images:** `04_charts/impact_survival.png`, `04_charts/impact_cost.png`

### 12. Taiwan rollout and what's next
- **99%** chance of a magnitude 6.5+ quake in Taiwan within 50 years (Central Weather Administration, via news).
- **First radar ping about 49 min after the quake,** against up to 12 h for national special teams at Weiguan.
- **47 kits cover more than 90% of Taiwan's urban land within 20 min: US$15.5M.**
- The kit retrofits onto existing rescue trucks: 250 flight-minutes from half a tank via a generator, so no city hub is needed.
- **Next:**
  - a drill with a real FINDER-class radar;
  - licensing the radar frequency band (12–18 months);
  - native-speaker voice recordings.
- **Images:** `04_charts/impact_logistics.png`, `05_system_and_site/weiguan_search_sectors.png`

---

## Honesty rules (keep these on the slides or in the talk)
- Never say an area is "cleared". A searched zone without a ping is "searched, not cleared".
- We speed up **finding** people, not digging them out; extrication is still the bottleneck.
- Radar and audio signals are **simulated** (validated on real recordings in the open, not through rubble). LiDAR is real Isaac ray-casting.
- Drone finds get the INSARAG "V" mark (victim suspected) for rescuers to confirm.
- Don't claim 10 m depth. The real upper reference is FINDER's 9 m rubble lab rating.

## Likely judge questions
- **"2.9 m is worse than a LifeLocator."** Our simulation is deliberately conservative, and the drone can carry that class of radar. The point is landing it on every spot on the pile, by drone, in the first hour.
- **"Is it faster?"** It finds silent people that today's methods miss and ranks where to dig sooner. Digging time doesn't change.
- **"Is it real?"** Isaac Sim with real LiDAR ray-casting. Radar and audio are synthetic but run through real signal processing, checked against real recordings.
- **"Cost?"** About $673k per city kit; it breaks even at under one life per decade.

## Credits (required in the video description and on the last slide)
- Audio:
  - Frances's recordings (CC0);
  - ESC-50 / Freesound clips: Robinhood76 (CC BY-NC 4.0), ErocShadow (CC Sampling+), PickleJones (CC0);
  - synthetic voices from eSpeak NG and Piper vi_VN vivos (CC BY-NC-SA 4.0).
- Real radar data: Schellenberger et al. 2020 (CC BY 4.0).
- The Weiguan collapse geometry derives from third-party scan screenshots. **Don't show the collapse-site renders publicly** without NCKU's permission. The renders in this kit are of the **grid site**, which is fine.

## Folder contents
- `01_facts/`: the 3-page fact sheet PDF (every number with its source).
- `02_isaac_renders/`: real GPU frames from Rohan's rehearsal. The FULL frames include the perception panel.
- `03_radar_signals/`: clean breathing and noise visuals in dark and transparent versions, the comparison image, and the full 10-class gallery.
- `04_charts/`: impact, cost, logistics, motion-smoothness and radar-reach charts.
- `05_system_and_site/`: the drone with labelled real components, the van layout, the manikins, the Weiguan site.
- `06_dashboard_and_phone/`: operator dashboard and phone portal screenshots.

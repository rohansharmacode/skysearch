# Deployment and logistics: getting Larpverse kits to a collapse in Taiwan

Researched 8 Nov 2026. This document extends [HARDWARE.md](HARDWARE.md) (parts, kit cost), [COMMS_ARCHITECTURE.md](COMMS_ARCHITECTURE.md)
(radio plan) and the half-tank analysis in [IMPACT_EVIDENCE.md §6](IMPACT_EVIDENCE.md). It doesn't repeat them.
Machine-readable twin: [`data/deployment_params.json`](../data/deployment_params.json) (same tags, low / central / high).

Tags: **[S]** sourced (the linked source states it), **[D]** derived (our arithmetic on sourced or sim numbers, shown here),
**[A]** assumption (team judgement, no direct source). **(weak)** marks news, vendor or secondary sources.
**[sim]** marks a number measured in our own simulator, not in the field.

A **kit** is 3 heavy drones (FlyCart 30 class, 95 kg MTOW) with belly-mounted radars, chargers, the mesh mast, the ops laptop
and spare batteries. It rides on an existing fire/rescue truck and is charged from that truck's generator or a towed genset.

## Judge summary

1. **Taiwan will get this earthquake.** The CWA puts the island-wide 50-year probability of an M≥6.5 quake at 99% and of an M≥7 at 54% [S] *(weak: news quoting the CWA)*.
   Tainan's 中洲構造 has the highest single-structure probability, 39% for M≥6.9 in 50 years [S] *(weak)*. Taipei's Shanchiao fault M6.6 scenario predicts 4,400 collapsed buildings and 4,100 dead or gravely injured [S] *(weak)*.
   About half of the homes in Tainan and Kaohsiung, and 70% in Taipei, are more than 30 years old [S] *(weak, search summary)*.
2. **A truck-mounted kit can give rescuers a first ping in under an hour.** Our central timeline runs from a 03:57 quake to the first acoustic lead at about **T+40 min** and the first radar breathing candidate at about **T+50 min** (range 23–94 min) [D].
   At Weiguan, the national special search teams took up to 12 h to arrive, and international teams usually take 12–56 h [S].
   The kit does not replace the dig. It tells the first crews where to dig, while survival is still about 85–90% [D, IMPACT_EVIDENCE §1b].
3. **Taiwan needs about 47 kits (central), 12 (low) or about 133 (high).** Low gives one kit to each of the 10 NAP-graded USAR teams plus the NFA team.
   Central puts a kit within 20 min of the urban area and gives one to every county. High puts a kit within 10 min of it.
   At about US$0.33 M per kit [D from HARDWARE], the central option costs about **US$15.5 M** of capital. That is the price of roughly **10 international USAR deployments** (US$1.6 M each, central), or of **17 lives at the ISAR cost per life saved** [D, IMPACT_EVIDENCE §5].
4. **A kit can run for 72 hours on one truck plus two fuel drops.** A kit draws about 22 kW at peak, uses about **1.2 MWh** and burns about **510 L of diesel** over 72 h (central; 225–1,410 L) [D].
   It needs 3 battery sets per drone (18 DB2000 packs), and 72 h uses only about 3% of their rated cycle life [D]. It needs a crew of **7** across two 12-h shifts [A].
5. **Regulation is the slowest part, so do it before the quake.**
   - **Aviation:** a Taiwanese government agency can get CAA consent to fly disaster missions at night, beyond line of sight and over people (Civil Aviation Act Arts. 99-14(2) and 99-16; drone rules Art. 34) [S].
     A ≥ 25 kg drone still needs a physical inspection, an operations manual and professionally licensed pilots [S]. PRC-made drones such as DJI's are restricted for government use [S] *(weak)*.
   - **Spectrum:** NCC low-power rules allow UWB only at 4–9 GHz and ban it on aircraft [S]. The 0.4–2 GHz belly radar therefore needs NCC special approval and a frequency assignment.
   - **Timeline:** plan **12–18 months** to the first operational kit [A]. After that, launching at a disaster needs only notification through the emergency operations centre (EOC), much as under Japan's Art. 132-92 exemption [S].

---

## 1. Taiwan's risk and readiness

### 1a. Seismic hazard

| Item | Value | Tag / source |
|---|---|---|
| P(M≥6.5 somewhere in Taiwan, 50 yr) | **99%** | [S] CWA seismology centre director, via [World Journal 2025](https://www.worldjournal.com/wj/amp/story/121221/8974642) *(weak: news)* |
| P(M≥7, Taiwan, 50 yr) | **54%** | [S] same |
| Tainan 中洲構造, P(M≥6.9, 50 yr) | **39%**, the highest single structure; 後甲里 fault was > 40% in the 2015 chart | [S] CWA/TEM via [CommonWealth](https://www.cw.com.tw/article/5122838) *(weak: page blocked, search summary only)* |
| Taipei 山腳 (Shanchiao) fault, P(50 yr) | **7%**, recurrence 511–1,220 yr | [S] TEM chart via [The Reporter](https://www.twreporter.org/a/dangerous-fault-architect) *(weak)* |
| Shanchiao south segment M6.6 scenario | **4,400** collapsed buildings, 13,000 severely damaged, **4,100** dead or gravely injured | [S] [Taiwan News](https://www.taiwannews.com.tw/news/5674244) *(weak)*; the 4,100 baseline is also in the [Control Yuan deck 2026](https://cybsbox.cy.gov.tw/CYBSBoxSSL/edoc/download/76780) |
| Active faults crossing or near dense urban areas | **25**, of which only 4 have scenario models. Tainan: 新化, 後甲里, 六甲, 左鎮; Taichung: 車籠埔, 大甲, 彰化; Kaohsiung: 旗山, 小崗山, 潮州 | [S] [Control Yuan deck 2026](https://cybsbox.cy.gov.tw/CYBSBoxSSL/edoc/download/76780) |
| Recent urban collapses | Weiguan, Tainan 2016 (M6.4, 115 dead in one building); Hualien 2024 (Uranus building tilted) | [S] IMPACT_EVIDENCE §4 |

We did not find TEM PSHA city-by-city exceedance values for Taipei, Taichung, Tainan and Kaohsiung (they are in Earthquake Spectra 36(1_suppl), 2020), or 30-year fault probabilities. Treat the hazard row as island-wide.

### 1b. Building stock at risk

| City | Homes ≥ 30 years old | Share | Tag |
|---|---|---|---|
| Taipei | 634,145 of 901,691 | **70%** | [S] MOI tax-record data (Q4 2020) via a Tainan City 危老 slide deck *(weak: search summary; the [PDF](https://twur.nlma.gov.tw/resources/website/download_file/67/05-臺南市危老重建經驗分享_V2.pdf) refused our fetch)* |
| Kaohsiung | 546,626 of 1,079,989 | **51%** | [S] same |
| Tainan | 348,708 of 704,118 | **50%** | [S] same |
| New Taipei | 756,093 of 1,646,064 | **46%** | [S] same |
| Taichung | 407,570 of 1,057,489 | **39%** | [S] same |

- Taiwan's seismic design code was overhauled after Chi-Chi (1999). The government's 安家固園 programme subsidises a free initial seismic screen for private buildings permitted **before 31 Dec 1999** [S] *(weak: search summary of nlma.gov.tw)*.
- Approvals under the 危老 (dangerous-and-old) rebuilding scheme: 3,837 by Aug 2024 [S] ([MOI](https://www.moi.gov.tw/News_Content.aspx?n=4&s=321319)) and 4,969 by May 2026 [S] *(weak)*. That is a few thousand buildings against a stock of millions of old homes [D].
- **Gap:** we found no count of soft-storey (騎樓 / 軟弱層) buildings, of pre-1999 RC buildings of 6+ storeys, or of completed screenings. Weiguan (1994, 16 storeys, ground-floor shops) is the type we design for.

### 1c. Fire service and USAR capacity

| Item | Value | Tag / source |
|---|---|---|
| Firefighters, Taiwan (end 2025) | **17,648** (authorised 23,707) | [S] [NFA 消防統計年報 114](https://www.nfa.gov.tw/cht/index.php?act=download&ids=26567) |
| Firefighters by city | New Taipei 2,520 · Taichung 1,852 · Taipei 1,819 · Kaohsiung 1,635 · Tainan 1,286 | [S] same |
| Fire engines | **2,636** (1,562 water tenders, 229 aerial ladders, 221 chemical) | [S] same |
| Rescue vehicles | **2,311**, of which **209 救助器材車 (rescue equipment trucks)**, 284 command, 716 reconnaissance | [S] same |
| NFA's own special search and rescue team (特種搜救隊) | 66 people (76 authorised) | [S] same |
| Taichung special search battalion | 93 people; 264 rescue vehicles in the city | [S] [Taichung FB 112 yearbook](https://www.fire.taichung.gov.tw/df_ufiles/h/112年消防統計年報.pdf) |
| Domestic USAR classification (NAP, on the INSARAG model) | **10 teams** at medium or above (Sept 2024): **heavy** Taipei, New Taipei, Kaohsiung, Tainan; **medium** Taichung, Taoyuan, Taitung, Pingtung, Hualien, Hsinchu City | [S] [CNA 2024-09-08](https://www.cna.com.tw/news/asoc/202409080168.aspx), [CNA 2024-05-20](https://www.cna.com.tw/news/aloc/202405200192.aspx) |
| INSARAG external classification (IEC) | **none found** for any Taiwan team (Taiwan is not a UN member; teams are graded domestically) | [S] absence in the CNA reports above; not a proof |
| Average arrival at fire scenes, Taiwan (2020) | **524 s (8.7 min)**; Taichung 545 s. Call-to-dispatch 42 s; turnout 54 s (day) / 60 s (night) | [S] NFA figures quoted in [NOWnews](https://www.nownews.com/news/5355038) *(weak)*; minutes [D] |
| Response targets, international | NFPA 1710: turnout 60–80 s, first engine travel **240 s**, full first-alarm travel **480 s**, 90% of calls | [S] [DC FEMS](https://fems.dc.gov/page/fire-response-time) |
| Rescue-truck generators | Hualien's gripper-arm rescue truck: petrol generator ≥ 13 kVA [S] *(weak)* ([PTS](https://news.pts.org.tw/article/467178)); international 8 kW hydraulic to 25–40 kW PTO | [S] IMPACT_EVIDENCE §6 |

We found no official national count of fire stations (消防分隊) and no 5- or 6-minute response target. The kit plan below is therefore built from travel time and urban area, not from the station count.

## 2. Deployment timeline: quake to first ping

A Weiguan-like case: an M6.4 quake at 03:57, a 16-storey collapse in Tainan, and a kit stationed with Tainan's heavy USAR team.
**Central** case; the low and high columns are the fastest and slowest plausible cases.
The sim numbers come from `sim/recon.py` (3 drones, cruise 4 m/s in recon, LiDAR swath ±25 m on large sites, 18 s rotors-off listening per perch) and the measured runs in [TASKING_INTEGRATION.md](TASKING_INTEGRATION.md).

| Clock (central) | T+ (central) | Low – high | Step | Tag / source |
|---|---|---|---|---|
| 03:57:00 | 0:00 | — | Quake | Weiguan 2016 [S] |
| 03:57:09 | 0:00:09 | 0:00:08 – 0:00:15 | CWA earthquake early warning reaches phones (eBEAR first alert 7.9 s for the 2025 Dapu M6.4; 9 s and 15 s for Hualien 2024) | [S] [JpGU 2025](https://confit.atlas.jp/guide/event/jpgu2025/subject/SCG63-01/date), [SSA abstract](https://seismosoc.secure-platform.com/a/solicitations/35/sessiongallery/1056/application/11871) |
| ~04:02 | ~0:05 | 0:01 – 0:05 | CWA earthquake report (automatic location ~60 s, report within ~5 min) | [S] same search; CWB 2018 data *(weak: summary)* |
| 04:03 | 0:06 | 0:02 – 0:15 | First 119 call; Tainan FB dispatches the 5th battalion. **Kit is dispatched with the heavy USAR team on the same call** [A] | call time [S] [Tainan FB](https://w3fs.tainan.gov.tw/001/Upload/137/epaper/337/消防-0206震災搶救-電子報-0222(結合搶救科)%20.pdf) |
| 04:05 | 0:08 | 0:03 – 0:17 | Turnout: crew on the truck, kit already loaded and on trickle charge. Taiwan average turnout 54–60 s; we add 1 min for a heavier vehicle | [S] NOWnews, [D] |
| 04:25 | 0:28 | 0:08 – 1:02 | Travel: 20 min central (5 min if the kit sits at the nearest station; 45 min across a damaged city). Normal Taiwan fire arrival averages 8.7 min, but post-quake roads are slower | [A], anchored on 524 s [S] |
| 04:33 | 0:36 | 0:13 – 1:17 | Size-up with the incident commander, unfold 3 drones, insert batteries, raise the ~10.5 m telescopic mesh mast, pre-flight checks, notify the CAA through the EOC (no permit wait under drone rules Arts. 33–34, §5) | [A] 8 min (5–15). No published FlyCart setup time; it is a two-person lift [S] *(weak: [reseller](https://heliguy.com/blogs/posts/dji-flycart-30-increases-roi-for-drone-deliveries))* |
| 04:37 | 0:40 | 0:16 – 1:22 | **First acoustic lead**: the first perch-and-listen round of recon, about 4 min into the flight | [D] from the sim's recon schedule |
| 04:42 | 0:45 | 0:20 – 1:29 | Recon done: 320 m site mapped by LiDAR, 4–6 perch rounds, zone proposals out. **9 min central (7–12)** | [D] below |
| 04:43 | 0:46 | 0:21 – 1:31 | Operator accepts proposals (the sim's unattended auto-accept waits 30 s) | [sim] `RECON_ACCEPT_S = 30` |
| 04:45 | 0:48 | 0:22 – 1:33 | First drone lands on a flat spot on the top lead: transit < 1 min, set down 8 s, IMU seating check | [sim] `PodSpec` lower 8 s (landing) |
| 04:46 | **0:49** | **0:23 – 1:34** | **First radar ping**: whole-dwell breathing verdict after a 45 s dwell. A *candidate search region*, never a confirmed person | [sim] 45 s dwell (VIDEO_PLAN / vitals) |
| 05:00 | ~1:03 | — | Tainan FB forward command post is in place (Weiguan: within 1 h) | [S] Tainan FB |
| ~16:00 | ~12 | — | National special search teams arrive (Weiguan: all within 12 h) | [S] Tainan FB |
| — | 12–56 h | — | First international team arrives (14-quake range) | [S] [Rom & Kelman 2020](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/) |

**Recon scaling [D].**
- **60 m grid site [sim]:** 176 s with 3 drones, 100% mapped, 3 perch rounds. Flight is only ~26 s per drone (106 m of lanes at 4 m/s), so perch rounds dominate.
- **320 m collapse site:** `coverage_path` gives **956 m of lanes per drone** with a ±25 m swath, 239 s at 4 m/s.
  Each perch round costs about 58 s: a 12 s climb and an 18 s descent to ~37 m above the tallest structure, 18 s listening and ~10 s settling.
  - 4 rounds: 239 + 4 × 58 ≈ **470 s (7.8 min)**.
  - 6 rounds (the sim's collapse run logged 18 perches across 3 drones): 239 + 6 × 58 ≈ **590 s (9.8 min)**.
  - Central **9 min**, range 7–12 min. The high end allows for gusts or one 60 s battery swap.
  - The FlyCart 30 can cruise at 20 m/s [S] ([DJI](https://www.dji.com/flycart-30/specs)), but the sim maps at 4 m/s for LiDAR density, so we don't count on the extra speed.
- **Sim caveat.** In the sim's grid evaluation (`--delivery pod`, radar on the rubble surface; literature propagation), the median time to the first ping away from the reported start pile is **1,037 s (745–1,769)** after launch [sim] ([EVAL_GRID.md](EVAL_GRID.md)).
  That is slower than the timeline above, because the first leads are often empty or out of reach. Our timeline is the time to the first *ping attempt* on the top lead. **The time to a true positive can be 15–30 min later.**

**What it buys [D].** Interpolating IMPACT_EVIDENCE §1b, P(alive at extrication) is about 85–90% at T+1 h, falling to about 50% at T+24 h.
A kit's first ping at T+0.8 h, against national specialist teams at T+12 h, moves the first technical search about 11 h earlier.
It doesn't shorten an extrication, which takes 8–9 h on average for an RC victim [S, IMPACT_EVIDENCE §2].

## 3. Coverage: how many kits

### Method [D]

- **Coverage area per kit** = 0.827 × π × (v × T / c)², where:
  - T is the travel budget;
  - v is the post-quake urban road speed, **20 km/h** (15–30) [A];
  - c is the road circuity factor, **1.3** [A];
  - 0.827 is the hexagon-in-circle packing factor [D].
- **Results:** a 10 min budget covers **17 km²**, 20 min covers **68 km²** and 40 min covers **273 km²**.
- **What it covers:** Taiwan's urban-planning areas contain **2,159 km² of urban development land**: housing, commerce, industry and public facilities. They hold **18.77 M people**, 80% of the population [S] ([NLMA 都市計畫統計 113](https://www.nlma.gov.tw/uploads/files/2cb8489b780b1d9e00989a6d4dc50ff2.pdf)).
- **Per-city development land** = the city's planned area [S, same] × the national development share of 44.3% [S]. This is a [D] estimate.
  For New Taipei it gives 553 km², which is inflated by large protection-zone special districts, so we use **300 km²** [A].

### Kits per city

| City | NAP USAR grade [S] | Dev. land km² [D] | Low: 1 per graded team | **Central: ≤ 20 min, +1 for a heavy team's 2nd worksite** | High: ≤ 10 min, +1 heavy |
|---|---|---|---|---|---|
| Taipei | heavy | 120 | 1 | **3** | 9 |
| New Taipei | heavy | 300 [A] | 1 | **6** | 19 |
| Taoyuan | medium | 159 | 1 | **3** | 10 |
| Taichung | medium | 239 | 1 | **4** | 14 |
| Tainan | heavy | 233 | 1 | **5** | 15 |
| Kaohsiung | heavy | 188 | 1 | **4** | 12 |
| 16 other counties/cities (668 km²) | 4 medium (Taitung, Pingtung, Hualien, Hsinchu City) | 668 | 4 | **16** (one per county fire bureau; coverage alone needs 10) | 40 |
| NFA national reserve (特搜, road or air move) | — | — | 1 | **2** | 2 |
| Maintenance float (~10%) | — | — | 1 | **4** | 12 |
| **Taiwan total** | | **2,159** | **12 kits / 36 drones** | **47 kits / 141 drones** | **133 kits / 399 drones** |

An INSARAG heavy team must be able to run 2 worksites for 24 h a day [S, IMPACT_EVIDENCE §2]. That is why each heavy team gets a second kit.

### Taiwan rollout options

| Option | Kits | Coverage | Capital [D] | Crews trained | When it fails |
|---|---|---|---|---|---|
| **Low**: one kit per NAP-graded team + NFA | 12 | the 10 graded teams' home cities, within ~40 min; everywhere else, 1–3 h by road | **US$4.0 M** | 84 (0.5% of firefighters) | Two collapses in one city, or any county without a graded team |
| **Central**: 20-min urban coverage + every county | 47 | 90%+ of urban development land within ~20 min of travel [A] | **US$15.5 M** | 329 (1.9%) | A Shanchiao-scale event with thousands of collapses. Kits then flow in from other cities (§2: 1–3 h) |
| **High**: 10-min urban coverage | 133 | the kit arrives as fast as an ordinary engine | **US$43.9 M** | 931 (5.3%) | Cost, and crew currency: at 400 drones, monthly practice gets hard (§6) |

- **Per-kit capital** ≈ 3 × US$87k (Harris H6HL build) + US$51k van kit + US$17k extra batteries = **US$0.33 M** [D from HARDWARE.md §1, §13].
  The extra batteries are 6 more DB2000-class packs, so each drone has 3 sets (§4). On FlyCart 30 airframes it would be about US$0.25 M, but Taiwanese agencies can't buy PRC-made drones (§5a).
- **Running cost:** about **15% of capital a year** [A], for battery replacement, maintenance, insurance and training. Central: about US$2.3 M a year.
  For scale, FEMA funds each of its US&R task forces at about US$1.2 M a year [S, IMPACT_EVIDENCE §5].
- **Recommended path:**
  1. **Pilot the low option in Tainan and Taipei** (2 kits, 1 year).
  2. **Scale to the 12 low-option kits** once the CAA/NCC paperwork (§5) is done.
  3. **Grow towards central** as the cities' 救助器材車 fleets (209 nationally [S]) are renewed. Each new rescue truck is specified with a ≥ 25 kW generator and a kit bay.

## 4. Power and endurance in the field: the 72 h ops budget

Inputs reused from IMPACT_EVIDENCE §6:
- average charging demand of **6.85 kW per drone in operation** (4.6–10.7) [D];
- 18.7 min of flight per pack set (15–21) [D];
- C8000 charge, 36 min for two packs (26–36) [S];
- a 25 kW PTO generator supports ~3.6 drones [D].

### Generators and diesel

| Item | Value | Tag / source |
|---|---|---|
| Rescue-truck generators | 8 kW hydraulic (common), 25 kW PTO (heavy rescue), up to 40 kW; Taiwan example ≥ 13 kVA petrol | [S] IMPACT_EVIDENCE §6; [PTS](https://news.pts.org.tw/article/467178) *(weak)* |
| Dedicated diesel genset, 25–27 kW (Kohler 20REOZJB, 60 Hz standby) | 8.6 L/h at 100%, 6.7 at 75%, 4.9 at 50%, 3.1 at 25% load | [S] [Kohler spec sheet](https://cdn.shopify.com/s/files/1/0457/0408/5661/files/g5202_20REOZJB_Spec_Sheet.pdf) |
| → electrical output per litre | **3.0 kWh/L** at 100%, 2.9 at 75%, 2.65 at 50%, 2.1 at 25% (26 kW basis) | [D] |
| Truck engine driving a PTO generator at light load | **1.6–2.4 kWh/L** (a 7–9 L truck engine idling is far less efficient than a matched genset) | [A] |
| Generic diesel generator | 0.28–0.38 L/kWh (2.6–3.6 kWh/L) at 25–50% load on large sets | [S] [USP&E chart](https://uspeglobal.com/articles/diesel-generator-fuel-consumption-chart/) *(weak)* |

**Recommendation:** a towed 25–30 kW diesel genset on the kit trailer, rather than the truck's own PTO. It is about 1.3–1.8× more fuel-efficient, the truck stays free to move, and the genset can be parked ≥ 100 m downwind of the listening sites (HARDWARE.md §13).

### Battery rotation [D]

- A set spends `flight_min / rotor_duty + 2 min swap` on the aircraft.
  - Central: 18.7 / 0.65 + 2 = **28.7 min**. Landed, rotors-off listening stretches each set.
  - High: 15 / 0.8 + 2 = **18.7 min**.
- The other sets must finish charging and cooling in that time: 36 min charge + ~10 min cooling (20 min on a hot Taiwanese summer day [A]).
- Sets per drone = 1 + ⌈(charge + cool) / time on aircraft⌉:
  - central **3 sets** (6 DB2000 per drone, 18 per kit);
  - low 2 sets;
  - high **4 sets**, for continuous hover-heavy work in heat.
  This agrees with HARDWARE.md §12.
- **Chargers:** one C8000 per drone (7.2 kW each). Three running at once is 21.6 kW, which is why the genset is 25–30 kW.

### 72-hour budget per kit [D]

Availability is the share of the 72 h that the kit is actually working. The rest goes to aftershock evacuations, weather stand-downs, re-tasking and maintenance:
0.6 / **0.75** / 0.9 [A].

| Item | Low | **Central** | High | How |
|---|---|---|---|---|
| Drone charging energy | 596 kWh | **1,110 kWh** | 2,080 kWh | 3 drones × demand × 72 h × availability |
| Base load (van electronics, lights, mast, Starlink/OneWeb) | 58 kWh | **108 kWh** | 180 kWh | 0.8 / 1.5 / 2.5 kW × 72 h [A] |
| **Total electricity** | 654 kWh | **1,218 kWh** | 2,260 kWh | |
| Peak electrical load | 14.6 kW | **22.0 kW** | 34.6 kW (needs a 40 kW set or two sources) | |
| Fuel efficiency | 2.9 kWh/L (genset) | **2.4 kWh/L** (genset at part load) | 1.6 kWh/L (truck PTO) | above |
| **Diesel, 72 h** | **225 L** | **~510 L** | **~1,410 L** | |
| Average burn | 3.1 L/h | **7.0 L/h** | 19.6 L/h | IMPACT_EVIDENCE §6 assumed 7–12 L/h for the central case [A] |
| Fuel drops (200 L tank on the genset [A]) | 1 | **2–3** | 7 | one per ~24 h in the central case |
| Pack-set charge cycles per drone | 62 | **116** | 218 | energy / 3.18 kWh per set (20→100%) |
| Cycles per pack | 31 | **39** | 55 | ÷ sets per drone |
| Share of DB2000 rated life (1,500 cycles) | 2% | **2.6%** | 3.7% | [S] rated cycles *(weak: [reseller](https://dronefly.com/dji-db2000-intelligent-flight-battery.html))* |
| Rotor-hours per kit | 65 h | **105 h** | 156 h | 3 × 72 × availability × rotor duty (0.5 / 0.65 / 0.8) |
| Radar modules' 99 Wh packs | swapped at every battery-swap visit; ~5 h at 16 W (HARDWARE §11) → 6 per radar module, charged on the van's 1.8 kW inverter set | | | [D] |

### Crew shifts (72 h)

| Role | Per shift | Task |
|---|---|---|
| **Mission operator** (in the van) | 1 | tasking, accepting proposals, the dashboard, approving broadcasts, liaison with the USAR worksite leader |
| **Remote pilot / safety pilot** | 1 | holds the professional licence (§5); keeps visual line of sight of the active drones; can take manual control; enforces stand-offs from rescuers |
| **Technician** | 1 | battery rotation, chargers, genset, radar-module battery swaps, airframe checks between sorties |
| **Kit leader** | 1 per kit, day shift | safety officer link, logs, CAA/NCC notifications, fuel and parts requests |

Two 12-h shifts of 3, plus the kit leader, plus 1 relief makes **7 people per kit for 72 h** [A].
This matches how USAR teams run: an INSARAG heavy team works 24 h a day across 2 worksites [S, IMPACT_EVIDENCE §2].
The drone crew is attached to the USAR team, not a separate unit. Phoenix FD flies with a two-person crew, a pilot and an observer [S] *(weak: [CleanTechnica](https://cleantechnica.com/2022/08/08/phoenix-fire-department-launches-drone-program/))*.
We add the technician because heavy-lift battery logistics are a full-time job.

## 5. Regulation

Sources: 民用航空法 (Civil Aviation Act) [CAA Act](https://law.moj.gov.tw/LawClass/LawAll.aspx?pcode=K0090001);
遙控無人機管理規則 (drone rules, amended 14 Nov 2024) [Rules](https://law.moj.gov.tw/LawClass/LawAll.aspx?pcode=K0090083);
電信管理法 (Telecommunications Management Act) [TMA](https://law.moj.gov.tw/LawClass/LawAll.aspx?pcode=K0060111);
NCC LP0002 low-power RF rules, 2020 text [LP0002](https://forum.flipper.net/uploads/short-url/5qQHHxW40bD5Vs4cEaPzTiDwmmU.pdf) *(weak host)*;
個人資料保護法 (Personal Data Protection Act) [PDPA](https://law.moj.gov.tw/LawClass/LawAll.aspx?pcode=I0050021).
Some articles were read through a summariser. Check the exact wording before filing.

### 5a. Taiwan: aviation (CAA)

| Topic | Rule | What it means for a fire-department kit | Tag |
|---|---|---|---|
| Registration | Government-owned drones must be registered **at any weight** (Act Art. 99-10); private drones from 250 g (Rules Art. 6) | register all 3 airframes per kit | [S] |
| Operator licence | Licences are 學習 / 普通 / 專業 (learner / ordinary / professional). Operators of government drones need one at any weight; private drones need a professional licence from 15 kg (Rules Arts. 19–20) | every pilot holds a **專業** licence | [S] |
| Heavy class | **≥ 25 kg needs a physical inspection (實體檢驗), valid 3 years** (Rules Art. 15); modified drones need an inspection certificate (Act Art. 99-11) | a 95 kg airframe with its belly radar is inspected as a configuration, once, and re-inspected every 3 years. We found no 150 kg cap in the current rules | [S] |
| Government operator approval | A government agency files registration proof, a pilot roster and an **operations manual (作業手冊)** with the CAA before flying (Rules Art. 30) | do this in peacetime; it is the standing approval | [S] (validity 1 or 2 years: our two readings disagree) |
| General limits | ≤ 400 ft; **no night flight**; **visual line of sight only**; **no flight over crowds** (Act Art. 99-14(1) items 1, 6, 7, 5). Extended visual line of sight is limited to a 900 m radius and 400 ft, with an observer within 300 m (Rules Art. 28) | our 320 m site fits inside extended visual line of sight even without an exemption | [S] |
| Exemptions for government | Government agencies may be exempted from items 1–8, which covers altitude, night, BVLOS and crowds (Act Art. 99-14(2)). For statutory duties such as disaster response, CAA consent can also replace the zone and activity approvals (Act Art. 99-16; Rules Art. 34) | **the route for night, BVLOS and over-people flying**; get the consent in advance as a standing disaster SOP | [S] |
| Restricted zones / activity plans | Normally filed 15 days ahead (30 days near military airspace); approvals last 3 months, **1 year for government agencies** (Rules Arts. 31–32) | not usable in an emergency, so rely on Art. 34 | [S] |
| In a disaster | Drones follow the disaster response centre commander, who applies to the CAA for the operator (Rules Art. 33) | the kit leader notifies through the EOC; no permit wait at T+30 min | [S] |
| Cybersecurity | From **1 Dec 2027**, government drones with navigation equipment need a cybersecurity test report from a body designated by the Ministry of Digital Affairs (Rules Art. 31(6)) | budget for the test in the rollout | [S] *(weak: [summary](https://www.rootlaw.com.tw/LetterContent.aspx?LCode=20&Soid=246047))* |
| PRC-made drones | Executive Yuan principles (Apr 2019) restrict products that endanger national cybersecurity, drones included, for central and local agencies. Agencies were told to replace PRC drones by end-2021 | **a DJI FlyCart 30 is not procurable by a Taiwanese fire bureau**; build on the H6HL class (HARDWARE §2) | [S] *(weak: [PTS](https://news.pts.org.tw/article/429396), [CTEE](https://www.ctee.com.tw/news/20201229700901-439901))* |

### 5b. Taiwan: spectrum (NCC) and privacy

| Link | Rule | Status for us | Tag |
|---|---|---|---|
| 2.4 GHz rescue hotspot | LP0002 4.10.1.2: ≤ **1 W** peak conducted; above 6 dBi of antenna gain, power must drop by the same amount | our AP runs at 20 dBm (0.1 W): **compliant** with a type-approved AP | [S] |
| 5.8 GHz mesh | LP0002 5.7.3.3: 5.725–5.85 GHz, ≤ 1 W conducted, PSD ≤ 30 dBm / 500 kHz | **licence-free** if the radio holds NCC type approval (審驗) | [S], [D] |
| **0.4–2 GHz belly radar** | LP0002 §5.12 allows UWB **only at 4.224–4.752, 6.336–7.920 and 7.392–8.976 GHz**, has no ground-penetrating or wall-imaging category, and **bans UWB on aircraft** (5.12.4.1). Each narrow step of a stepped-frequency sweep might instead fall under the general sections (not read) | **does not fit the low-power rules.** It needs NCC 專案核准 (special approval, TMA Art. 66) plus a frequency assignment for emergency or R&D use (TMA Arts. 52, 56, 57). Mitigation: firmware lets the radar transmit **only when the drone has landed on the rubble** (IMU still + landing detected), never while airborne, at a few dBm, and it skips GNSS, cellular and rescue-radio bands (HARDWARE §4) | [S] rules; [A] mitigation |
| Type approval | Controlled RF equipment must pass 審驗 before sale or use, "unless specially approved" (TMA Art. 66; 審驗 Rules Art. 5 allows up to 5 self-built units for own use) | the pilot's 2 kits (6 radar modules) are just over that self-built limit, so file 專案核准 | [S] ([審驗 Rules](https://law.moj.gov.tw/LawClass/LawAll.aspx?pcode=K0060137)) |
| Hotspot privacy | Public agencies may collect personal data within their statutory duties (PDPA Art. 15). They may use it beyond the original purpose to remove danger to the data subject's own life or body (Art. 16 item 3) or to prevent serious harm to others (item 4). Personal data includes indirectly identifying data (Art. 2) | survivors' portal messages are lawful for the fire bureau to collect. Treat MAC/RSSI as personal data (no ruling found) and keep HOTSPOT.md §7: no MAC stored, opt-in only, deleted after the operation. The new 個人資料保護委員會 (2025 amendment) is not yet in force | [S]; MAC status [A] |

### 5c. How others do it

| Jurisdiction | Emergency route | Heavy (> 25 kg) drones | Speed | Tag / source |
|---|---|---|---|---|
| **Japan** | Aviation Act **Art. 132-92**: the state, local governments, or anyone they commission, flying for search and rescue, are **exempt** from the airspace, flight-method, third-party, flight-plan and flight-log rules. They must self-impose equivalent safety, and NOTAMs are still needed near airports, in 緊急用務空域 or above 150 m | same exemption | **immediate** | [S] [MLIT guideline](https://www.mlit.go.jp/common/001110204.pdf), [MLIT 緊急用務空域](https://www.mlit.go.jp/common/001406479.pdf) |
| **US** | Special Governmental Interest (SGI) amendment via the FAA SOSC. It needs an existing Part 107 certificate or a COA. BVLOS usually needs a TFR and takes longer. Public aircraft operations are possible under a COA | Part 107 stops at 55 lb (25 kg); above that, a 49 USC 44807 exemption | SGI: minutes to hours *(weak, anecdotal: one county reported 40 min)*; 44807: ~4–9 months *(weak)* | [S] [FAA emergency page](https://www.faa.gov/uas/advanced_operations/emergency_situations), [14 CFR 107.3](https://www.law.cornell.edu/cfr/text/14/107.3), [49 USC 44807](https://www.law.cornell.edu/uscode/text/49/44807), [DroneLife](https://dronelife.com/2021/03/11/sgi-waivers-special-government-interest-waivers-and-how-they-work/) |
| **EU** | Reg. 2018/1139 **Art. 2(3)(a)** excludes aircraft doing police, **search and rescue** and **firefighting** work from EU drone rules; Member States regulate them nationally, "with due regard to safety" | national (state aircraft) | depends on the state; for civil operators, specific-category (SORA) authorisations take 4–12 months (Denmark) or ~12 months (Finland) | [S] [EUR-Lex 2018/1139](https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX:32018R1139), [Danish CAA](https://www.en.droneregler.dk/commercial-drone-operations/overview-of-drone-operations-in-the-specific-category/sora-risk-assessment-for-the-specific-category), [Traficom](https://traficom.fi/en/unmanned-aviation/applying-operational-authorisation-specific-category) |
| Radar (US) | 47 CFR 15.509: ground-penetrating and wall-imaging radar only for law enforcement, firefighting, emergency rescue, research, mining and construction, with coordination. Average limits −65.3 dBm/MHz (960–1,610 MHz) and −53.3 (1,610–1,990 MHz); below 960 MHz the §15.209 limits apply. 15.510 through-wall imaging is law enforcement only | | | [S] [15.509](https://www.law.cornell.edu/cfr/text/47/15.509), [15.510](https://www.law.cornell.edu/cfr/text/47/15.510) |
| Radar (EU) | ECC/DEC/(06)08 + ETSI EN 302 066 (30 MHz–12.4 GHz): ground- and wall-probing radar for trained professionals, radiating into the ground or wall, non-interference basis. Airborne use is out of scope | | | [S] [CEPT](https://docdb.cept.org/document/402); [D] |

The belly radar's design choice helps in every jurisdiction. It radiates downward into the rubble only while the landed drone rests it on the surface, which is exactly the ground-probing case the US and EU rules allow. It never radiates from the air.

### 5d. Realistic certification timeline (Taiwan, before the first kit is on call) [A]

| Step | Duration | Basis |
|---|---|---|
| Procurement of non-PRC airframes (H6HL class) + integration | 4–8 months | HARDWARE §2 (quote-only airframe) [A] |
| CAA: ≥ 25 kg physical inspection + Art. 30 operations manual + standing Art. 99-16 / Art. 34 disaster consent | 3–6 months | no CAA processing time is published; activity plans alone take 15–30 days [S] |
| NCC: 專案核准 for the belly radar + emergency or R&D frequency assignment; type-approved mesh and Wi-Fi radios | 6–12 months, in parallel | no published days; type approval takes "2 weeks to 1 month" [S] *(weak: [iThome](https://www.ithome.com.tw/news/111270))*; the radar is novel |
| Pilot licences (專業) + type and system training (§6) | 3 months, in parallel | |
| MODA cybersecurity test (needed from 1 Dec 2027) | 1–3 months | [A] |
| **Total to the first operational kit** | **12–18 months** (24 if the NCC treats the radar as a new category) | |

Compare: a US 44807 exemption takes ~4–9 months *(weak)*, an EU SORA authorisation 4–12 months, and Japan needs no approval for government search and rescue. Taiwan's Art. 99-16 route is closer to Japan's once the peacetime paperwork is done.

## 6. Operations

### Training plan [A, anchored on sourced programmes]

| Block | Hours | Anchor |
|---|---|---|
| CAA professional remote-pilot licence (heavy class), ground + flight test | ~40 | CAA licence classes (§5); Phoenix FD trains ~50 h to its basic FAA licence [S] *(weak: [CleanTechnica](https://cleantechnica.com/2022/08/08/phoenix-fire-department-launches-drone-program/))* |
| Type rating: 95 kg airframe, landing on rubble, emergency procedures | 24 | [A] |
| Larpverse system: tasking, recon, reading pings, vitals honesty, broadcast approvals, hotspot privacy rules | 16 | [A] |
| Joint USAR exercise on a training rubble pile (ASR 2–3 with dogs and listening devices) | 2 × 8 | Japan's FDMA drone-advisor course is 4 days, including BVLOS, night and search training [S] ([FDMA 消防の動き 2022-02](https://www.fdma.go.jp/publication/ugoki/items/rei_0402_14.pdf)) |
| **Initial total** | **~96 h per person** | one 100-h commercial public-safety course is in the same range [S] *(weak)* |
| **Recurrent** | one 3-h fly day a month (≥ 1 flight hour per pilot), one quarterly night drill with the special search team, one annual multi-kit exercise | Cal Fire ≥ 1 flight h/month; Austin 3-h monthly fly days; DroneResponders found that the typical public-safety pilot averages 12 flights a year, which is "not enough" [S] *(weak: [Firehouse](https://www.firehouse.com/technology/drones/article/55297350/fire-tech-brief-5-things-to-know-before-starting-a-fire-department-drone-program), [DroneResponders](https://www.droneresponders.org/_files/ugd/ef6978_19517dfe38b34fad94d29c2b0c908491.pdf))* |

### Fitting into INSARAG ASR levels and incident command

INSARAG defines 5 assessment, search and rescue (ASR) levels [S] ([INSARAG Vol III Annex B20](https://insarag.org/wp-content/uploads/2020/04/INSARAG_Guidelines_Vol_III_-_Annex_B20_-_Assessment_Search_and_Rescue_Level.pdf)).
The table shows where the kit fits in each.

| ASR level (INSARAG) | Who does it [S] | Kit's role |
|---|---|---|
| 1 Wide area assessment | LEMA / UNDAC / the first USAR teams | Fresh LiDAR map of the block (satellite imagery is stale after a collapse); hotspot beacons for phones; collapse outlines for the sectorisation plan |
| 2 Sector assessment | USAR teams in the sector | **Main role.** Recon + perch-and-listen + zone proposals = viable live-rescue sites in minutes |
| 3 Primary search and rescue | USAR teams at the worksite | Radar landings on the top leads; the dig order goes to the worksite leader |
| 4 Secondary search and rescue | USAR teams | Re-check open leads and deep voids, layer by layer, as material comes off |
| 5 Full coverage | LEMA, sometimes with USAR | Life-sign checks of each layer before machines take it (the Weiguan practice [S, IMPACT_EVIDENCE §3]) |

- **Incident command:** the kit leader reports to the worksite / USAR team leader, inside the city's incident command structure, the way canine and technical-search elements do.
  The operator never tasks rescuers. The kit proposes a dig order, and the human team leader decides.
- **Weiguan context:** the Tainan FB forward command post stood up within 1 h, and the city EOC opened at 04:30 (T+33 min) [S] ([Tainan FB](https://w3fs.tainan.gov.tw/001/Upload/137/epaper/337/消防-0206震災搶救-電子報-0222(結合搶救科)%20.pdf)).
  The kit's first ping (T+~49 min) lands before the forward command post is complete. The kit leader therefore briefs the first-in battalion commander directly.

### Hand-off to rescuers: dig order and marking

The INSARAG marking rules come from [Vol III Annex B26](https://bomberosbogota.gov.co/sites/default/files/documentos/INSARAG%20Guidelines%20Vol%20III%20-%20Annex%20B26%20-%20USAR%20Team%20Marking%20System%20and%20Signalling.pdf) [S]:
- a large **"V"** about 50 cm high marks a *potential* victim location;
- **"L-n" / "D-n"** under the V are added only for *confirmed* live or dead victims;
- markings go as close as possible to the surface point;
- the **Rapid Clearance Marking** (a diamond with "C" = clear, equivalent to ASR 5 completion) is applied only after a full search.

Our pings are candidate search regions, never confirmations (RULES.md rule 9). So the hand-off works like this [A]:

1. **The dashboard's dig-order list is the hand-off.** It is printed or sent to the worksite leader's tablet with a ranked list (pile / zone ID, grid coordinates, evidence: voice / knock / breathing, confidence, time).
2. **A drone-found candidate gets a "V" only**, painted by the rescue team, or marked from the air with a dropped, weighted orange marker card carrying the zone ID (a 20 cm card is the same scale as INSARAG's RCM sign).
   It **never gets an "L"**: "L-1" is written only once a human searcher (a dog alert plus camera, voice contact, or sight) confirms the victim. This matches the US practice that a victim stays unconfirmed until a human sees or hears them [S] *(weak: [TEEX log book](https://teex.org/wp-content/uploads/WAS_PER213_TEEX-Unit_Log_Book.pdf))*.
3. **Drone data never applies an RCM "C" (clear).** A searched zone with no ping is reported as "searched, no ping" (the impact claim says sectors are never "cleared"). The USAR team decides whether to mark it clear.
4. **Signals:** the kit obeys the INSARAG horn signals [S, Annex B26]:
   - **"cease operations / quiet"** (one 3-s blast): drones land and listen, which is also our listening window;
   - **"evacuate"** (three 1-s blasts, repeated): drones climb to a holding altitude away from the pile and stop radar scans;
   - **"resume"** (long + short): drones resume tasks.

### Maintenance and storage between disasters

| Item | Practice | Tag |
|---|---|---|
| Battery storage | store DB2000 packs at the vendor's storage charge (~40–60%) in a ventilated, fire-rated cabinet in the station; the kit truck keeps 1 set per drone at full charge on a trickle charger for instant dispatch | [A]; vendor storage guidance not found online |
| Battery health | a monthly cycle on fly day; retire packs at 80% capacity or the vendor cycle limit (1,500 rated) | [S] rating *(weak)*, [A] retirement rule |
| Airframe | inspect props and motors before each fly day and every ~8 rotor-hours in an operation; yearly vendor service | [A] |
| Readiness checks | weekly power-on test of the drones, radar modules, mesh, hotspot and generator; monthly fly day; quarterly joint drill | [A] |
| Software | frozen, signed builds; updates only after a drill passes | [A] |

### Other uses between disasters

Using the kits keeps crews current and spreads the cost across more missions. Japan's fire service went from **9.6% of fire HQs using drones in 2017 to 52.9% in 2021** (383 of 724) [S] ([FDMA](https://www.n-bouka.or.jp/local/pdf/2022_02_16.pdf)).

- **Structural inspection after smaller quakes:** LiDAR maps of tilting buildings (Hualien 2024: the Uranus building moved 5 cm in a day [S] *(weak)*). These are routine, CAA-permitted flights.
- **Fires:** thermal overwatch, and the heavy airframe can lift hose or equipment to roofs (the FlyCart 30 is a 30 kg cargo drone [S]).
- **Floods and typhoons:** the loudspeaker and hotspot reach stranded residents; the drones can carry 5–30 kg of supplies.
- **Mountain search:** the Chamonix PGHM uses the FlyCart 30 for mountain rescue logistics [S] ([DJI](https://www.dji.com/media-center/announcements/pghm-chamonix-dji-flycart-30-mountain-rescue)) *(vendor)*. In Hualien 2024, drones searched Taroko gorge [S] *(weak)*.
- **Not for:** routine surveillance of people. The hotspot privacy rules in HOTSPOT.md §7 apply to every use.

## 7. Failure modes and mitigations

| Risk | Facts | Mitigation | Tag |
|---|---|---|---|
| **Wind** | FlyCart 30 max wind resistance **12 m/s**, measured with a 30 kg payload in calm test conditions "for reference only" | operating limit 10 m/s sustained, 12 m/s gusts [A]; landing on rubble in ≤ 8 m/s [A]; typhoon = no-fly | [S] [DJI](https://www.dji.com/flycart-30/specs) |
| **Rain** | IP55 aircraft and case; DJI publishes no rain limit; H6HL IP43 | fly in light rain only [A]; rain also raises the acoustic noise floor, so the belly radar takes over | [S] DJI; HARDWARE §2 |
| **Heat** | operating range −20 to 45 °C; DB2000s charge slower when hot | shade and fan the battery rack; budget a 4th set in summer (§4) | [S] DJI |
| **Aftershocks** | Hualien 2024: **707 aftershocks in the first ~4 days**, 27 of M≥5; two M6+ by day 5 | drones are the *safest* asset in an aftershock (no one on the pile); on the "evacuate" signal they climb and hold; landed drones lift off; re-map after any M≥5 because the pile moves | [S] [Wikipedia / CWA counts](https://en.wikipedia.org/wiki/2024_Hualien_earthquake) *(weak)* |
| **Dust** | airborne dust is detected by LiDAR as obstacles and hides what is behind it | wait out the first dust plume; multi-echo filtering (94% F1 particle classification reported); radar and acoustics are dust-blind | [S] [FSR 2021](https://easychair.org/publications/preprint/TCpW) |
| **GNSS denial among tall buildings** | urban-canyon multipath gives 10–50 m errors (95th percentile 47.8 m before mitigation in one study) | position against the LiDAR map (SLAM), which is our primary nav on site; RTK base on the van mast; geofence by map, not by GNSS | [S] [NASA PLANS 2025](https://ntrs.nasa.gov/api/citations/20250000809/downloads/PLANS-final.pdf), [PolyU 2019](https://research.polyu.edu.hk/en/publications/a-new-path-planning-algorithm-using-a-gnss-localization-error-map/) |
| **Comms loss** | cell networks fail; mesh shadowed by rubble | 5.8 GHz mesh with a drop node; store-and-forward on 2 TB; OneWeb (Chunghwa) via satellite, since Starlink wasn't commercial in Taiwan | COMMS_ARCHITECTURE §1; HARDWARE §13 |
| **Crowds** | relatives and volunteers gather at the pile (Weiguan: ~2,000 rescuers) | CAA rules on flying over people (§5); a police cordon; flight lanes over the pile only, launch pad ≥ 30 m from the public [A]; loudspeaker announcements | [S] IMPACT_EVIDENCE §4; [A] |
| **Airspace conflicts** | Weiguan: 6 helicopters on day 1 | one airspace coordinator at the command post; drones hold below 60 m AGL over the worksite and land when a helicopter comes in [A] | [S] IMPACT_EVIDENCE §4 |
| **Generator noise** | raises the acoustic noise floor | genset ≥ 100 m downwind; use quiet periods | HARDWARE §13 |
| **Fuel or road blocked** | a kit needs ~510 L in 72 h | a 200 L tank + jerrycans on the trailer; the fuel request goes through the EOC on day 1 | [D] §4 |
| **Procurement** | new DJI models are on the US FCC Covered List; Taiwan restricts PRC-made drones for government use (§5) | build on the NDAA-compliant H6HL class (HARDWARE §2) | [S] HARDWARE §2 |

## Biggest uncertainties

1. **City hazard numbers.** We have island-wide and per-fault probabilities from news reports of CWA/TEM figures, but not the TEM city exceedance tables.
2. **Building counts.** "30+ years old" is a proxy. We found no count of soft-storey or pre-1999 mid-rise RC buildings.
3. **Travel time after a quake.** The 20 km/h urban speed and the 1.3 circuity factor are assumptions, and they drive the kit count to the second power.
4. **Setup time.** No published unfold-to-launch time exists for a 95 kg drone. Measure it in the first drill.
5. **First ping vs first true ping.** The sim's median first ping away from the start pile is ~17 min after launch, not ~4 min. Field performance is unknown.
6. **Approval timelines.** These depend on how the CAA treats a 95 kg government drone and on whether the NCC grants an experimental licence for a 0.4–2 GHz through-rubble radar. Both need early, direct meetings.

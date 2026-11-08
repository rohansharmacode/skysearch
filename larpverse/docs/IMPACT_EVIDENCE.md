# Impact evidence: how we compare with real-world rubble search

Researched 8 Nov 2026 for the Larpverse defence. Every number carries a tag:
**[S]** sourced (stated by the linked source), **[D]** derived (our arithmetic on sourced numbers, shown here),
**[A]** assumption (team judgement, no direct source). Ranges are low / central / high.
The machine-readable version is [`data/impact_params.json`](../data/impact_params.json) (93 parameters, same tags).
Weak sources are flagged *(weak)*.

## Judge summary

1. **Time kills, and it kills fast.** Of the people Kobe's fire brigade pulled out in 1995, 80.5% were alive on day 1,
   28.5% on day 2, 21.8% on day 3 and about 6% after 72 h [S] ([Kobe FD data via Miyagi Pref.](https://www.pref.miyagi.jp/documents/11095/351765.pdf)).
   Weiguan Jinlong followed the same curve: 164 people were brought out alive on day 1, 7 on day 2 and 4 on day 3, and nobody
   was found alive after 62.3 h [S] ([Chiu & Tseng 2018](https://dm.cpu.edu.tw/var/file/40/1040/img/737/149757727.pdf)).
   In a peer-reviewed study of the Weiguan victims, extrication time was the strongest predictor of death, with an odds ratio of 38 at a 24 h cut-off and 95 at 72 h [S]
   ([Pan et al. 2019, JFMA](https://libmast.utm.my/Record/doaj.art-b389d61d901944c5b0bafe12007eef84)).
2. **Professional search arrives late and finds few people alive.** Neighbours and relatives perform 60–100% of extrications.
   Across 14 earthquakes, international teams' live rescues were below 0.85% of the death toll, the first team took 12–56 h to arrive,
   and each life saved cost about US$0.8–0.94 M [S] ([Rom & Kelman 2020, BMJ Glob Health](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)).
   After the first day, the people still trapped are the deep and the badly injured.
3. **Current tools cannot hear silent victims.** Acoustic and seismic listening devices only find people who can tap or call
   [S] ([Firehouse](https://www.firehouse.com/rescue/article/10506099/building-collapse-rescue-life-detection-systems)).
   In Bam (2003), 28% of injured survivors were unconscious when they were rescued [S] ([Nia 2008 via Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)).
   Dogs work 20–45 min and then rest for the same time [S] ([FEMA](https://webarchive.library.unt.edu/web/20090505222451/http://www.fema.gov/emergency/usr/canine.shtm)),
   and nobody has published an accuracy figure for dogs on rubble. Radar life detectors sense breathing to about 4.5–7 m [S]
   ([GSSI](https://www.fireengineering.com/technical-rescue/gssi-lifelocator-teex/)), but a rescuer has to carry them onto the pile one spot at a time.
   Our winch pod brings breathing radar and a microphone to many points per hour, and nobody has to stand on the pile.
4. **What happened at Weiguan:** 309 residents [S], 175 rescued alive [S] and 115 dead [S]. About 84 people were still unaccounted for at the end of day 2 [S].
   About 2,000 rescuers [D] and 23 dogs [S] worked the site. Manual search reached its limit at 64 h, when demolition with heavy machinery began [S],
   and the demolition took 192 h [S].
   **Honest caveat:** for the last live victims at Weiguan, the bottleneck was getting them out (12–29 h per deep victim [S]), not finding them.
   Our claim is that the system clears empty sectors faster, finds the silent victims, and checks for life signs layer by layer while machines dig.
   We do not claim it does the extrication.
5. **Power is feasible on rescue trucks, not on cars.** Half a tank in an average car (25 L) gives one heavy drone about
   **74 min of flight through the alternator** (range 29–186 min) [D], but converting that fuel takes about 14 h of idling [D].
   The same fuel through an inverter generator gives about **250 min** (range 121–411 min) [D].
   A heavy-rescue truck with a 25 kW PTO generator [S] can sustain about 3–4 drones [D], so a retrofit works without a city hub.
   It does need diesel resupply of about 9 L/h [A].

---

## 1. Survival against time trapped

### 1a. Share alive among people extricated in each window (the data that exist)

Each row gives alive / (alive + dead) for the people pulled out during that window. The figures are biased by selection, because the easy victims come out first.
Even so, they are the real-world curve that rescuers face.

| Window | Kobe FD 1995 [S] | Weiguan 2016 | Other |
|---|---|---|---|
| Day 1 | **80.5%** (486/604) | **92.7%** (164/177) [D from S] | Wenchuan, one Dujiangyan site, 12–15 May pooled: 23.8% (87/366) [S] ([CJEM 2009](https://search.bvsalud.org/gim/resource/en/wpr-395576)) |
| Day 2 | **28.5%** (129/452) | **25.9%** (7/27) [D from S] | |
| Day 3 | **21.8%** (89/408) | ~57% (4/~7, n tiny) [D] *(weak: Feb-8 bodies estimated from death-toll snapshots)* | |
| Day 4 | 5.9% ("after 72 h") | 0% (0 of ~79) [D] | |
| Day 5 | 5.8% | 0% | |

Sources: Kobe FD figures as reproduced by [Miyagi Prefecture](https://www.pref.miyagi.jp/documents/11095/351765.pdf), which cites
Kawata, *都市大災害* (we did not see the primary). The Weiguan daily counts come from [Chiu & Tseng 2018](https://dm.cpu.edu.tw/var/file/40/1040/img/737/149757727.pdf),
and the total of 175 alive from the [Tainan City Fire Bureau](https://w3fs.tainan.gov.tw/001/Upload/137/epaper/337/消防-0206震災搶救-電子報-0222(結合搶救科)%20.pdf).

Other anchors:
- Italy 1980: half of all trapped victims were out within 8 h (the "LD50" of extrication), and 93.6% of those rescued alive were out within 24 h [S]
  ([de Bruycker et al. 1983, Bull WHO](https://www.iris.unina.it/handle/11588/461321); the 93.6% is via [Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)).
- Kobe overall: the fire brigade extricated 1,892 people, of whom 733 were alive (39%). The Self-Defence Forces extricated 1,403, of whom 165 were alive (12%) [S]; the gap is consistent with the army's later rescues (our reading)
  ([Cabinet Office](https://www.bousai.go.jp/kyoiku/kyokun/hanshin_awaji/data/detail/1-4-1.html)).
- Turkey 2023: in a surgical cohort, extrication time predicted death with an AUC of 0.895 and an optimal cut-off of 21 h. The median was 34 h for those who died and 22 h for survivors [S]
  ([Özdoğan et al. 2025](https://pmc.ncbi.nlm.nih.gov/articles/PMC12000980)).
- Macintyre, Barbera & Smith (2006) reviewed 34 earthquakes. They report times to rescue (live rescues are recorded beyond 48 h and up to ~13–14 days), not
  survival percentages [S] ([PDM 21(1):4–19](https://resolve.cambridge.org/core/journals/prehospital-and-disaster-medicine/article/surviving-collapsed-structure-entrapment-after-earthquakes-a-timetorescue-analysis/D24F04BFBA9EDA264EA11010418B09B5)).
- **The widely quoted "91% at 30 min, 81% day 1, 36.7% day 2, 33.7% day 3, 19% day 4, 7.4% day 5" curve, and its variant with "95% / 53%", has no primary
  source we could trace.** It is usually attributed to Coburn/Tangshan. We found it only on an unsourced tutorial page
  ([ikcest-drr](https://ikcest-drr.data.ac.cn/tutorial/p9c3f)) *(weak)*. Don't put it on a slide; use Kobe and Weiguan instead.

### 1b. Point values for the model [D]

These are linear interpolations between the window mid-points (day 1 at 12 h, day 2 at 36 h, and so on). The low bound uses the lowest
window value seen and the high bound the highest.

| t | 30 min | 6 h | 24 h | 48 h | 72 h | 5 d |
|---|---|---|---|---|---|---|
| P(alive at extrication) | 91% (85–99) [A] | 85% (75–93) [D] | 50% (30–81) [D] | 25% (20–37) [D] | 14% (6–34) [D] | 5.8% (0–7.4) [D] |

### 1c. Unconscious or seriously injured victims

- No survival curve exists for unconscious victims. The evidence on direction is clear:
  - In the Weiguan cohort (n=309), being on a high floor gave an adjusted OR of 14.7 for severe triage acuity, and 80% of victims from collapsed blocks had crush injury [S]
    ([Pan et al. 2019a, JFMA 118:311](https://scholars.lib.ntu.edu.tw/handle/123456789/604741)).
  - The optimal cut-off for preschool children was 12 h, against 24 h for school-age children and adults [S] ([Pan et al. 2019b](https://libmast.utm.my/Record/doaj.art-b389d61d901944c5b0bafe12007eef84)).
  - The model uses a death-hazard multiplier for unconscious or severe victims of 1.5× (1.0–3.0×) [A].
- **Share unconscious:** in Bam (2003), 28% of 211 injured survivors were unconscious at rescue [S]
  ([Nia et al. 2008](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/), Table 1). That gives 15–40% for all trapped survivors [A]
  and 30–60% for those still trapped after day 1 [A].
  This sample counts only *survivors*, so people who were unconscious and died are missing from it. The true share among trapped people is probably higher.

## 2. How USAR time is split

**No published study gives a clean percentage split** of USAR time into search, extrication, medical and logistics. Statheropoulos et al. 2015
([Nat Hazards 75:57](https://link.springer.com/article/10.1007/s11069-014-1304-3)) name the phases and say they have "unforeseen time frames",
but do not quantify them. Rom & Kelman 2020 explicitly call for teams to start recording these timings. INSARAG defines ASR levels 1–5
(wide-area assessment → sector assessment → rapid search & rescue → full search & rescue → total coverage) and team capacity
(Heavy ≥ 59 people, 2 worksites, 24 h, 10 days; Medium ≥ 40, 1 worksite, 7 days; Light ≥ 17) [S]
([INSARAG Guidelines Vol II Man C](https://preparecenter.org/wp-content/uploads/2021/03/INSARAG-Guidelines-V2-Manual-C-IEC-and-R.pdf)),
but publishes **no search-time-per-structure standard**. FEMA Type 1 task force: 70 people [S].

### Components we can source

| Component | Value | Tag / source |
|---|---|---|
| Mobilisation + travel (international) | first team on site after 12–56 h | [S] [Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/) |
| Mobilisation (domestic, Weiguan) | Tainan FB called at 04:03, 6 min after the quake; forward command post within 1 h; national special-search teams within 12 h | [S] [Tainan FB](https://w3fs.tainan.gov.tw/001/Upload/137/epaper/337/消防-0206震災搶救-電子報-0222(結合搶救科)%20.pdf) |
| Extrication per live victim, RC collapse | Haiti USAR: average 8–9 h, range ~3–32 h | [S] [Fire Engineering](https://www.fireengineering.com/technical-rescue/california-task-force-rescues/) |
| Extrication, Weiguan deep victims | Lee T.-t. ~12 h of breaking from below (located ~33 h, out ~60 h); F14/E12 ~20 h through 4 slabs | [S] [Chiu & Tseng](https://dm.cpu.edu.tw/var/file/40/1040/img/737/149757727.pdf), [Taipei Times](https://www.taipeitimes.com/News/front/archives/2016/02/09/2003639107) |
| Labour per rescue, Kobe | 84 person-min (wooden house), 188 person-min (RC); shallow, mostly citizen rescues | [S] [Cabinet Office](https://www.bousai.go.jp/kyoiku/kyokun/hanshin_awaji/data/detail/1-4-1.html) |
| Search at a false site | e.g. 2 h of digging at an empty site before a 3 h live rescue (Haiti) | [S] [Fire Engineering](https://www.fireengineering.com/technical-rescue/california-task-force-rescues/) |
| Canine duty cycle | 20–45 min on, equal rest, 12 h shifts | [S] [FEMA](https://webarchive.library.unt.edu/web/20090505222451/http://www.fema.gov/emergency/usr/canine.shtm) |

### Estimated split of worksite time per live rescue, deep RC collapse [D]

The arithmetic for one live rescue at the central values:
- **Locate** (canine screen + technical confirmation + re-checks + false-alert digs): 1.5–4 h, central 2.5 h [A, anchored on the 2 h false dig above].
- **Access / shoring / breaching / extrication:** central 9 h [S, Haiti average].
- **Medical** in the void (crush prophylaxis, packaging), partly concurrent with extrication: central 1 h [A].
- **On-site coordination / briefings / safety stand-downs** (aftershocks, the Weiguan building shifting at 08:00 on day 1): central 1.5 h [A].
- Total ≈ 14 h. Shares: locate 2.5/14 = **18%**, extricate 9/14 = **64%**, medical 1/14 = **7%**, coordination 1.5/14 = **11%**.
- Ranges in the parameter file: locate 10–35%, extricate 50–80%, medical 5–15%, coordination 5–20% [D].

**Calibration of search rate at Weiguan [D]:** 4 sectors worked around the clock until 64 h, when "all reachable spaces were re-checked with no life signs"
[S, Chiu]. 4 × 64 = 256 sector-team-hours over a collapsed area of 13,142 m² [S] gives **~2 sector-team-hours per 100 m²**, a range of 1–4.
This includes extrication time, so it overstates pure search time, and it is crude.

**What this means for us:** extrication dominates the worksite, so a faster search does not shorten an individual rescue. It changes
*which* voids the scarce extrication crews open first, and how soon empty sectors are written off. Writing off a sector early is the "negative information" that let
Weiguan start the machines at 64 h. That decision came after crews had searched by hand to the limit of what they could reach.

## 3. Current technical search and its limits

| Method | What it needs | Known performance | Tag |
|---|---|---|---|
| Acoustic / seismic (Delsar LD3: 6 seismic + 2 acoustic sensors, 1–3,000 Hz) | **site silence and a victim who taps or calls**; debris blocks sound "beyond a few feet" | no published field sensitivity; for unconscious victims ≈ 0 | [S] [Firehouse](https://www.firehouse.com/rescue/article/10506099/building-collapse-rescue-life-detection-systems); P(detect) 0.6–0.9 conscious, 0–0.1 unconscious [A] |
| Search camera | a void or bored hole and line of sight | confirmation tool, not area search | P 0.3–0.7 given void access [A] |
| Canine | scent path; handler; rest | no rubble accuracy study found. Terrestrial field study: effectiveness 62.9%, one dog false-alerted ~every 100 m. Standards allow 0–1 false alerts in certification | [S] [Greatbatch 2015](https://journals.sagepub.com/doi/10.1016/j.wem.2015.02.009); P 0.5–0.85, false alert 0.05–0.25 per empty sector [A] |
| UWB radar life detector (GSSI LifeLocator) | hand-placed on the pile, ~stationary | breathing to ~15–18 ft (4.5–5.5 m), motion ~20–30 ft (vendor, trade press); TRx example 7 m; TRx now "no longer available" | [S] [Fire Engineering](https://www.fireengineering.com/technical-rescue/gssi-lifelocator-teex/), [GSSI](https://geophysical.com/products/lifelocator-trx) |
| Thermal camera | surface or near-surface | no through-rubble capability | general |

- **Unconscious victims:** 28% of injured Bam survivors were unconscious at rescue [S]. A device that waits for a tap cannot find them, by
  design [S, Firehouse]. That leaves dogs and radar, and both are limited: dogs by duty cycle and scent paths, radar by being hand-carried one point at a time.
- **At Weiguan** the army brought 24 teams with breathing-detection devices [S] ([Taipei Times](https://www.taipeitimes.com/News/front/archives/2016/02/07/2003639002)),
  plus thermal imagers and 3D laser scanners. The machinery-led phase also used life detectors and dogs to check each layer before the
  machines took it away [S, Chiu].

## 4. Weiguan Jinlong (6 Feb 2016, 03:57, M6.4)

| Item | Value | Tag / source | Conflicts |
|---|---|---|---|
| Building | 9 blocks (A–I), 16 storeys + 1 basement, U-shaped, RC; ~92 households | [S] [Tainan FB](https://w3fs.tainan.gov.tw/001/Upload/137/epaper/337/消防-0206震災搶救-電子報-0222(結合搶救科)%20.pdf) | "17 storeys" in BBC/AP |
| People in building | **309** residents (76 uninjured, 118 injured, 115 dead) | [S] [Pan 2019a](https://scholars.lib.ntu.edu.tw/handle/123456789/604741) | Tainan FB "300+"; zh.wiki "~200" (stale) |
| Extricated by rescuers | 290 (incl. 1 body on 18 Feb) | [S] Tainan FB | 289 (SETN, 13 Feb) |
| Rescued alive | **175** (164 on day 1, 7 on day 2, 4 on day 3) | [S] Tainan FB; [S] Chiu | 211 (Taipei Times, 9 Feb, early count) |
| Last live rescues | Lee Tsung-tien, Chen Mei-ri, girl Lin Su-chin at ~62 h; last one 8 Feb 18:15 (**62.3 h**) | [S] Tainan FB, Chiu | |
| Unaccounted end of day 2 | ~84 | [S] Chiu | Mayor: 131 still to be rescued early on |
| Deaths | **115** (13 day 1, 20 day 2, most after machinery started; peak 12 Feb) | [S] Tainan FB, Pan, Chiu | 114 in early counts; 117 for the whole quake |
| Heavy machinery | staged by 36 h; "big steel teeth" demolition from 64 h; demolition done 14 Feb 03:00 after **192 h**; >400 person-times of technical staff, 1,000 machine trips, 2,000 debris truckloads | [S] Chiu; [Tainan Public Works](https://w3fs.tainan.gov.tw/001/Upload/137/epaper/338/月份定稿版_工務局0206地震電子報.pdf) | |
| Last resident found | 13 Feb 15:57 (180 h); search declared over 13 Feb (mayor) / 14 Feb 14:30 (Tainan FB); one non-resident's body found 18 Feb | [S] [zh.wikipedia](https://zh.wikipedia.org/zh-tw/維冠金龍大樓), [Nikkei](https://web.archive.org/web/20160214085405/http://asia.nikkei.com/Politics-Economy/Policy-Politics/Search-for-Taiwan-quake-victims-ends-with-death-toll-at-116), Tainan FB | the two end dates differ by a day |
| Rescuers | 1,236 from outside Tainan incl. 840 army, 6 helicopters, **23 dogs** (day-1 snapshot); ~600 Tainan professional + volunteer firefighters; **~2,000 total** | [S] AP via [The Quint](https://www.thequint.com/amp/story/news/world/building-collapses-as-strong-earthquake-hits-taiwan-3-dead); total [D] | army 800 / 810 / 840 in different reports |
| International | Japan sent an expert team (JDR), the only foreign team on site. USA, Singapore and Israel offered USAR teams; **declined** (confined site, domestic capacity enough) | [S] Tainan FB; [Focus Taiwan](https://focustaiwan.tw/news/asoc/201602070014.aspx) | team size not verified |
| Collapsed area | 13,142 m² | [S] Tainan FB | 13,412 (Chiu): digit swap |
| Difficulty locating | H and I blocks pancaked onto A and G; access only by ladders and light tools; burst 2,000 mm water main flooded the basement; I and H kept leaning | [S] Chiu; Public Works | |

**Alive but found too late?** We found **no autopsy or time-of-death data** for the 115 dead, so we cannot say how many were alive for a while.
The circumstantial facts are these: about 84 people were unaccounted for at 48 h, 4 of them were found alive, and machinery started at 64 h only after crews had re-checked every reachable void.
Treat any claim that "N victims could have been saved" as unsupported.

**Taiwan context.**
- **Chi-Chi 1999:** 37 international teams made **6** live rescues [S] ([Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)).
  Taipei's Tunghsing Building killed 87, and two brothers were pulled out alive after ~130 h by layer-by-layer machine removal [S] ([Wikipedia](https://en.wikipedia.org/wiki/Tunghsing_Building), [CommonWealth](https://www.cw.com.tw/index.php/article/5106698)) *(single interview source for the 130 h)*.
- **Hualien 2024:** 18–19 dead in the whole quake. The 9-storey Uranus building tilted about 45° and ~23 people were rescued from it. Most of the people "trapped" were in Taroko gorge, not in buildings [S]
  ([Wikipedia](https://www.wikipedia.org/wiki/2024_Taiwan_earthquake)) *(news-level sources)*.

## 5. Costs

| Item | Value | Tag / source |
|---|---|---|
| ISAR team deployment | US$0.24 M (UK, Nepal, excluding transport) to US$6.2 M (US, Nepal); US Haiti 2010 US$35 M for 47 lives | [S] [Rom & Kelman Table 3](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/) |
| Cost per life saved by ISAR | US$0.79 M (UK), 0.94 M (US), 0.93 M combined; UK Haiti review: £250k per life against £2.5k for surgical teams | [S] same |
| Cost per rescuer-hour, international, all-in | **US$100** (40–250) = deployment cost / (60–75 people × 10–16 days × 24 h) | [D] |
| Cost per rescuer-hour, US domestic | US$46–90: BLS mean firefighter wage $29.03/h × 1.6–2.2 for benefits and overtime | [D] from [BLS](https://www.bls.gov/oes/2023/may/oes332011.htm) |
| Cost per rescuer-hour, Taiwan | US$10–25 | [A] not sourced |
| FEMA readiness funding | ~US$1.2 M per task force per year (true cost > $2 M) | [S] [Rep. Mullin letter](https://kevinmullin.house.gov/wp-content/uploads/2025/05/USR-Letter-2025.pdf) |
| Heavy machinery | FEMA 2023 rates: excavator 1.5 cy **$100.52/h**, 2.5 cy **$162.85/h** (no operator); demolition shear with operator $150–400/h | [S] [FEMA rates](https://eec.ky.gov/Environmental-Protection/Waste/recycling-and-local-assistance/Documents/FEMA%20Rates%207-26-23.pdf); shear [A] |
| VSL, US | **US$13.7 M** (DOT, 2024 base year) | [S] [US DOT](https://www.transportation.gov/office-policy/transportation-policy/revised-departmental-guidance-on-valuation-of-a-statistical-life-in-economic-analysis) |
| VSL, Taiwan | **~US$5 M** (1–13.7) by benefit transfer (income ratio ~0.39, elasticity 1). Local studies are old (NT$21–34 M, 1984 data, [Hsueh & Wang](https://ageconsearch.umn.edu/record/260344)) or an outlier (US$21.2 M, 2023 thesis) | [D]; local studies [S] *(weak)* |

**FEMA US&R:** no public per-day deployment rate. Each sponsor files a "Daily Cost Estimate" under 44 CFR 208 [S]
([CFR](https://www.govinfo.gov/content/pkg/CFR-2020-title44-vol1/pdf/CFR-2020-title44-vol1-sec208-2.pdf)).
We found **no Taiwan NFA deployment cost data**.

## 6. Logistics: charging drones from emergency vehicles

### Inputs

| Input | Value | Tag / source |
|---|---|---|
| Average car tank | 50 L (40–60). US cars 12–16 gal (45–61 L); EU family cars 45–55 L | [S] [Edmunds](https://www.edmunds.com/car-maintenance/how-many-gallons-of-gas-does-car-hold.html) *(no registry-based average found)*; Taiwan [A] |
| Half tank | 25 L (20–30) | [D] |
| Gasoline / diesel LHV | 31.2–32.4 MJ/L = 8.7–9.0 kWh/L / 35.8 MJ/L = 9.9 kWh/L | [S] [AFDC/GREET](https://webarchive.library.unt.edu/web/20161031223216/http://www.afdc.energy.gov/fuels/fuel_comparison_chart.pdf) |
| Alternator efficiency | 50–62% average, 65% max; whole on-board chain ~10–21% while driving | [S] [bus study](https://yadda.icm.edu.pl/baztech/element/bwmeta1.element.baztech-b402dc84-3863-464f-a643-d9743b7f2978), [Delco Remy](https://www.delcoremy.com/Documents/High-Efficiency-White-Paper.aspx) |
| Car idle fuel | 0.16 gal/h (0.61 L/h); 0.29 gal/h (1.1 L/h) at raised idle | [S] [Argonne](https://sandbox4-www.anl.gov/sites/www/files/2018-02/idling_worksheet.pdf) |
| Car alternator net output for charging at idle | 1.0 kW (0.6–1.6) | [A] |
| Fire-truck alternator | 320–430 A at 12 V ≈ 4.5–6 kW; 63–90% of rated output at idle | [S] [Delco Remy flyer](https://wwwtest.delcoremy.com/getmedia/0208588a-1f2f-429c-92e9-4c3a19a707a8/Firetruck-Flyer-4-16.pdf.aspx), [Leece-Neville](https://www.fleetmaintenance.com/equipment/emissions-and-efficiency/product/21234756/prestolite-electric-leeceneville-idlepro-and-idlepro-extreme-alternators) |
| Rescue-truck onboard generator | 8 kW hydraulic (most popular), 25 kW PTO (common on heavy rescues, e.g. Houston), up to 40 kW PTO; NFPA 1901 requires none | [S] [Rigspot](https://rigspot.com/?p=44884), [Houston](https://www.svitrucks.com/wp-content/uploads/2016/05/Houston-884.pdf) |
| Inverter generator | Honda EU2200i: 1.8 kW for 3.2 h on 3.6 L = 1.6 kWh/L (1.0 at ¼ load); DJI D12000iE: **500 ml/kWh DC at 9 kW = 2.0 kWh/L** | [S] [Honda](https://hondanews.com/en-US/power-equipment/releases/release-60f8b86b82a14607a3e13e208c572efc-honda-eu2200i-super-quiet-series-generator-technical-specifications), [DJI T50](https://ag.dji.com/t50/specs) |
| FlyCart 30 | 2 × DB2000 (1,984.4 Wh, 38 Ah, 52.22 V, 11.3 kg, max 5.7 kW charge); hover 29 min empty (65 kg), 18 min with 30 kg (95 kg); payload 30 kg dual / 40 kg single battery | [S] [DJI specs](https://www.dji.com/flycart-30/specs) |
| C8000 charger | 7.2 kW (3.6 kW on single input); ~26 min for one pack, ~36 min for two; needs a ≥ 8 kW 220 V single-phase generator; D12000iE not supported | [S] [DJI](https://www.dji.com/flycart-30/specs), [Heliguy](https://www.heliguy.com/products/dji-c8000-intelligent-battery-station/), [DJI FAQ](https://www.dji.com/ae/flycart-30/faq) |
| Agras T50 | DB1560 30 Ah × 52.22 V ≈ 1,567 Wh [D], 12.1 kg; 9–12 min charge on the C10000 (9 kW) or D12000iE; 40 kg spray payload, MTOW 92 kg | [S] [DJI T50](https://ag.dji.com/t50/specs) |
| Our flight power | **9.7 kW** (8.5–12). 3.57 kWh usable / 29 min = 7.4 kW at 65 kg; / 18 min = 11.9 kW at 95 kg; our 79 kg → 9.5–9.9 kW | [D] |

### Half a tank → flight minutes [D]

```
energy into battery = half_tank_L × kWh_per_L(path)
flight min          = energy × battery_round_trip / flight_power_kW × 60

Alternator path (car idling, 12 V → 52 V DC-DC charger):
  fuel at raised idle with ~1 kW net load = 1.1 L/h + 1.0 kW / (0.55 alt × 0.30 marginal engine) / 8.85 kWh/L = 1.78 L/h
  into battery = 1.0 kW × 0.90 charger / 1.78 L/h = 0.50 kWh/L   (range 0.31–0.91)
  central: 25 × 0.50 × 0.95 / 9.7 × 60 =  74 min   (low 20×0.31×0.93/12 → 29; high 30×0.91×0.97/8.5 → 186)

Inverter generator path (1.3–2.0 kWh DC per L; 2.0 = DJI D12000iE reference):
  central: 25 × 1.7  × 0.95 / 9.7 × 60 = 250 min   (low 20×1.3×0.93/12 → 121; high 30×2.0×0.97/8.5 → 411)
```

- **Alternator:** about **74 min (29–186)**, which is about 4 sorties of ~19 min. Turning 25 L into charge takes **~14 h of idling (11–18 h)**, because
  the car alternator delivers only ~1 kW. That rate is far too slow.
- **Generator:** about **250 min (121–411)**, which is about 13 sorties. The generator path gets 3.4× more flight from the same fuel, because an engine idling to turn a 1 kW
  alternator wastes most of its fuel.

### Recharge time per pack set (2 × DB2000, 20 → 100%, 3.18 kWh) [D]

| Source | Net charge power | Time per set | One set flies |
|---|---|---|---|
| Car alternator | 0.6–1.6 kW | **3.5 h** (2.1–6.2) | ~19 min |
| Fire-truck high-output alternator (surplus) | 1.5–4 kW | **1.4 h** (0.85–2.4) | ~19 min |
| Generator ≥ 8 kW + C8000 (7.2 kW) | 7.2 kW | **~30 min** (26–36 min) | ~19 min |

### Hub or truck retrofit? [D]

- One drone running continuously needs about **6.9 kW of charging power on average** (4.6–10.7). That assumes rotors are on for 50–80% of the time: hover and winch work need the rotors, landed listening does not.
  - **Car:** a car alternator supports about **0.13 drones** (0.05–0.33). It is an emergency top-up, not a power plant.
  - **Heavy rescue truck with a 25 kW PTO generator:** about **3.6 drones** (2.3–5.4), so 3 drones per truck, with one C8000 per drone.
    A truck with only an 8 kW hydraulic generator supports about 1 drone.
- **No city hub is needed.** Three drones on one heavy rescue truck draw about 20 kW. At an assumed 2.0–2.8 kWh/L for a truck diesel engine driving a PTO generator at light load [A],
  that burns **~7–12 L/h of diesel**, about 0.5–0.9 m³ over 72 h. That is ordinary fuel resupply at a USAR site.
  Keep 3 battery sets per drone (HARDWARE.md §12).
- **Acoustic caveat:** a truck engine running beside the pile raises the noise floor for our own listening phase.
  Park it ≥ 100 m downwind, or use the generator in a quiet window, as HARDWARE.md §13 already says.

## Biggest uncertainties

1. **The survival curve comes from two cities.** Kobe and Weiguan provide all the window data we have, and both are biased by selection. The popular "91/81/36.7/33.7/19/7.4" curve is untraceable.
2. **Detection probabilities are assumptions.** We found no published field sensitivity or false-alert rate for canines, listening devices or cameras on real rubble.
3. **Time split.** No clean percentage split exists. Ours comes from components: Haiti extrication times plus the Weiguan calibration.
4. **Unconscious share** rests on a single survivor survey (Bam, n=211), which undercounts people who were unconscious and died.
5. **Weiguan "found too late":** there is no time-of-death data, so it is unknowable from public sources.
6. **Energy:** the car's net alternator output at idle and the drone's real flight power (wind, winch hover) dominate the half-tank range.
   The DJI hover figures are vendor numbers measured in calm conditions at sea level.

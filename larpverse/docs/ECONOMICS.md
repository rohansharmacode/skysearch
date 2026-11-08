# Economics: what Larpverse costs, what it is worth, who buys it

Researched 8 Nov 2026. Tags follow [IMPACT_EVIDENCE.md](IMPACT_EVIDENCE.md): **[S]** sourced (the linked source states it),
**[D]** derived (our arithmetic, shown), **[A]** assumption (team judgement). Ranges are low / central / high. For costs, low
is the cheapest plausible case. For lives, low is the pessimistic case. Money is constant 2026 US$ unless stated, at NT$32 per US$ [A].
Machine-readable twin: [`data/economics_params.json`](../data/economics_params.json) (87 tagged parameters plus every computed
number below). This file extends [HARDWARE.md](HARDWARE.md) (BOM prices) and [IMPACT_EVIDENCE.md](IMPACT_EVIDENCE.md)
(survival, VSL, rescuer-hour and machinery rates) and does not repeat their tables.

## Judge summary

1. **A city kit costs about $0.67M up front and $2.8M over 10 years, or about $280k a year** (range $1.1–6.3M) [D].
   A kit is 3 heavy drones with winch pods, spares, chargers and a command van.
   That is roughly the yearly cost of one police drone-as-first-responder subscription ($140–435k) [S], or about 8.5 dog teams over the same 10 years [D].
   It is also about 14% of one FEMA task force's federal readiness money over 10 years ($1.99M a year) [D].
2. **The kit pays for itself if it saves a fraction of a life per decade at standard values of life.**
   At the US DOT figure of $13.7M per life it needs 0.20 lives per decade (0.08–0.46) [D], and at Taiwan's $5M (benefit-transfer) figure it needs 0.56 lives [D].
   At the cost-effectiveness bar that international USAR teams set today, US$0.93M per life saved, it needs about 3 lives (1.1–6.8) [D].
3. **Whether it actually saves those lives depends on the radar pod working in the field.** The impact model's upper bound assumes the pod pings every silent victim the radar can reach.
   In that case it adds about 5.7 survivors in a Weiguan-sized collapse (p5–p95: 0.6–13.9), so one such event per decade pays the kit back about 10× at Taiwan's value of life [D].
   The system as simulated today, with stub acoustics and no pod pings in the collapse run, adds about 0.2 survivors per 100 trapped. At that rate the kit needs about 300 trapped people per decade to break even at Taiwan's value of life [D].
4. **Labour savings are real but secondary.** At the upper bound it saves about 120 rescuer-hours per trapped person, about $0.8M per 100 trapped [D].
   Fewer digs at empty voids save about 120 rescuer-hours and 10 machine-hours per major site, only about $10k [D].
   The value of those hours is that they go to live voids sooner, not the money they save.
5. **The market is a niche inside a fast-growing one.** Public-safety drones were worth $2.6B in 2024 and are forecast to reach $5.8B by 2030 (14.5% a year) [S].
   We see about 400 candidate kits worldwide (150–800 [A]), worth about $1.1B over 10 years [D].
   Taiwan is the beachhead: it has a policy of buying drones without Chinese parts, the national fire agency plus 6 city search-and-rescue units to sell to, and two quake rebuilds that each cost more than NT$25B.
   The risks are the rules for drones over 25 kg, the exclusion of DJI (which forces the $87k non-DJI build), approval for the radar's emissions, and liability when a void reported empty was not.

---

## 1. What search costs today

### USAR teams

| Item | Value | Tag / source |
|---|---|---|
| FEMA US&R readiness money, all 28 task forces | **$55.6M** in FY2026 ($40.3M in FY2025, awards of $1.3–1.6M each) | [S] [FEMA advisory, Aug 2026](https://content.govdelivery.com/attachments/USDHSFEMA/2026/08/24/file_attachments/3756608/FEMA%20Advisory%20-%20FEMA%20Continues%20to%20Support%20National%20Urban%20Search%20and%20Rescue%20Task%20Forces%20%28August%2024,%202026%29.pdf), [HigherGov FY25](https://www.highergov.com/grant-opportunity/fiscal-year-2025-national-urban-search-and-rescue-response-system-readiness-coop-360153) |
| Per task force, federal share | **$1.99M/yr** (1.3–2.0M); sponsor agencies pay the rest; the true cost is above $2M | [D]; [S] [Mullin letter](https://kevinmullin.house.gov/wp-content/uploads/2025/05/USR-Letter-2025.pdf) |
| What the system did in 2025 | 7 disasters, more than 5,000 buildings searched, more than 200 rescues (mostly floods and storms, not collapses) | [S] FEMA advisory |
| Equipment cache per task force | $1.4M and about 27 t (FEMA 2008); about $4.7M (Utah TF-1); about $7M (FEMA report to Congress, 2018); Texas TF-2 cache is more than 45 t | [S] [FEMA 2008](https://webarchive.library.unt.edu/eot2008/20081024175231/http://www.fema.gov/emergency/usr/equipment.shtm), [UT-TF1](https://www.uttf1.org/participation), [report to Congress](https://kevinmullin.house.gov/wp-content/uploads/2024/11/USR-Final-Report-to-Congress-transmitted-6-05-18.pdf), [TX-TF2](https://texastaskforce2.org/cache/) |
| INSARAG Heavy team | at least 59 people (directory range 43–80), 3–8 dogs, 2 worksites, 10 days; FEMA Type 1 has 70 people. 60 INSARAG-classified teams worldwide at the end of 2025 | [S] [Springer table](https://jhumanitarianaction.springeropen.com/articles/10.1186/s41018-022-00116-z/tables/1), [INSARAG 2025 overview](https://insarag.org/wp-content/uploads/2026/02/Annual-Overview-2025.pdf) |
| Real deployments | UK ISAR in Türkiye 2023: 77 people. US in Türkiye 2023: 2 teams, 161 people, 12 dogs | [S] [GOV.UK](https://www.gov.uk/government/news/uk-sends-life-saving-support-to-earthquake-hit-turkey-and-syria), [USAID fact sheet](https://uatweb.usaid.gov/sites/default/files/2023-04/2023-03-07_USG_Turkiye_and_Syria_Earthquakes_Fact_Sheet_10.pdf) |
| International deployment | $0.24–6.2M per deployment; **about $20k–600k per team-day** (central $250k) | [S] [Rom & Kelman 2020](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/); per day [D] over 10–16 days on site |
| Domestic collapse, Surfside 2021 | FEMA obligated $16.9M for search & rescue and operations; Miami-Dade claimed more than $26.2M | [S] [GAO](https://www.gao.gov/assets/d24106558.pdf), [Miami-Dade](https://documents.miamidade.gov/mayor/memos/01.06.22-Report-Outlining-The-Costs-Associated-With-The-Surfside-Tragedy-Directive-No-211658.pdf) |

**Taiwan.** We found **no published budget line for the NFA's special search-and-rescue team** [S, absence]. For scale, Kaohsiung's fire bureau alone spends NT$3.47B a year
([Kaohsiung FB](https://orgws.kcg.gov.tw/001/KcgOrgUploadFiles/310/relfile/68107/65849/38d08cb8-4cfc-4bc2-93b3-4b68d357365f.pdf)).
International missions are paid by the 賑災基金會 (disaster relief foundation), not the NFA. Other facts:
- The central special SAR team has about 73 posts in its establishment table [D, truncated table]
  ([table](https://www.laws.taipei.gov.tw/lawatt/Law/A040040010016000-20231115-1000-001.pdf)).
- Taiwan has 31 IRO-certified search dogs across the NFA and 8 local governments, and 16 teams passed the Mission Ready Test in November 2025 [S]
  ([PTS](https://news.pts.org.tw/article/495211), [Newtalk](https://newtalk.tw/news/view/2025-11-29/1007152)).
- 16,913 firefighters were in service against 21,907 budgeted posts in 2023 [S] ([TPP/MOI](https://tpp.org.tw/en/our_platform-detail.php?id=37)).
- Taiwan had about 2,569 fire engines at the end of 2020 [S] ([Statista/MOI](https://statista.com/statistics/936469/taiwan-number-of-fire-engines)).

### Canine teams

| Item | Value | Tag / source |
|---|---|---|
| Cost to field one certified dog-and-handler team | $35k (task-force cost: courses $4k, certification $6k, dog) to about $75k (SDF, including training) | [S] [TX-TF2](https://texastaskforce2.org/position/canine-search-specialist/); [SDF via Wikipedia](https://en.wikipedia.org/wiki/National_Disaster_Search_Dog_Foundation) *(weak)* |
| Time to certification | about 1 year (6–8 months of dog training) | [S] [EMS1](https://www.ems1.com/search-and-rescue/from-shelter-dogs-to-disaster-search-specialists) |
| Working life | 6–9 years; recertification every 3 years | [S] EMS1 *(secondary)* |
| Upkeep training | 2 sessions a week of 3+ h = **312 handler-hours a year** | [D] from EMS1 |
| Care (food, vet, insurance) | $2–6k a year; dog insurance about $37 a month; Pingtung budgets only NT$135k a year for dog upkeep | [A]; [S] [udn](https://udn.com/news/story/7327/9432188) |
| **10-year cost of one dog team** | **$325k** ($222–424k): start-up + 312 h × $46–90/h + care + 0.1–0.7 replacement dogs | [D] |

### Technical search equipment (per unit)

| Device | Price | Tag / source |
|---|---|---|
| Delsar LD3 (6 sensors + victim simulator kit) | $42,979 catalogue (a used kit on eBay for $1,850) | [S] [Safeware](https://www.safewarecontracts.com/itemdetail/SAV%206020-01-016) |
| SearchCam 3000 search camera | $17,869 – $44,193, depending on the kit | [S] [Allsafe](https://allsafeindustries.com/searchcam-3000-search-rescue-camera-kit.aspx), [Safeware](https://www.safewarecontracts.com/itemdetail/SAV%206000-11-001) |
| LifeLocator-class UWB radar | no public price (quote only; TRx discontinued); $20–80k | [A]; [GSSI](https://www.geophysical.com/products/lifelocator-trx) |

A typical set of listening device, camera and radar costs about $100–170k [D]. One Larpverse pod costs $5.6k in parts [S, BOM].
The pod does not replace these tools. It carries radar and microphones to many points without a person on the pile.

## 2. What disasters cost and what a life is worth

| Measure | Value | Tag / source |
|---|---|---|
| US DOT VSL | **$13.7M** (2024) | [S] [DOT](https://www.transportation.gov/office-policy/transportation-policy/revised-departmental-guidance-on-valuation-of-a-statistical-life-in-economic-analysis) |
| OECD 2025 base VSL | $7.1M (OECD members), **$8.4M (EU-27)**, $8.5M (US); 7.7–8.7M GDP-weighted. To transfer to another country, scale by the ratio of GDP per head | [S] [OECD](https://www.oecd.org/en/publications/mortality-risk-valuation-in-policy-assessment_76ca89a2-en/full-report/recommended-vsl-estimates-for-policy-analysis_9b41dca3.html) |
| EU transport handbook | €4.3M per fatality (2019 Handbook as used by EUROCONTROL) | [S] [ANS CBA](https://ansperformance.eu/economics/cba/standard-inputs/chapters/value_of_a_statistical_injury.html) *(weak)* |
| **Taiwan: the method actually used** | MOTC counts the **social cost** of a road death at about **NT$16M (≈ $0.5M)** and of an injury at about NT$1M (2014 data). This is a cost-of-injury figure, not willingness to pay. We found no official Taiwan VSL | [S] [Newtalk / MOTC](https://newtalk.tw/news/view/2018-01-25/111980) |
| Taiwan VSL used in this repo | **$5M** (1–13.7) by benefit transfer. A 2023 thesis gives $21.2M (outlier) | [D] IMPACT_EVIDENCE; [S] [NCCU thesis](https://ah.lib.nccu.edu.tw/handle/140.119/146858) |
| ISAR cost per life saved | $0.79–0.94M | [S] [Rom & Kelman 2020](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/) |
| Tainan 2016 (Meinong) | reconstruction budget NT$25B; one loss estimate NT$13.9B *(weak)*; public donations **NT$4.27B** | [S] [Wikipedia](https://en.wikipedia.org/wiki/2016_southern_Taiwan_earthquake), [NTNU](https://www.es.ntnu.edu.tw/wp-content/uploads/2024/11/林威廷.pdf), [udn](https://udn.com/news/story/7326/9311715) |
| Hualien 2024 | Executive Yuan reconstruction **NT$28.55B** (NT$18.44B for infrastructure); insurable loss $0.5–1B (CoreLogic); about NT$3B county damage | [S] [Focus Taiwan](https://focustaiwan.tw/politics/202404190008), [AIR](https://www.asiainsurancereview.com/News/View-NewsLetter-Article/id/87810/Type/eDaily), [Taipei Times](https://www.taipeitimes.com/News/feat/archives/2024/05/13/2003817775) |
| **Weiguan compensation** | 128 plaintiffs claimed NT$5.0B and were awarded **NT$450M** (final; NT$350M damages + NT$100M punitive). The consumer foundation's class action won NT$710M. The defendants hold few assets. **State-compensation suits against Tainan were dismissed.** Donations paid **NT$4M per death**, and up to NT$4M per Weiguan owner (NT$256M to 127 owners) | [S] [PTS](https://news.pts.org.tw/article/491257), [Taiwan News](https://www.taiwannews.com.tw/zh/news/3858554), [PTS](https://news.pts.org.tw/article/523911), [Tainan donation ledger](https://w3fs.tainan.gov.tw/001/Upload/177/attachment/19541/4378732/0206震災捐款專戶截至1080831收支情形表.pdf) |

What paid out at Weiguan (about NT$4M ≈ $125k per death from donations [D]) is far below any VSL.
Compensation is not a measure of what society would pay to avoid the death, so the break-even below uses VSL, with MOTC's social cost as a deliberately low floor.

## 3. What our system costs

### Kit definition and capex

A **city kit** has 3 drones, each with its payload, winch and pod. It also includes:
- 3 battery sets per drone (IMPACT_EVIDENCE §6);
- one C8000-class charger per drone;
- spare pods, a spare winch and airframe spares;
- the van fit-out from the BOM: mesh mast, satellite/5G link, laptop, server, UPS, two generators and a tent;
- a vehicle, integration and initial training.

| Line (US$) | Low: FC30, truck retrofit | **Central: H6HL** | High: H6HL + thermal | Tag |
|---|---|---|---|---|
| 3 drones incl. payload + winch + pod | 182,301 | **260,901** | 284,901 | [D] BOM totals: 60,767 / 86,967 / 94,967 each |
| 2 spare battery sets per drone | 33,600 | 38,400 | 48,000 | [S] DB2000 $2.8k / [A] H6HL set $6.4–8k |
| Van fit-out (BOM $51k minus spare packs) | 34,200 | 34,200 | 45,000 | [D] BOM |
| Chargers, 1 per drone | 12,000 | 15,000 | 21,000 | [A] |
| Spare pods ($5,567 each) + spare winch | 8,067 | 19,201 | 21,001 | [A] count, [S] BOM price |
| Airframe spares (5 / 8 / 12% of drones) | 9,115 | 20,872 | 34,188 | [A] |
| **Hardware at cost** | 279,283 | **388,574** | 454,090 | [D] |
| × vendor markup (assembly, test, warranty, margin) | ×1.15 | ×1.35 | ×1.8 | [A] |
| Vehicle | 0 (existing truck) | 70,000 | 110,000 | [A] |
| Integration, acceptance, radar/EMC approval | 10,000 | 30,000 | 80,000 | [A] |
| Initial training (6 / 8 / 12 operators) | 30,000 | 48,000 | 96,000 | [A] |
| **Capex per kit** | **361k** | **673k** | **1.10M** | [D] |

### Annual operating cost

| Line (US$/yr) | Low | **Central** | High | Basis |
|---|---|---|---|---|
| Maintenance | 16,059 | **52,458** | 122,604 | 5 / 10 / 15% of hardware price [A] |
| Battery replacement | 16,800 | **28,800** | 72,000 | 9 sets, each replaced every 3 / 2 / 1 years. The DB2000 is rated for **1,500 cycles or 12 months** [S] ([Heliguy](https://www.heliguy.com/blogs/knowledge-base/what-is-the-battery-life-of-an-intelligent-flight-battery-for-the-dji-flycart-30/)). A training kit flies only about 100–200 cycles a year, so calendar ageing wears the packs out first [D] |
| Insurance | 14,579 | **31,177** | 66,539 | hull 6 / 8 / 12% of aircraft value [S] ([SkyWatch](https://www.skywatch.ai/blog/understanding-drone-hull-insurance-protect-uav-investment), [NMSU](https://safety.nmsu.edu/uas-operations/getting-started/uas-requirements.html)) + liability $2–5k [A] |
| Recurrent training | 9,600 | **48,000** | 144,000 | 6–12 operators × 40–120 h × $40–100/h [A]; Taiwan wages of $10–25/h [A] would cut this line about 4× |
| Software, agent, updates | 10,000 | **25,000** | 50,000 | [A] |
| Connectivity + exercise consumables | 3,200 | **7,400** | 14,000 | [A] |
| **Opex per kit** | **70k** | **193k** | **469k** | [D] |

### 10-year total cost of ownership per city kit

TCO₁₀ = capex + 10 × opex + mid-life refresh.
The refresh is none in the low case, 50% of the aircraft value at year 6 in the central case (compute, LiDAR, motors), and full replacement at year 5 in the high case [A].

| | Low | **Central** | High |
|---|---|---|---|
| Mid-life refresh | 0 | 176k | 513k |
| **TCO₁₀ (undiscounted)** | **$1.06M** | **$2.78M** | **$6.31M** |
| TCO₁₀, present value at 3% real ([OMB A-4](https://obamawhitehouse.archives.gov/omb/circulars_a004_a-4/)) | $0.96M | $2.46M | $5.55M |
| Per year | $106k | **$278k** | $631k |

In Taiwan, the same central kit drops to about **$2.4M** (≈ NT$78M) once operator hours are priced at local wages [D].
Retrofitting it onto an existing heavy-rescue truck with a PTO generator also removes the $70k van [A, IMPACT_EVIDENCE §6].

### TCO comparison (10 years)

| Capability | 10-year cost | What you get | Tag |
|---|---|---|---|
| **Larpverse city kit** | **$2.8M** (1.1–6.3) | 3 drones that map, listen, call and drop a radar pod; nobody on the pile; works around the clock | [D] |
| One canine team | $325k (222–424k) | 1 dog; 20–45 min on, then equal rest; no published rubble accuracy | [D] |
| Kit ÷ canine team | **≈ 8.5 dog teams** | | [D] |
| One FEMA Type 1 task force, federal readiness funding only | **$19.9M** (13–20M) + cache $1.4–7M | 70 people, 4+ dogs, about 27–45 t of kit, extrication capability | [D] / [S] |
| Kit ÷ task force | **≈ 14%** of its federal readiness money | | [D] |
| Technical search set (listening device + camera + radar) | about $0.1–0.17M to buy + upkeep | hand-carried, one point at a time | [D] |
| One international heavy-team deployment | $0.24–6.2M per event | arrives after 12–56 h | [S] Rom & Kelman |

**The kit complements a task force; it does not replace one.** Extrication takes about 64% of worksite time (IMPACT_EVIDENCE §2), and no drone does it.
The fair comparison is with the search component: dogs, listening devices and radar. Against that the kit costs about the same as the canine and technical-search line of one Heavy team.

## 4. ROI and break-even

### Lives needed per decade

Break-even lives = TCO₁₀ ÷ value per life [D]:

| Value per life | Low TCO $1.06M | **Central TCO $2.78M** | High TCO $6.31M |
|---|---|---|---|
| US DOT VSL $13.7M | 0.08 | **0.20** | 0.46 |
| OECD/EU VSL $8.4M | 0.13 | **0.33** | 0.75 |
| Taiwan VSL $5M (benefit transfer) | 0.21 | **0.56** | 1.26 |
| ISAR cost per life saved $0.93M | 1.14 | **2.99** | 6.78 |
| Taiwan MOTC social cost $0.5M (floor) | 2.13 | **5.55** | 12.6 |

### Lives the impact model says we save

Source: [`data/impact_results.json`](../data/impact_results.json), collapse site, snapshot of 8 Nov 2026 01:50.
These figures come from `sim/impact.py`, which is still uncommitted, uses placeholder parameters and ran with stub acoustics. **Recompute when it changes.**
The model has two arms. *Ideal* is the upper bound: the pod pings every silent victim the radar can reach in the sectors it is sent to.
*Ours* is the system as simulated today; the collapse run produced 0 pod pings.

| Arm | Extra survivors per 100 trapped [median, p5, p95] | Per Weiguan-scale collapse (290 trapped) |
|---|---|---|
| Ideal (upper bound) | 0.75 [−1.82, 2.72] | **5.67 [0.61, 13.87]** |
| Ours as simulated | 0.18 [−0.14, 0.68] | 0.0 [0.0, 1.59] |

Trapped people a kit must reach per decade to break even at the central TCO [D]:

| Value per life | Ideal arm | Ours as simulated | Weiguan-scale events per decade (ideal) |
|---|---|---|---|
| US DOT $13.7M | 27 | 113 | 0.04 |
| Taiwan $5M | 74 | 309 | **0.10** |
| ISAR $0.93M | 398 | 1,659 | 0.53 |
| MOTC $0.5M | 741 | 3,086 | 0.98 |

We assume a regional kit reaches 100 trapped people per decade (20–400 [A]); Weiguan alone had 290 extricated.
On that basis the *ideal* arm clears the VSL bar easily. The *ours* arm clears it only at the US VSL or above roughly 300 trapped people per decade.
**The economic case is the radar pod's case.** If the field pod finds silent victims about as well as the ideal arm assumes, one Weiguan-sized collapse a decade pays for the kit about 10× at Taiwan's value of life.
If it does not, the kit is a mapping, comms and labour-saving tool whose value per life is unproven.

### Rescuer-hours and machinery

- **Rescuer-hours [D]:** per trapped person, the model's baseline is 785 h. The ideal arm needs 664 h, saving **121 h**, and ours needs 778 h, saving 6.6 h.
  At $68/h (US, [D] IMPACT_EVIDENCE) that is worth **$0.82M per 100 trapped** in the ideal arm and $45k in ours.
  At the central assumption of 100 trapped per decade, the ideal arm's labour saving alone covers about 30% of the TCO.
- **Digs at empty sites avoided [A, anchored on the Haiti 2 h empty dig, [S] [Fire Engineering](https://www.fireengineering.com/technical-rescue/california-task-force-rescues/)]:**
  10 digs (3–30) × 2 h (1–4) × 6 rescuers, with half the dig-hours on a machine at FEMA rates of $100–163/h
  ([FEMA rates](https://eec.ky.gov/Environmental-Protection/Waste/recycling-and-local-assistance/Documents/FEMA%20Rates%207-26-23.pdf)).
  That comes to **120 rescuer-h + 10 machine-h ≈ $9.8k per major site** ($0.6k–134k).
  In money this is small. The benefit is that crews and machines are free for live voids hours earlier, and that negative information supports the call to start machinery (Weiguan: 64 h).
- **Running cost of a deployment:** about $19.8k per 72 h ($12–31k), from the impact model's `drone_cost_72h_usd` [D].
  For comparison, an international heavy team costs about $250k per team-day [D].

### Chart to draw (data in `computed` of the JSON)

**Break-even chart.**
- The x-axis is extra lives saved per kit per decade on a log scale, 0.01–30.
- The y-axis is net 10-year benefit in US$M, equal to lives × value per life − TCO₁₀.
- Draw one line per value per life: $13.7M, $5M, $0.93M and $0.5M. Each line crosses zero at its break-even from the table above.
- Draw a grey band for the TCO range ($1.06–6.31M).
- Mark two vertical bands for the expected lives per decade at 100 trapped: *ours* 0.18 (−0.03 to 2.7) and *ideal* 0.75 (−0.4 to 10.9).
- Mark one Weiguan-scale point: ideal 5.67.

The reader should see that the ideal band lies right of every VSL break-even. The *ours* band straddles them.

## 5. Market and adoption

### Size

| Measure | Value | Tag / source |
|---|---|---|
| Public-safety drone market | **$2.6B (2024) → $5.8B (2030), 14.5% CAGR**; other publishers give $1.1B (2023) → $2.0B (2028), 13%, up to $5.3B (2024) | [S] [GII/R&M](https://m.giikorea.co.kr/report/go1737376-public-safety-drones.html), [MarketsandMarkets](https://marketsandmarkets.com/market-reports/public-safety-drone-market-52227848.html), [MRFR](https://marketresearchfuture.com/reports/public-safety-drone-market-33317) |
| Drone-as-first-responder segment | $1.8B (2025) → $8.7B (2034) | [S] [MarketIntelo](https://marketintelo.com/report/drone-as-a-first-responder-market) *(weak)* |
| US fire departments | 29,452 (82% all or mostly volunteer) | [S] [NFPA via FirefighterNation](https://www.firefighternation.com/?p=74705) |
| USAR teams | 60 INSARAG-classified, about 33 of them Heavy *(weak)*; 28 FEMA task forces | [S] INSARAG 2025, FEMA |
| Taiwan | NFA special SAR team + 6 municipalities' special SAR brigades; 22 local fire bureaus; about 2,569 fire engines | [S] (§1) |
| **Our serviceable market** | **about 400 kits** (150–800): INSARAG + FEMA + national, state and municipal heavy-rescue teams in seismic countries | [A] |
| SAM value | capex **$269M**, opex **$77M/yr**, 10-year **$1.1B** (central kit price) ≈ 2–4% of the 2030 public-safety drone market per year | [D] |
| Taiwan first market | 4 kits (north/central/south/east pool) → 7 (NFA + 6 cities) → 23 (every county) | [A] |

At 4 kits, a Taiwan regional pool costs about **NT$36M a year** [D]. That is about 0.1% a year of the Hualien 2024 reconstruction budget, or about 1% of the Tainan 2016 donations.

### Procurement pathways

- **Taiwan:**
  - Buying routes are tenders under the Government Procurement Act, NFA and municipal fire budgets, and special post-quake budgets (Hualien NT$28.55B [S]).
  - Since 2020 the Executive Yuan has banned Chinese ICT products in agencies, and drone tenders now require supply chains without Chinese parts ("non-red") [S].
    In 2024, the Ministry of Digital Affairs pushed fire units to shelve DJI drones above a price threshold [S]
    ([Taipei Times](https://www.taipeitimes.com/News/taiwan/archives/2024/12/15/2003828539), [Focus Taiwan](https://focustaiwan.tw/sci-tech/202605190019)).
    The non-DJI H6HL-class build, or a Taiwanese heavy-lift airframe from the drone national team, is therefore the only sellable configuration.
- **US:**
  - The FEMA **Assistance to Firefighters Grant** had $324M appropriated in FY2024 ($291.6M awarded); only "High"-priority equipment is considered [S]
    ([FEMA](https://fema.gov/sites/default/files/documents/fema_fy2024_afg_key_changes.pdf)). Vendors list drones as eligible *(weak)*.
  - The **US&R readiness cooperative agreement** ($55.6M) funds equipment that is on the 44 CFR 208 cache list, so a line item would have to be added to that list [S].
  - **BRIC** was terminated in 2025 and ruled unlawful in December 2025, and a $1B round reopened in April 2026 [S]
    ([Smart Cities Dive](https://www.smartcitiesdive.com/news/FEMA-BRIC-states-court-order/812462/)). BRIC funds mitigation infrastructure, so **search drones probably do not qualify** [A].
- **EU:** the Union Civil Protection Mechanism has €3.32B for 2021–2027, including rescEU, whose capacities are fully EU-financed [S]
  ([Commission](https://commission.europa.eu/funding-and-tenders/find-funding/eu-funding-programmes/union-civil-protection-mechanism-resceu_en)).
  A rescEU USAR-support capacity or Horizon pilot is the realistic route [A].
- **Retrofit:** heavy-rescue trucks with 25 kW PTO generators can power 3 drones (IMPACT_EVIDENCE §6). Selling the kit as a truck module avoids buying a vehicle and fits fleet replacement cycles [A].

### Comparable products and prices

| Product | Price | What it is | Tag / source |
|---|---|---|---|
| DJI Dock 3 + Matrice 4TD | $23.4–26.4k (US dealers, Sep 2026) | drone-in-a-box, 1 small drone | [S] [DroneBundle](https://dronebundle.com/blog/dji-dock-3) |
| Skydio X10 + Dock | $50,547 list (Dock $36k), up to $62k with support | DFR dock, small drone, US-made | [S] [DroneXL](https://dronexl.co/drone-companies/skydio/) *(weak)* |
| BRINC Responder DFR program | **$140k–435k a year per city**; Victorville $832k over 3 years; Newport Beach $2.18M over 5 years for 7 drones; often first year free | subscription with hardware, software, batteries, training | [S] [DroneXL](https://dronexl.co/2026/04/27/brinc-drones-victorville-832k/), [Gov1](https://gov1.com/public-safety/calif-pd-to-deploy-911-response-drones-under-new-2-17m-contract), [Porterville](https://www.recorderonline.com/news/ppd-drone-program-approved/article_bdf9a228-2d98-41f9-9eb5-07f22985a56f.html) |
| Flock Aerodome DFR 2.0 | about $114k a year per system (Everett: $457k a year for 4) | subscription | [S] [robotics.press](https://robotics.press/news/dfr-program-costs-2026/) *(weak)* |
| **Larpverse kit** | **capex $673k / about $278k a year all-in** | 3 heavy drones, winch radar pods, listening and calling, hotspot, van | [D] |

Our yearly cost sits inside the price band US cities already pay for DFR, but the kit does a different job: heavy-lift, sensing under rubble, and rare deployment.
That has two consequences. It is a regional asset, not a precinct asset. And it needs dual use (§6) to keep crews current and the budget defensible.

### Business models

1. **Kit sale + service contract.** Sale at about $0.67M plus a yearly service fee of about 15–20% of capex, covering batteries, repairs, software and recertification. This mirrors Skydio and Axon. It fits capital grants such as AFG and post-quake special budgets.
2. **Subscription per region (preferred for Taiwan).** About $0.3M a year per kit, all-in, priced from the TCO plus margin.
   The vendor owns the hardware and refreshes it, and the agency supplies operators. The model matches how BRINC and Flock sell DFR, so procurement offices know it.
3. **Regional pools.** One kit per region, with staging time limited by road transport. Taiwan with 4 kits costs about $11M over 10 years.
   US states could share a kit across task forces, as Florida TF-1 and others share caches.
4. **Activation fee (drones-as-a-service in disasters).** A standby retainer plus about $20k per 72 h deployed [D].
   This is attractive for countries with rare large events and for international teams (INSARAG) that want the capability without owning it.
   The weakness is that a crew that has not trained with the kit performs badly. The retainer must include exercises.

## 6. Dual use beyond earthquakes

The kit's parts (LiDAR mapping, listening, loudspeaker calls, Wi-Fi hotspot, radar pod) also apply to other incidents. Dual use is what keeps a rarely used regional asset funded and its crews current.

| Incident | What transfers | Evidence / frequency | Tag |
|---|---|---|---|
| Non-earthquake building collapse | everything | India alone: **1,435 structure collapses and 1,525 deaths in 2024**; 38,363 deaths 2001–15. No global count exists (EM-DAT keeps a "collapse" subgroup) | [S] [Dataful/NCRB](https://insights.dataful.in/articles/four-states-account-for-half-of-indias-building-collapse-deaths) |
| Industrial collapse | everything | Rana Plaza 2013: 1,134 dead | [S] [Wikipedia](https://en.wikipedia.org/wiki/2013_Dhaka_garment_factory_collapse) |
| Gas explosions | mapping, listening, pod | Kaohsiung 2014: 32 dead | [S] [Wikipedia](https://en.wikipedia.org/wiki/2014_Kaohsiung_gas_explosions) |
| Landslides, typhoon damage | mapping, hotspot (phones), loudspeaker; the pod only on debris shallow enough for its 4.5–7 m radar reach | Taiwan typhoon season every year; mountain rescue teams at 12 new stations from 2026 | [A]; [S] [Taipei Times](https://www.taipeitimes.com/News/taiwan/archives/2026/01/24/2003851145) |
| Avalanche / wilderness | phone detection: the hotspot is a weaker cousin of Lifeseeker, which runs a fake cell tower | Lifeseeker: more than 35 operators in more than 10 countries, more than 200 missions, about 5 m accuracy, phone must be on; one buried victim found in under 10 min after 8 h of visual search | [S] [HeliHub](https://www.helihub.com/2024/06/03/lifeseeker-data-shows-how-sar-uses-phone-signals-to-find-missing-people/), [UST](https://www.unmannedsystemstechnology.com/2023/06/lifeseeker-completes-over-100-successful-sar-missions-in-one-year/) |
| Context | | Disasters 2000–2019: 7,348 events, 1.23M deaths, $2.97T in losses; earthquakes and tsunamis were the deadliest. FEMA US&R in 2025: 7 disasters, more than 5,000 buildings searched | [S] [UNDRR](https://www.undrr.org/publication/human-cost-disasters-2000-2019), FEMA |

Adding a Lifeseeker-style cellular payload (a licensed emergency base station) would make phone detection a standard feature. Price unknown [A].

## 7. Honest risks

1. **Heavy-UAS regulation.**
   - US Part 107 covers drones under 55 lb (25 kg) only [S] ([14 CFR 107](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-107)). Our 70–95 kg aircraft need public-aircraft COAs or a 44807 exemption and emergency waivers to fly over people and beyond line of sight.
   - In the EU, SAR and firefighting flights are excluded from Regulation 2018/1139 and fall under national rules [S] ([EUR-Lex](https://eur-lex.europa.eu/eli/reg/2018/1139/oj)). That helps agencies but means 27 national regimes.
   - Taiwan's CAA rules for drones of 25 kg and above require registration and inspection [A, verify].
   - Budget 6–18 months to certify the first kit in each country [A].
2. **DJI supply chain.**
   - New DJI models cannot get FCC authorisation since the December 2025 Covered List action ([HARDWARE.md](HARDWARE.md)).
   - A 100% US tariff on imported thermal drones and docks is reported but unconfirmed *(weak)* ([DroneBundle](https://dronebundle.com/blog/dji-dock-3)).
   - Taiwan agencies cannot buy Chinese ICT [S].
   - The cheap FC30 build ($61k per drone) is therefore a simulation reference only. The sellable build is the $87k H6HL class, about +$78k per kit before markup. That is already the central case.
3. **Radar certification.** The pod is an SDR sweeping 0.4–2 GHz.
   - US UWB imaging under 47 CFR 15.509 is limited to law enforcement, fire and rescue users and needs equipment authorisation [S] ([eCFR](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-F/section-15.509)).
   - Taiwan needs NCC type approval [A].
   - A software radio does not hold that approval by default, so it needs a certified fixed configuration or an experimental licence. This is the long pole for the whole economic case, because the ROI rests on the pod.
4. **Liability.**
   - A sector the system scores as low-priority, where a survivor later dies, is the nightmare case.
   - Mitigation: the system never says "clear" or "confirmed", only candidate search regions (RULES rule 9); humans decide; the system logs everything.
   - Weiguan shows courts put collapse liability on the builder, and claims against the city were dismissed [S]. That says nothing about a search tool vendor, so product liability insurance is needed [A] and is not in the opex above.
5. **Evidence risk.** Today's impact model shows a small benefit for the system as simulated (0.18 per 100 trapped) and a large one only in the ideal arm. Judges should see both.
   Field trials, such as Disaster City / TEEX or the NFA training centre in Nantou, are needed before any agency signs.
6. **Use-it-or-lose-it.** Big collapses are rare. Without dual use and monthly exercises, operator skill decays, and the kit sits in a garage while its batteries age by calendar.

## Gaps

- No NFA special-SAR budget line, no Taiwan fire-station count, and no LifeLocator price was found.
- INSARAG Heavy equipment tonnage is not published in the guideline text we reached; we use the FEMA caches instead (24–45 t).
- No global count of non-earthquake building collapses; India's NCRB series is the best national series found.
- The H6HL price ($50k) and every kit line marked [A] are budget guesses. Get quotes before using them in a tender.

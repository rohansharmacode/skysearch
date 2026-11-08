# Case studies: what happened in real collapses

Researched 8 Nov 2026 for the Larpverse defence. This file **extends** [IMPACT_EVIDENCE.md](IMPACT_EVIDENCE.md)
(survival curves, USAR time split, sensor limits, Weiguan facts, costs, charging). Weiguan, Kobe's survival-by-day curve and
the Rom & Kelman cost data are summarised there and only cross-referenced here.
The machine-readable version is [`data/case_studies.json`](../data/case_studies.json) (same numbers, low / central / high where uncertain).

Tags: **[S]** stated by the linked source. **[D]** our arithmetic on sourced numbers (shown). **[A]** team assumption.
*(weak)* = single news report, Wikipedia-only, vendor claim, or a link we could not open ourselves.
Hours are counted from the main shock. "Rescued alive" means extricated alive, not survived to discharge.

---

## 1. Summary table

| Case | Deaths | Trapped (est.) | Rescued alive | Last live rescue (h) | Search (days) | International USAR | Notes |
|---|---|---|---|---|---|---|---|
| **Chi-Chi, Taiwan 1999** (M7.3) | 2,415 [S] (+29 missing) | n/a | n/a nationally; int'l teams 6 [S] | ~130 (Tunghsing brothers) [S] | n/a | 37 teams [S], >700 people from >20 countries [S]; first at 18 h [S] | 51,711 buildings fully collapsed [S] |
| **Tunghsing Bldg, Taipei 1999** | 87 [S] | n/a | 2 brothers late [S] | ~130 [S] (Taipei Times: "6 days") | ≥6 | Japan/others | found by layer-by-layer machine removal (IMPACT §4) |
| **Weiguan Jinlong, Tainan 2016** | 115 [S] | 309 residents [S] | 175 [S] | 62.3 [S] | ~8 [S] | Japan expert team only; US/SG/IL declined [S] | full detail in IMPACT §4 |
| **Hualien 2018** (M6.4) | 17 [S] (14 Yun Men Tsui Ti, 2 Marshal) | n/a | n/a | ≥15 (Marshal receptionist) [S] | 4.4 (106 h at YMTT) [S] | Japan 7 people, body-heat detectors, arrived 48–72 h [S/D] | 4 buildings partly collapsed [S] |
| **Hualien 2024** (M7.2–7.4) | 18–19 [S] (1 at Uranus) | ~600 stranded (mostly Taroko, not rubble) [S] | 24 from Uranus bldg [S] | same day (Uranus) | Taroko search ≥6 d [S] | Turkey 7-person drone team (Taroko) [S] | missing couple not found by phone signal [S] |
| **Turkey (Kahramanmaraş) 2023** (M7.8 + 7.5) | 53,537 TR [S] (+5,951–8,476 Syria [S]) | n/a nationally; ~800 in one complex [S] | >8,000 by day 2–3 [S]; int'l ~300 [S] | **296** [S] | 13 (most provinces) [S] | 90 teams / ~5,000 people (INSARAG) [S] vs 255 teams (AFAD) [S]; 49 INSARAG-classified [S] | 38,901 buildings collapsed [S]; search in 26,000 buildings [S] |
| **Nepal 2015** (M7.8) | 8,962 [S] | n/a | int'l 16 [S] | ~168 (101-yr-old, "7 days") [S/D] | n/a | 76 teams [S], first <12 h [S] | NASA/DHS FINDER radar located 4 men [S] |
| **Haiti 2010** (M7.0) | 100,000–316,000 [S] (disputed) | n/a | int'l 132–150 [S] | ~360 (Darlene Etienne, 15 d) [S/D] | 11 [S] | 43–67 teams [S], 1,739 rescuers + 161 dogs on day 6 [S] | 250,000 homes + 30,000 commercial bldgs collapsed or severely damaged [S] |
| **Wenchuan 2008** (M7.9) | 69,227 [S] (+17,923 missing) | n/a | int'l 2 [S] | 196 (Pengzhou woman) [S] | n/a | 6 teams [S], first >72 h [S] | ≥7,000 school buildings collapsed [S]; >100,000 troops [S] |
| **Kobe 1995** (M6.9) | 6,434 [S] | ~35,000 needed rescue [S] | 27,100 by citizens + 7,900 by FD/police [S] | n/a | n/a | Swiss team only (25 people, 12 dogs), held in quarantine [S] | 95–98% rescued by self / family / neighbours [S] |
| **Marmara (İzmit) 1999** (M7.6) | 17,127–18,373 [S] | ">10,000 trapped or missing" on day 1 [S] *(weak)* | int'l 144 [S] | n/a | ~7 [S] *(weak)* | 92 teams [S], first at 16 h [S] | ≥60,434 buildings collapsed [S] *(weak)* |
| **Rana Plaza, Bangladesh 2013** (collapse) | 1,134 [S] | 3,122 workers inside [S] | 2,438 pulled out [S] (~2,500 [S]) | ~390 (Reshma, "17 days") [S/D] | 19–20 [S] | INSARAG help **rejected** [S] | heavy machinery only from day 5 [S] |
| **Surfside, Florida 2021** (collapse) | 98 [S] | up to 159 unaccounted at day 1 [S] | 4 from rubble (1 died) [S] + ≥35 from the standing part [S] | <12 ("initial hours") [S] | 14 rescue + 16 recovery = 29 [S] | 2 FEMA TFs + Israel + Mexico [S] | 304 drone missions, 0 found alive after day 1 [S] |
| **Mexico City 2017** (M7.1) | 369–370 national [S], 228 CDMX [S] | n/a | 69 in CDMX by day 4 [S] | n/a | n/a | n/a | 38 buildings collapsed in CDMX [S] (44–52 early) |
| **Morocco 2023** (M6.8) | 2,946 [S] | n/a | int'l ≈0 reported [S] *(weak)* | n/a | n/a | only 4 countries accepted [S]; 100 teams / 3,500 people registered and waiting [S] | rural masonry villages |
| **Myanmar 2025** (M7.7) | 3,785–3,835 [S] | n/a | 653 by emergency teams [S] | ~120 ("5 days") [S/D] | n/a | 2,095 personnel from 26 countries [S] | Sky Villa: 52 alive, 103 dead [S] |

n/a = we found no source. Conflicting figures are listed per case below.

---

## 2. Per-case sections

### 2.1 Chi-Chi (921) earthquake, Taiwan — 21 Sep 1999, 01:47

| Item | Value | Tag / source |
|---|---|---|
| Deaths / missing / injured | 2,415 / 29 / 11,305 | [S] [Wikipedia](https://en.wikipedia.org/wiki/1999_Jiji_earthquake) |
| Buildings | 51,711 fully collapsed, 53,768 severely damaged | [S] same |
| International | 37 teams, **6 live rescues**, first team at 18 h | [S] [Rom & Kelman 2020, Table 2](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/) |
| International (other count) | >700 rescuers from >20 countries; US 85, Japan 100+, Russia 76, Turkey 15 | [S] [Wikipedia](https://en.wikipedia.org/wiki/1999_Jiji_earthquake) |
| Cost | NT$300 bn (US$10 bn), ~10% of GDP | [S] same |

Timeline:
- **+50 h:** Turkey's AKUT team rescued a woman in Changhua [S] ([PTS](https://news.pts.org.tw/article/715861)).
- **+86–88 h:** a Korean team pulled 6-year-old Chang Ching-hung from the Wang Chao building in Taichung. He was the only one of five family members to survive [S] ([PTS](https://news.pts.org.tw/article/715861); Wikipedia gives 84 and 88 h).
- **~+130 h:** two brothers came out of the Tunghsing Building in Taipei (87 dead) [S] ([Wikipedia](https://en.wikipedia.org/wiki/1999_Jiji_earthquake)). The [Taipei Times](https://www.taipeitimes.com/News/taiwan/archives/2009/09/21/2003454083) says "six days" (~144 h). **Conflict:** 130 vs ~144 h.

Gaps: we found **no national count of people trapped or rescued alive**. The NFA and MND reports exist but are not online in searchable form.

### 2.2 Weiguan Jinlong, Tainan — 6 Feb 2016

Covered in full in [IMPACT_EVIDENCE §4](IMPACT_EVIDENCE.md#4-weiguan-jinlong-6-feb-2016-0357-m64): 309 residents, 175 alive (164 / 7 / 4 by day), 115 dead, last live rescue at 62.3 h,
machinery from 64 h, ~2,000 rescuers and 23 dogs, and foreign USAR offers declined. Nothing new found that changes those numbers.

### 2.3 Hualien — 6 Feb 2018, 23:50 (Marshal Hotel, Yun Men Tsui Ti)

| Item | Value | Tag / source |
|---|---|---|
| Deaths / injured | 17 / 285 (14 in Yun Men Tsui Ti, 2 in the Marshal Hotel) | [S] [Wikipedia](https://en.wikipedia.org/wiki/2018_Hualien_earthquake) |
| Buildings | 4 partly collapsed or severely damaged; YMTT (12 storeys) tilted 30° → 45° during day 1 | [S] same; [Taipei Times](https://www.taipeitimes.com/News/front/archives/2018/02/08/2003687280) |
| Japan | 7-person team, accepted for its **body-heat detection equipment**, arrived 9 Feb (48–72 h [D]) | [S] [Wikipedia](https://en.wikipedia.org/wiki/2018_Hualien_earthquake) |

Timeline:
- **~+4 h:** Chen Ming-hui was freed from basement level 1 of the 11-storey Marshal Hotel [S] ([Taipei Times](https://www.taipeitimes.com/News/front/archives/2018/02/08/2003687282)).
- **~+15 h:** receptionist Liang Shu-wei was freed near the lobby [S] (same).
- **by 8 Feb (≤48 h):** the county declared rescue at the Marshal Hotel complete [S] (same).
- **10–11 Feb:** 4 of a family of 5 were found dead in Room 201 of YMTT. The last 2 were pinned under a main beam that could not be removed without dropping the building [S] ([SETN](https://www.setn.com/news/347089), [NTD](https://www.ntdtv.com/gb/2018/02/11/a1362641.html)).
- **11 Feb morning (~106 h):** manual search stopped and machines took the building down from the top [S] ([NTD](https://www.ntdtv.com/gb/2018/02/11/a1362641.html)).

For us: this is the same end-game as Weiguan. Search by hand stops when the structure is too unstable, and then machinery has to dig without knowing where the remaining people are.

### 2.4 Hualien — 3 Apr 2024, 07:58

- 18–19 dead in the whole quake. Most of the people "trapped" were in Taroko Gorge (rockfall, tunnels), not in collapsed buildings [S] ([Wikipedia](https://en.wikipedia.org/wiki/2024_Hualien_earthquake), see IMPACT §4).
- **Uranus building** (9–10 storeys; first floor pancaked, tilted ~45°): **24 people rescued**, several by cherry picker from upper floors [S] ([WAM](https://www.wam.ae/en/article/b2hjxit-twenty-four-trapped-people-rescued-from-severely)). One woman died after going back in for her cat [S] (news). **Conflict:** 23 (IMPACT §4) vs 24.
- **Drones:** the NFA used drones and helicopters to search Taroko from day 1 [S] ([Al Jazeera](https://www.aljazeera.com/news/2024/4/4/taiwan-says-1000-injured-in-earthquake-rescue-efforts-focus-on-hualien)).
  A **7-person Turkish drone search team** arrived on 6 Apr (~3 days) [S] ([Taiwan News](https://www.taiwannews.com.tw/en/news/5137584)).
- **Phone location failed:** search teams "still could not locate" a missing Singaporean couple through their mobile phone signal on day 6 [S] ([TVBS](https://news.tvbs.com.tw/amp/english/2449120)).

### 2.5 Turkey–Syria (Kahramanmaraş) — 6 Feb 2023, 04:17 (M7.8) and 13:24 (M7.5)

| Item | Value | Tag / source | Conflicts |
|---|---|---|---|
| Deaths | **53,537** Turkey; 5,951–8,476 Syria | [S] [Sputnik Türkiye](https://anlatilaninotesi.com.tr/20240205/6-subat-depreminde-kac-kisi-oldu-neler-yasandi-kac-bina-yikildi-depremin-siddeti-son-depremler-1080416968.html), [Wikipedia](https://en.wikipedia.org/wiki/2023_Turkey%E2%80%93Syria_earthquakes) | 50,783 (Kızılay Akademi); 55,537 (typo in the same Sputnik piece) |
| Collapsed buildings | **38,901** "flattened the same day"; 518,000 homes destroyed or heavily damaged | [S] Sputnik Türkiye | 164,000 bldgs destroyed or severely damaged by 23 Feb (Wikipedia) |
| Buildings searched | **26,000** | [S] Sputnik Türkiye | |
| Trapped | no national estimate. Rönesans Rezidans (Antakya) alone: ~800 trapped, ≥269 dead | [S] [Wikipedia](https://en.wikipedia.org/wiki/2023_Turkey%E2%80%93Syria_earthquakes) | unofficial estimates of 80,000–180,000 are not credible *(weak)* |
| Rescued alive | Hatay 1,846 by 7 Feb; **>8,000 in 10 provinces** by 8 Feb | [S] [Wikipedia (aftermath)](https://en.wikipedia.org/wiki/Aftermath_of_the_2023_Turkey%E2%80%93Syria_earthquakes), [Diken](https://www.diken.com.tr/enkazdan-kurtarilanlarin-sayisi-8-binden-fazla/) | **no final national total found**. 108,068 "rescued injured" (Health Ministry) counts a different thing. Malatya fire brigade alone: 600 alive at 89 sites [S] ([Sözcü](https://www.sozcu.com.tr/6-subat-depremlerinde-600-kisiyi-enkazdan-sag-olarak-cikardilar-p20493)) |
| International live rescues | **~300** by ~5,000 rescuers in **90 teams** | [S] [INSARAG AAR page](https://insarag.org/aar-turkiye/) | "115 survivors" quoted for one coordination centre (TRT) |
| International teams | **49** INSARAG-classified; **255** classified + non-classified (AFAD figure); the UN USAR Coordination Cell coordinated **118** | [S] [INSARAG Future Directions](https://www.insarag.org/wp-content/uploads/2024/04/Future-Directions-of-INSARAG.pdf) | 90 teams (AAR page) |
| SAR personnel | 60,217 total incl. 3,251 foreign (Interior Ministry, 24 Apr 2023) | [S] [TİP report](https://tip.org.tr/wp-content/uploads/2024/06/depremin_8._ayina_iliskin_rapor.pdf), Sputnik | 14,740 (day 1); 24,727 incl. 5,709 foreign (~day 10); ">141,000 people from 94 countries" (government, includes aid workers) |
| Cost | US$103.6 bn (TL 2 trn) | [S] [SBB](https://www.sbb.gov.tr/2023-kahramanmaras-ve-hatay-depremleri-raporu/) | US$150 bn (Wikipedia) |

**Live-rescue timeline (selected, Turkey):**

| h | Event | Source |
|---|---|---|
| ~12 | Regional coordination cell set up in Adana | [S] [INSARAG FD](https://www.insarag.org/wp-content/uploads/2024/04/Future-Directions-of-INSARAG.pdf) |
| ~44 [D] | 1,846 rescued in Hatay as of 7 Feb | [S] [Wikipedia](https://en.wikipedia.org/wiki/Aftermath_of_the_2023_Turkey%E2%80%93Syria_earthquakes) |
| ~50–70 | >8,000 rescued across 10 provinces | [S] same; Diken |
| ~50–55 [D] | Twitter restricted on the main mobile networks, while survivors were using it to send their addresses | [S] [The Record / NetBlocks](https://therecord.media/turkey-twitter-blocked-earthquake/) |
| 103 | Azerbaijani team rescues a 15-year-old in Kahramanmaraş | [S] [Al Jazeera](https://www.aljazeera.com/news/2023/2/10/miracle-rescues-saved-after-100-hours-under-rubble-in-turkey) |
| 198 | 3 people rescued | [S] [L'Orient](https://today.lorientlejour.com/article/1328252/three-people-rescued-in-turkey-198-hours-after-earthquake.html) |
| 222 | woman rescued | [S] [Gulf News](https://gulfnews.com/world/mena/a-woman-rescued-from-ruins-in-turkey-222-hours-after-powerful-quake-1.93861388) |
| 248 | Aleyna Ölmez (17) rescued | [S] [CNN](https://www.cnn.com/2023/02/16/europe/turkey-syria-earthquake-rescue-efforts-intl/index.html) |
| 261 | 2 men rescued | [S] [ABC](https://www.goodmorningamerica.com/news/story/turkey-earthquake-2-men-rescued-rubble-261-hours-97280487) |
| 278 | Hakan Yasinoğlu rescued (Hatay) | [S] [Al Jazeera](https://www.aljazeera.com/news/2023/2/18/turkey-rescuers-find-three-people-13-days-after-quake) |
| **296** | **2 adults and a child rescued in Antakya; the child died later** | [S] [NTV](https://www.ntv.com.tr/turkiye/hatayda-296-saatte-enkazdan2-kisi-sag-cikarildi,s7Ou77FfN0q1wHZJYr-k4A) |
| ~13 d (19 Feb) | AFAD ends search and rescue in most provinces; work continues at 40 buildings | [S] [Wikipedia](https://en.wikipedia.org/wiki/Aftermath_of_the_2023_Turkey%E2%80%93Syria_earthquakes) |

**Conflict:** Sputnik Türkiye calls a 185 h rescue (Hüseyin Berber) the "last known survivor". That contradicts the dated 248–296 h reports above, so we use 296 h.

**Phones, social media and silence:**
- "Hundreds of thousands" of calls for help were posted on social media in the first hours. Volunteers mapped them for rescuers, for example on afetharita.com [S] ([Euronews](https://www.euronews.com/2023/02/10/how-twitter-helped-find-survivors-trapped-beneath-rubble-after-turkeys-earthquakes)).
  afetharita had 35 M requests and 627,000 unique visitors (web traffic, not help messages) [S] ([dev.to, developer](https://dev.to/erayg/how-an-open-source-disaster-map-helped-thousands-of-earthquake-survivors-afetharitacom-440)) *(weak)*.
  The project itself declines to claim "we saved X people" [S] (Euronews). An academic study of the tweeted addresses found many could not be identified [S] ([dergipark](https://dergipark.org.tr/en/pub/tucbis/article/1316898)).
- Some trapped survivors live-streamed pleas or posted their location, and rescuers reached them that way [S] ([Wikipedia](https://en.wikipedia.org/wiki/Aftermath_of_the_2023_Turkey%E2%80%93Syria_earthquakes)).
- **Silence periods:** crews stopped generators and machines, called "Sesimi duyan var mı?" ("Can anyone hear me?"), and waited [S] ([AP via News4Jax](https://www.news4jax.com/news/world/2023/02/08/in-earthquake-rescues-noisy-gear-and-digging-then-silence/)).
  The Konya fire team leader explained why: the acoustic devices pick up even the movement of a small stone, so the area must be quiet. Crews ask the trapped person to **scrape a fingernail** against something so it can be heard [S] ([Al-Ain](https://tr.al-ain.com/article/enkaz-arama-kurtarma-ekipler)).
  This is the whole method, and it only works on conscious victims.
- **Heard, but nobody came:** in Hatay, people said trapped relatives "were still making sounds while nobody approached them" and "we made several calls, but no one came" [S] ([Arab News](https://www.arabnews.pk/node/2249346), [SCMP](https://www.scmp.com/news/world/middle-east/article/3209413/turkeys-worst-quake-hit-area-desperate-screams-heard-rubble)).
  President Erdoğan acknowledged "shortcomings" [S] (Wikipedia aftermath).

**Drones (Turkey 2023):**
- Bayraktar TB2 / Akıncı flew >1,551 h of damage detection and coordination; ~50 UAVs were on duty [S] ([Baykar](https://baykartech.com/en/press/bayraktar-tb2-aknc-uavs-conduct-over-1551-hours-of-flights-in-quake-zone/)) *(vendor)*.
- Xtend and Polaris thermal drones in Hatay [S] ([JPost](https://www.jpost.com/middle-east/article-731686)) *(vendor)*. BRINC LEMUR indoor drones, 12–17 Feb, used mainly to identify remains [S] ([BRINC](https://brincdrones.com/?p=3784)) *(vendor)*.
- **No source gives a count of survivors located by drone.**

### 2.6 Nepal (Gorkha) — 25 Apr 2015, 11:56

| Item | Value | Tag / source |
|---|---|---|
| Deaths / injured | 8,962 / 21,952 (8,857 in Nepal) | [S] [Wikipedia](https://en.wikipedia.org/wiki/April_2015_Nepal_earthquake) |
| International | 76 teams, **16 live rescues**, first team <12 h | [S] [Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/) |
| Damage | US$10 bn (~50% of GDP) | [S] Wikipedia |

Timeline:
- **>80 h:** Rishi Khanal freed from a 3-storey hotel in Kathmandu by French and Nepalese rescuers [S] ([NBC](https://www.nbcnews.com/storyline/nepal-earthquake/nepal-earthquake-survivor-pulled-rubble-after-four-days-n350106)).
- **Day 5 (~120 h [D]):** teenager Pemba Tamang pulled from a 7-storey building by a Nepalese team and the USAID USAR team [S] ([CBS](https://cbsnews.com/news/nepal-earthquake-survivor-teen-boy-pemba-tamang-rescued-rubble-kathmandu)).
- **Day ~7 (~168 h [D]):** a 101-year-old man pulled from his house [S] ([Inquirer](https://newsinfo.inquirer.net/?p=689100)).
- **FINDER radar:** the DHS/NASA-JPL microwave radar detected **two heartbeats under each of two collapsed buildings** in Chautara, and 4 men were dug out from under up to ~3 m of rubble [S] ([DHS](https://www.dhs.gov/archive/science-and-technology/news/2015/05/05/finder-helps-save-four-nepal), [JPL](https://www.jpl.nasa.gov/news/dhs-and-nasa-technology-helps-save-four-in-nepal-earthquake-disaster/)).
  Lab spec: through 30 ft (9 m) of rubble or 20 ft (6 m) of solid concrete [S] (JPL). It was a hand-placed ground unit, and **the closest real-world precedent for our belly radar**.
- **Drones:** the Humanitarian UAV Network (UAViators), Kathmandu University and others mapped damage, for example in Panga, where 578 of 1,334 buildings were uninhabitable. The civil aviation authority refused some of their applications [S] ([Drones in Humanitarian Action case study](https://zoinet.org/wp-content/uploads/2018/01/3Case-Study-Nepal.14April2016.pdf)). This was mapping, not victim search.

### 2.7 Haiti — 12 Jan 2010, 16:53

| Item | Value | Tag / source | Conflicts |
|---|---|---|---|
| Deaths | 100,000–160,000 (scholarly); government 220,000–316,000 | [S] [Wikipedia](https://en.wikipedia.org/wiki/2010_Haiti_earthquake) | 46,000–85,000 (unreleased USAID draft) |
| Buildings | 250,000 homes + 30,000 commercial buildings collapsed or severely damaged | [S] same | |
| Int'l teams on day 6 | **43 teams, 1,739 rescuers, 161 dogs** (USAID, 18 Jan) | [S] [USAID fact sheet](https://pdf.usaid.gov/pdf_docs/PA00J2B7.pdf) *(weak: usaid.gov offline Nov 2026; figure from search index)* | 67 teams (Rom & Kelman); ~60 (INSARAG) |
| Int'l live rescues | 70+ by 17 Jan ("a record", OCHA); **132** by 22 Jan, 47 of them by US teams | [S] [OCHA/ReliefWeb](https://reliefweb.int/report/haiti/rescuers-still-finding-survivors-haiti); USAID fact sheet *(weak)* | 134 (Rom & Kelman); ">130" ([INSARAG book](https://library.alnap.org/system/files/content/resource/files/main/printed-haiti-book-low-definition-version.pdf)); 150 ([Fire Engineering](https://www.fireengineering.com/?p=197952)) |
| First team | Iceland, <24 h; 23 h in Rom & Kelman | [S] | |
| Search ended | government called off the search on 23 Jan (day 11) | [S] Wikipedia | |
| Cost | US ISAR US$35 M for 47 lives (IMPACT §5) | [S] Rom & Kelman | |

- **Day 15 (~360 h [D]):** Darlene Etienne (16) was pulled out by a French team, after **neighbours heard her voice** [S] ([PBS](https://www.pbs.org/newshour/show/fifteen-days-below-rubble-final-quake-survivor-in-haiti), [NBC](https://www.nbcnews.com/id/wbna35086799)).
  That was 4 days after the official end of the search.
- A man claimed 27 days under rubble (Evans Monsigrace). Doctors could not verify it [S] ([NBC](https://www.nbcnews.com/id/wbna35333222)). Don't use it.
- Rescuers per live rescue, by day 10: 1,739 / 132 ≈ **13 international rescuers per life** [D].

### 2.8 Wenchuan (Sichuan) — 12 May 2008, 14:28

| Item | Value | Tag / source |
|---|---|---|
| Deaths / missing / injured | 69,227 / 17,923 / 374,643 (25 Sep 2008) | [S] [Sina / State Council](https://news.sina.cn/sa/2008-09-25/detail-ikftpnny4058155.d.html) |
| Buildings | ≥7,000 school buildings collapsed; ~80% of Beichuan county town | [S] [Wikipedia](https://en.wikipedia.org/wiki/2008_Sichuan_earthquake) |
| Schools | Juyuan, Dujiangyan: ~900 students buried, <60 survived | [S] same |
| Troops | >100,000 troops, medics and volunteers (Xinhua); >130,000 incl. foreign teams by day 6 | [S] [China Daily](https://covid-19.chinadaily.com.cn/china/2008-05/14/content_6684866.htm), Wikipedia *(weak: day-6 figure via search)* |
| International | 6 teams, **2 live rescues**, first >72 h | [S] [Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/) |
| Cost | ¥845.1 bn (US$130 bn) | [S] Wikipedia |

Timeline:
- **Day 1–2:** up to 600 PLA troops reached Wenchuan county on foot and pulled >1,000 people from debris [S] ([China Daily](https://covid-19.chinadaily.com.cn/china/2008-05/14/content_6682723.htm)). Advance troops could not do large-scale rescue because machinery could not follow [S] (Sina, 2008, via search) *(weak)*.
- **Day 5:** "at least 63 people pulled alive" that day [S] (news via search) *(weak)*.
- **~127 h:** a Russian team rescued a 61-year-old woman [S] (Mail & Guardian via search) *(weak)*.
- **179 h:** Ma Yuanjiang was **located on 18 May and freed on 20 May**, after a Shanghai team spent >30 h chiselling through 10 slabs [S] ([Oneindia/Xinhua](https://www.oneindia.com/2008/05/20/survives-quake-rubble-179-hours-1211268600.html)).
- **196 h:** a 60-year-old woman was rescued in Pengzhou [S] ([China Daily](https://covid-19.chinadaily.com.cn/china/2008-05/21/content_6699093.htm)).
- We found **no verified national total of people rescued alive** from rubble. The 1,486,407 "rescued and transferred" figure includes evacuations.

### 2.9 Kobe (Great Hanshin) — 17 Jan 1995, 05:46

- **Deaths:** 6,434 [S] (commonly cited figure; [GovTech retrospective](https://www.govtech.com/em/emergency-blogs/disaster-zone/20yearretrospectiveonthekobeearthquake.html)) *(weak attribution)*.
- **Trapped:** 164,000 trapped in some form. 129,000 got out unaided and **35,000 had to be rescued**: **27,100 (77.5%) by citizens** and **7,900 (22.5%) by firefighters or police** [S]
  ([Kobe City, ADRC 2015](https://web.adrc.asia/acdr/2015/documents/ID251_3.pdf), citing the Japan Assoc. for Fire Science & Engineering survey).
  The [FDMA](https://www.fdma.go.jp/relocation/e-college/cat66/cat62/cat55/2-25.html) says 95% were rescued by themselves, family or neighbours. Other sources give 98% (Cabinet Office via secondary) and ~80% (GovTech). Specialised rescue teams did **1.7%** [S] ([bousai-base](https://bousai-base.com/hanshin-awaji-daishinsai/)) *(weak)*.
- **Survival by day of extrication:** 80.5% → 28.5% → 21.8% → ~6% after 72 h (see IMPACT §1a).
- **International:** Japan accepted only a Swiss team (25 people, 12 dogs), which was then held in quarantine at Kansai airport [S] ([NPR, 21 Jan 1995](https://www.wkms.org/1995-01-22/rescue-dogs)). 652 people were still missing on 20 Jan [S] ([CS Monitor](https://www.csmonitor.com/layout/set/amphtml/1995/0120/20202.html)).
  The slow acceptance of help from the SDF and foreign USAR teams was criticised and reshaped Japan's disaster system [S] (same).

### 2.10 Marmara (İzmit / Gölcük) — 17 Aug 1999, 03:02

- **Deaths:** 17,127 (official, Oct 1999), 17,480 (later official) and 18,373 (AFAD). Some researchers estimate up to ~45,000 [S]
  ([Wikipedia](https://en.wikipedia.org/wiki/1999_%C4%B0zmit_earthquake), [Daily Sabah](https://www.dailysabah.com/feature/2015/08/17/turkey-mourns-loss-of-lives-on-commemoration-of-izmit-earthquake)).
- **Buildings:** ≥60,434 collapsed, 127,251 damaged (Sept 1999 reports) [S] (Wikipedia) *(weak; other counts are lower)*.
- **International:** **92 teams, 144 live rescues, first at 16 h** [S] ([Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)). A contemporary report said the first 4 days were "desperate but poorly coordinated" efforts by foreign teams and local volunteers [S] ([WSWS](https://www.wsws.org/en/articles/1999/08/turk-a24.pdf)) *(weak, partisan)*. Most teams left after about a week [S] *(weak)*.

### 2.11 Rana Plaza, Savar, Bangladesh — 24 Apr 2013, ~08:57 (non-earthquake collapse)

| Item | Value | Tag / source | Conflicts |
|---|---|---|---|
| People inside | 3,122 workers (BGMEA); ~5,000 employed | [S] [Wikipedia](https://en.wikipedia.org/wiki/2013_Savar_building_collapse) | |
| Deaths | **1,134** (final) | [S] Wikipedia | 1,127 at end of operation (incl. 12 in hospital) [S] [Daily Star](https://www.thedailystar.net/news/savar-rescue-operation-ends); 1,136 |
| Pulled out alive | **2,438** | [S] [Daily Star](https://www.thedailystar.net/news/savar-rescue-operation-ends) | ~2,500 (Wikipedia, NBC) |
| Missing at end | ≥98 | [S] Daily Star | |
| Search length | 24 Apr → 12/13 May: **19–20 days** | [S] Daily Star, Wikipedia | |
| Building | 9 storeys, the top 4 unpermitted | [S] Wikipedia | 8 storeys |

Timeline and failures:
- **Days 1–4:** firefighters and local volunteers did most of the digging, often with no equipment and in sandals [S] ([Daily Star](https://www.thedailystar.net/ill-equipped-for-rescue-57377), Wikipedia).
  **The fire service could not use its search camera because the gaps between pillars were too narrow and the voids too dark** [S] ([Daily Star](https://www.thedailystar.net/ill-equipped-for-rescue-57377)).
- **Day 5 (28 Apr):** heavy machinery was only then deployed, after being held back for fear of crushing survivors [S] ([Daily Star](https://www.thedailystar.net/news/rescuers-decide-to-deploy-powerful-cranes), [Dhaka Tribune](https://www.dhakatribune.com/feature/travel/25389/heavy-rescue-gears-kept-idle-fearing-risk)).
- **28 Apr (>100 h):** Shaheena was found alive behind a beam on the 3rd floor. After **>20 h** of tunnelling, a fire broke out, the team had to retreat, and she died. Rescuer Ezazuddin Kaikobad later died of his burns [S]
  ([Daily Star "Sorry Shaheena"](https://www.thedailystar.net/node/1093939), [Daily Star](https://www.thedailystar.net/news/burnt-rescuer-dies-in-spore)).
- **10 May, day 17 (~390 h [D]: 16 days from 08:57 is 384 h, plus the hours of 10 May):** Reshma Begum was found alive and rescued [S] (Wikipedia, [Dhaka Tribune](https://www.dhakatribune.com/bangladesh/bangladesh-others/26242/reshma-rescued-in-rana-plaza-%E2%80%98miracle%E2%80%99)).
  She had survived on rainwater and co-workers' lunches [S] ([Dawn](https://www.dawn.com/news/812473)) *(weak attribution)*. Some outlets alleged a hoax, which was not substantiated [S] (Wikipedia). Nobody else was found alive after 28 Apr.
- **International:** Bangladesh **rejected INSARAG assistance** [S] (Wikipedia).

### 2.12 Champlain Towers South, Surfside, Florida — 24 Jun 2021, 01:22 (non-earthquake collapse)

| Item | Value | Tag / source |
|---|---|---|
| Deaths | **98** (+ an unborn child); last victim identified 26 Jul | [S] [Wikipedia](https://en.wikipedia.org/wiki/Surfside_condominium_collapse) |
| Unaccounted | up to 159 on day 1 | [S] same |
| Rescued from rubble | **4**, one died soon after; ≥35 rescued on 24 Jun from the standing part / balconies | [S] same |
| Rescuers | >80 units on day 1; FL-TF1 and FL-TF2 (3 more FEMA TFs on standby); Israel Home Front Command and Mexico from ~25–27 Jun; 210 workers on site in 12 h shifts by 29 Jun | [S] same |
| Phases | search and rescue for **14 days** (to 7 Jul), recovery until **23 Jul** (29 days in total). Paused 15 h on 1 Jul (structure unsafe); standing part demolished 4 Jul | [S] same; [CBS](https://www.cbsnews.com/news/surfside-condo-collapse-search-ends) |
| Fire in the pile | burned until 29 Jun | [S] Wikipedia |
| Cost | FEMA obligations **US$106.9 M** by Jun 2023, of which US$16.9 M for search and rescue and other ops | [S] [GAO](https://www.gao.gov/assets/d24106558.pdf) *(link from search index)* |
| Cost (county) | US$31.7 M estimated (Jan 2022); US$54.9 M FEMA-approved (Sep 2023) | [S] [Miami-Dade](https://documents.miamidade.gov/mayor/memos/01.06.22-Report-Outlining-The-Costs-Associated-With-The-Surfside-Tragedy-Directive-No-211658.pdf), [Miami-Dade](https://www.miamidade.gov/govaction/legistarfiles/Matters/Y2023/232103.pdf) |
| Settlement | US$1.02 bn | [S] Wikipedia |

Failures and technology:
- **A woman's voice was heard until ~11:00 on day 1, but rescuers could not reach her** [S] ([Wikipedia](https://en.wikipedia.org/wiki/Surfside_condominium_collapse)).
- **Jonah Handler** (15) was found because a passer-by walking his dog **heard his voice and saw his arm** [S] ([WSVN](https://wsvn.com/news/local/jonah-handler-reunites-with-rescuers-who-pulled-him-from-rubble-moments-after-surfside-condo-collapse/)).
- **Drones:** **304 logged strategic missions** (tactical flights were not logged), thermal drones from the first night, and an orthomosaic on 1 Jul [S] ([IEEE Spectrum, Murphy et al.](https://spectrum.ieee.org/building-collapse-surfside-robots)).
  Drones also provided lighting [S] ([Firehouse](https://www.firehouse.com/tech-comm/drones/article/21260858/fire-technology-champlain-towers-south-collapse-drones-value-soars)).
- **Other tools:** sonar, sensitive microphones, a microwave-based detector, ground robots with thermal and 360° cameras, and dogs. **"No search and rescue teams anywhere have found anyone alive with a ground robot."**
  A ground robot costs US$50–150k, an aerial drone ~US$1k [S] ([Robin Murphy, The Conversation via Robohub](https://robohub.org/an-expert-on-search-and-rescue-robots-explains-the-technologies-used-in-disasters-like-the-florida-condo-collapse/)).

### 2.13 Mexico City (Puebla quake) — 19 Sep 2017, 13:14

- **Deaths:** 369–370 nationally, 228 in Mexico City [S] ([La Razón](https://www.razon.com.mx/ciudad/2018/09/19/interactivo-a-un-ano-del-sismo-que-sacudio-a-la-cdmx/)). On 23 Sep the count was 310 and 172 [S] ([Quadratín](https://mexico.quadratin.com.mx/ya-307-muertos-sismo-del-19-septiembre)).
- **Collapsed:** 38 buildings in CDMX (final) [S] (La Razón). Early counts were 44 (mayor, day 1) and 52 (Reuters) — they count partial collapses differently.
- **Rescued alive in CDMX:** 50 on day 1, **69 by 23 Sep** [S] (Quadratín).
- **Enrique Rébsamen school:** 19 children and 6 adults died; 11 children were rescued [S] (Reuters via search).
- **"Frida Sofía", a wrong target:** for ~1 day, national TV and the navy reported a girl trapped under a table. **She did not exist.**
  A camera lowered into the rubble had shown only blood tracks, and the navy apologised [S] ([CBC](https://www.cbc.ca/1.4301403), [Truthdig/AP](https://www.truthdig.com/articles/report-girl-trapped-school-rubble-false-navy-apologizes/)).
- **Frida the dog:** her life-saving credit ranges from "12 lives" over her career [S] ([Inquirer/AP](https://newsinfo.inquirer.net/1013832/hero-dog-that-rescued-12-lives-in-2017-mexico-quake-honord-with-statue)) to "52" (Guardian headline) to "didn't rescue anyone" in 2017 [S] ([NBC](https://www.nbcmiami.com/news/national-international/frida-the-rescue-dog-2/1939037/?amp=1)).
  Don't put a number on her; use her as a morale story only.

### 2.14 Morocco (Al Haouz) — 8 Sep 2023, 23:11

- **Deaths:** 2,946 by 14 Sep [S] ([EMPHNET/ReliefWeb sitrep](https://epinews.emphnet.net/en/news/morocco-earthquake-september-2023-situation-report-15092023)).
- **International:** Morocco accepted only **Spain, UK, Qatar and UAE**, announced on 10 Sep (~1.5–2 days after the quake [D]). **100 teams (3,500 people) were registered with the UN and waiting** [S] ([Al Jazeera](https://www.aljazeera.com/news/2023/9/12/is-morocco-accepting-aid-certain-countries-political-reasons)).
  The UK sent 60 specialists and 4 dogs [S] ([GOV.UK](https://www.gov.uk/government/news/uk-deploys-search-rescue-teams-to-morocco-following-earthquake)).
- **Outcome:** foreign crews "found few people to save" [S] ([CBC](https://www.cbc.ca/1.6963772)). One UK firefighter said "we didn't find anyone alive" [S] ([Sleaford Standard](https://sleafordstandard.co.uk/news/firefighters-tell-of-experiences-back-from-moroccan-quake-zone-4348829)) *(weak: covers their villages only)*.

### 2.15 Myanmar (Sagaing / Mandalay) — 28 Mar 2025, 12:50

- **Deaths:** 3,785 (with 5,105 injured and 92 missing, 4 May) [S] ([Eleven Myanmar](https://elevenmyanmar.com/node/302522)). **Conflict:** 3,835 [S] (Eleven) and 3,798 (Xinhua, 30 Apr).
  The USGS model had suggested >10,000 *(weak)*. These are junta state-media figures.
- **Rescued alive:** **653** by emergency teams (early May) [S] ([The Star](https://www.thestar.com.my/aseanplus/aseanplus-news/2025/05/02/653-survivors-saved-by-rescuers-in-myanmar-earthquake)). In Mandalay Region, 403 alive and 259 bodies [S] (Fire Services Dept via [Myanmar Now](https://myanmar-now.org/en/news/days-after-myanmar-quake-hopes-fade-for-trapped-missing-victims/)) *(weak attribution)*.
- **Sky Villa condo (Mandalay):** 52 rescued alive, 103 bodies [S] ([China Daily HK](https://chinadailyhk.com/article/610146)). A woman was rescued at 30 h [S] ([Malay Mail](https://www.malaymail.com/amp/news/world/2025/03/30/against-all-odds-rescuers-save-woman-trapped-for-30-hours-in-myanmar-quake-wreckage/171352)).
  A Chinese team rescued 3 people at ~60 h [S] (China Daily).
- **~120 h [D]:** a man was pulled out alive "five days after" the quake [S] ([Jakarta Post/AFP](https://www.thejakartapost.com/world/2025/04/02/myanmar-quake-victim-rescued-after-5-days-as-aid-calls-grow.html)).
- **International:** 2,095 personnel from 26 countries by 30 Apr; ~30 Chinese teams (>500 people) [S] ([Xinhua](https://imgs.news.cn/english/20250430/7e840738aa264740807edbb522859115/c.html)) *(weak attribution)*. Civil war complicated access [S] (news).

---

## 3. Drones and new sensors in real collapses

| Use | What happened | Found anyone alive? | Tag / source |
|---|---|---|---|
| Surfside 2021 | 304 logged strategic drone missions; thermal flights at night; lighting; orthomosaic; NIST imaging | No live finds after day 1 (nobody survived) | [S] [IEEE Spectrum](https://spectrum.ieee.org/building-collapse-surfside-robots) |
| Turkey 2023 | ~50 military UAVs, TB2/Akıncı >1,551 h; Xtend/Polaris thermal drones; BRINC indoor drones; 10 donated Chinese drones | No count published | [S] [Baykar](https://baykartech.com/en/press/bayraktar-tb2-aknc-uavs-conduct-over-1551-hours-of-flights-in-quake-zone/), [JPost](https://www.jpost.com/middle-east/article-731686) *(vendor)* |
| Nepal 2015 | mapping (UAViators, KU, Pix4D, DJI); civil-aviation refusals | No: mapping only | [S] [case study](https://zoinet.org/wp-content/uploads/2018/01/3Case-Study-Nepal.14April2016.pdf) |
| Hualien 2024 | NFA drones + 7-person Turkish drone team over Taroko; phone signal search failed | No | [S] [Taiwan News](https://www.taiwannews.com.tw/en/news/5137584), [TVBS](https://news.tvbs.com.tw/amp/english/2449120) |
| Nepal 2015, FINDER (ground radar) | heartbeats detected under 2 buildings; 4 men rescued | **Yes, 4** | [S] [DHS](https://www.dhs.gov/archive/science-and-technology/news/2015/05/05/finder-helps-save-four-nepal) |

**DJI "Drones Save Lives" (vendor-sourced, built from press reports, an undercount by DJI's own admission):**
- ≥59 people in 18 incidents, 2013–Mar 2017 [S] ([DJI](https://dji.com/newsroom/news/dji-releases-first-count-of-lives-saved-by-drones)).
- 124 by Apr 2018 [S] ([DroneDJ](https://dronedj.com/2018/04/30/dji-reports-65-people-rescued-by-drones-last-year-and-124-in-total-around-the-world/)).
- 500 by Dec 2020, and >1,000 people in 616 incidents in >40 countries by Jul 2023 [S] ([DSLRPros, citing DJI](https://www.dslrpros.com/blogs/drone-trends/dji-drones-1000-lives-saved-and-counting)).
- **1,054** by Nov 2025 [S] ([DroneXL](https://dronexl.co/guides/drones-for-good/)).
- Almost all of these are wilderness, water and missing-person searches with thermal cameras. **We found no entry in which a drone located a person inside a collapsed building.**

**Academic drone-radar and phone-detection work (none tested on real rubble):**
- **Jing et al. 2025, *Sensors* 25:5232.** X4M200 IR-UWB radar (7.29 GHz) on a ~3 kg quadcopter, 0.4–5 m range.
  Respiration-rate accuracy **98.68%** at 3 m over grass, **93.76% through a brick wall**; heart rate 96.4% (grass only). Falls to 93% / 90% at 5 m. One subject, 30 s per test, **no rubble test** [S] ([PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC12431357/)).
- **Qatar University LifeRadar:** drone swarm with distributed MIMO radar; "90% ML accuracy", early-stage validation [S] ([QU](https://www.qu.edu.qa/en-us/conference/qu-ip-and-innovation/Pages/LifeRadar.aspx)) *(weak)*.
- **Centum Lifeseeker:** a phone-geolocation payload for drones and helicopters that works without network coverage. The vendor lists "detection under rubble" and claims >25 customers and "hundreds of missions".
  We found **no documented collapse rescue** [S] ([Unmanned Systems Technology](https://www.unmannedsystemstechnology.com/company/centum-research-technology/lifeseeker-mini/)) *(vendor)*.
- **Patents** (e.g. US 11,754,696) claim UWB through 10 cm of concrete from a UAV [S] ([USPTO](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11754696)). These are claims, not results.

**Our reading:** drones in collapses so far have been used for **overview, mapping and lighting**. Detecting life has been done by hand-placed ground devices (FINDER, LifeLocator) or by ears and dogs.
Nobody has fielded a drone that **lands on a pile and scans for breathing through the rubble with a belly radar**. The published flying-radar work is single-subject, hovering 0.4–5 m above open ground or a brick wall. That gap is where our landed belly radar sits. It is a gap, not proof that it works.

---

## 4. Patterns: what fails again and again

**P1. Neighbours, not professionals, make most rescues, and they rely on hearing people.**
- Kobe: 77.5% of the 35,000 people rescued were dug out by citizens; 95–98% got out by themselves or with family and neighbours [S] (§2.9).
- Across 14 earthquakes, relatives and neighbours performed 60–100% of extrications [S] ([Rom & Kelman](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)).
- Surfside's Jonah Handler and Haiti's Darlene Etienne were both found **because a bystander heard a voice** [S] (§2.12, §2.7).
- *Implication:* whoever cannot call out is left for the professionals, and they arrive late (P2).

**P2. International teams arrive late and save a tiny fraction of victims.**

| Event | Int'l teams | Live rescues | Rescues / deaths | First arrival |
|---|---|---|---|---|
| Marmara 1999 | 92 | 144 | 0.85% [D] | 16 h |
| Chi-Chi 1999 | 37 | 6 | 0.25% [D] | 18 h |
| Wenchuan 2008 | 6 | 2 | 0.003% [D] | >72 h |
| Haiti 2010 | 67 | 134 | 0.07–0.13% [D] | 23 h |
| Nepal 2015 | 76 | 16 | 0.19% [D] | <12 h |
| Turkey 2023 | 90 (AAR) – 255 (AFAD) | ~300 | **0.56%** (300 / 53,537) [D] | "within hours" (mobilising) |
| Morocco 2023 | 4 countries accepted (100 teams waiting) | ≈0 reported | ≈0 [S] *(weak)* | ~1.5–2 days to acceptance [D] |

Rows 1–5 are from Rom & Kelman's Table 2 [S]; the percentages are ours, using their death tolls. Their headline result is that **every event was under 0.85%** [S].
Kobe let in only one foreign team and quarantined its dogs (§2.9). Rana Plaza and Weiguan declined foreign help (§2.11, IMPACT §4).

**P3. The 72 h window closes, but the tail is long and lives are still being found.**
- The share of extricated people who are alive falls steeply after day 1: Kobe 80.5 → 28.5 → 21.8% → ~6% after 72 h; Weiguan's last live rescue was at 62.3 h (IMPACT §1a).
- Yet people come out alive **far** beyond 72 h, typically one at a time, after a long single-site effort:
  - Turkey 103 / 198 / 222 / 248 / 261 / 278 / **296 h**;
  - Wenchuan 179 / 196 h;
  - Haiti ~360 h;
  - Rana Plaza ~390 h;
  - Chi-Chi ~130 h;
  - Myanmar ~120 h;
  - Nepal ~168 h [S] (sections above).
- In Turkey "the numbers rescued alive have dropped to just a handful" by the second week [S] (Reuters via [Inquirer](https://newsinfo.inquirer.net/?p=1731643)).
- *Implication:* every site written off early may hold one of these. Every site kept open too long ties up a crew. The value is in **knowing which voids still hold someone breathing**.

**P4. Search depends on sound, and sound needs a conscious victim and a silent site.**
- Turkey: crews stopped every generator, shouted "Sesimi duyan var mı?" and asked trapped people to scratch with a fingernail, because the listening devices pick up "even the movement of a small stone" [S] (§2.5). AP: helicopters and digging made it hard to hear any sign of life [S].
- Weiguan: listening devices and dogs checked each layer before the machines took it away (IMPACT §3).
- Bam 2003: 28% of injured survivors were unconscious when rescued (IMPACT §1c). By design, a silent victim gets no tap and no call.
- *Implication:* an unconscious person in a void who breathes but cannot call is invisible to the main method. That is the gap the landed radar targets.

**P5. Heard or located, but not reached in time.**
- Surfside: a woman's voice was heard until ~11:00 on day 1, and nobody reached her [S].
- Rana Plaza: Shaheena was located alive at >100 h; >20 h of tunnelling ended in a fire, and she died [S].
- Hatay 2023: trapped people "were still making sounds while nobody approached" [S].
- Wenchuan: Ma Yuanjiang was located on 18 May and freed on 20 May after 30 h of chiselling [S].
- Weiguan: extrication of deep victims took 12–29 h (IMPACT §2).
- *Implication* (honest): faster location does not fix slow extrication. It does start the long extrications **earlier**, and it stops crews spending hours in the wrong place (P6).

**P6. Wrong targets, false leads and lost information.**
- Mexico 2017: the "Frida Sofía" girl who never existed held national attention and rescuers for ~1 day [S].
- Haiti: 2 h of digging at an empty site before a 3 h live rescue (IMPACT §2).
- Turkey: many tweeted addresses could not be identified [S]. Worksites triaged by non-INSARAG and local teams were **not marked in the shared system (ICMS)** [S] ([INSARAG FD / team reports](https://www.insarag.org/wp-content/uploads/2024/04/Future-Directions-of-INSARAG.pdf)) *(weak: from a team report quoted in search)*.
- Rana Plaza: the search camera could not get into the narrow voids [S].

**P7. Coordinating hundreds of teams.**
- Turkey: AFAD counted 255 international teams, but the UN coordination cell coordinated 118, i.e. **only ~46% of the foreign teams were inside the UN coordination system** [D]. There were 26,000 buildings to search [S].
  The cell itself moved from Hatay to Adıyaman mid-operation [S] (INSARAG FD).
- Haiti: 43–67 teams, and the counts of live rescues disagree by date (70 → 122 → 132 → 134 → 150) [S], so even the scoreboard was not shared.
- Morocco: 100 registered teams waited for a request that never came [S].
- *Implication:* a shared, sector-level map of "checked / candidate / unchecked", and **a list of candidates, not confirmations**, is what our planner already outputs.

**P8. The "manual limit" then machinery: the blind dig.**
- Weiguan: machines from 64 h, after every reachable void had been re-checked, with no life signs (IMPACT §2).
- Hualien 2018: manual search stopped at 106 h, and the last 2 people were left under a beam that held up the building [S].
- Rana Plaza: machinery held back to day 5 for fear of crushing survivors [S].
- Tunghsing 1999: the brothers were found by layer-by-layer machine removal (IMPACT §4).
- Surfside: a 15 h pause for the unstable structure [S].
- *Implication:* a drone that lands and re-scans each newly exposed layer for breathing, with nobody standing on the pile, fits exactly the phase where people currently stand aside and wait.

---

## 5. Judge-ready stat cards

1. **77.5%** of the 35,000 people rescued from the rubble in Kobe (1995) were dug out by neighbours and family. Firefighters and police rescued 22.5%.
   — [Kobe City / ADRC 2015](https://web.adrc.asia/acdr/2015/documents/ID251_3.pdf)
2. **< 0.85%:** in every one of 14 major earthquakes reviewed, international USAR teams' live rescues were under 0.85% of the death toll, and the first team took 12–56 h to arrive.
   — [Rom & Kelman 2020, BMJ Global Health](https://pmc.ncbi.nlm.nih.gov/articles/PMC7745699/)
3. **~5,000 international rescuers in 90 teams saved ~300 lives** in Türkiye 2023, while 53,537 people died there (0.56%).
   — [INSARAG AAR](https://insarag.org/aar-turkiye/); [Sputnik Türkiye (AFAD data)](https://anlatilaninotesi.com.tr/20240205/6-subat-depreminde-kac-kisi-oldu-neler-yasandi-kac-bina-yikildi-depremin-siddeti-son-depremler-1080416968.html)
4. **255 foreign USAR teams deployed in Türkiye; the UN coordination cell coordinated 118 of them (46%).** The rest reported only through AFAD or nobody, across 26,000 buildings to search.
   — [INSARAG Future Directions](https://www.insarag.org/wp-content/uploads/2024/04/Future-Directions-of-INSARAG.pdf); Sputnik Türkiye
5. **296 hours:** in Antakya, two adults and a child were pulled out alive 12 days after the quake. Search depends on silence and on victims who can still scratch or call.
   — [NTV](https://www.ntv.com.tr/turkiye/hatayda-296-saatte-enkazdan2-kisi-sag-cikarildi,s7Ou77FfN0q1wHZJYr-k4A); [AP](https://www.news4jax.com/news/world/2023/02/08/in-earthquake-rescues-noisy-gear-and-digging-then-silence/)
6. **Surfside 2021:** rescuers heard a woman's voice under the rubble until ~11 a.m. on day 1 and could not reach her. Of the up to 159 people unaccounted for, nobody was found alive after the first hours, despite 304 drone missions and 14 days of rescue.
   — [Wikipedia](https://en.wikipedia.org/wiki/Surfside_condominium_collapse); [IEEE Spectrum](https://spectrum.ieee.org/building-collapse-surfside-robots)
7. **Haiti 2010:** 1,739 international rescuers and 161 dogs in 43 teams made 132 live rescues, about **13 rescuers per life** [D]. The last famous survivor (day 15) was found because neighbours heard her voice.
   — [OCHA](https://reliefweb.int/report/haiti/rescuers-still-finding-survivors-haiti); USAID fact sheets *(offline; via search index)*; [PBS](https://www.pbs.org/newshour/show/fifteen-days-below-rubble-final-quake-survivor-in-haiti)
8. **Nepal 2015:** a hand-carried microwave radar (FINDER) detected heartbeats under 2 collapsed buildings, and 4 men were dug out alive. The best published *drone* breathing-radar test is still one person over grass or behind a brick wall at ≤5 m, with no rubble.
   — [DHS](https://www.dhs.gov/archive/science-and-technology/news/2015/05/05/finder-helps-save-four-nepal); [Jing et al. 2025, Sensors](https://pmc.ncbi.nlm.nih.gov/articles/PMC12431357/)

Backup cards:
- Rana Plaza: Shaheena was found alive at >100 h, and a fire ended 20 h of tunnelling. Reshma came out at day 17.
- Morocco 2023: 100 teams waited unused.
- Mexico 2017: "Frida Sofía", the girl who never existed.

---

## 6. Biggest gaps

1. **National "rescued alive" totals are missing** for Chi-Chi, Wenchuan, Marmara, Nepal and Turkey 2023 (only a ">8,000 by day 2–3" snapshot). Official AFAD, NFA and State Council reports exist but are not searchable online.
2. **Trapped-person estimates** exist only for Kobe (35,000 needing rescue) and single buildings (Weiguan 309, Rana Plaza 3,122, Rönesans ~800).
3. **No source counts survivors located by drone in a collapse.** The vendor tallies (DJI 1,054) are wilderness and water searches.
4. **The INSARAG Türkiye AAR PDF** (>10 MB) could not be fetched. Its per-team live-rescue and arrival-time data would strengthen P2 and P7.
5. **Last live rescue times** come from news reports (hours as stated by agencies via media). The Chi-Chi Tunghsing time conflicts (130 vs ~144 h).

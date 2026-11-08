# Hardware: real components for a Larpverse drone, its belly radar and the command van

Machine-readable twin: [`data/hardware_bom.json`](../data/hardware_bom.json) (BOM, budget, frequency plan, Isaac
mount positions). How the parts talk to each other: [docs/COMMS_ARCHITECTURE.md](COMMS_ARCHITECTURE.md). Rescue
hotspot link budget, portal and privacy: docs/HOTSPOT.md (comms agent). Sim numbers: `sim/platform.py` and
[docs/PLATFORM.md](PLATFORM.md).

## One-slide summary

- **Every sensor is a part you can buy today.** The drones use an Ouster OS1-128 LiDAR, MEMS mic arrays, a 130 dB-class loudspeaker, a Jetson AGX Orin, a Doodle Labs mesh radio and a 2.4 GHz OpenWrt hotspot. The only custom parts are the belly-mounted radar module and the mounts.
- **The radar is mounted on the drone's underside, facing down.** The drone uses its LiDAR map to find a flat spot on the rubble and lands there, so the radar's antennas rest on the rubble surface and scan down through it. The 5 kg belly-mounted radar module has a USRP B210 software radio that sweeps 0.4–2 GHz, wideband antennas, 4 mics, a Raspberry Pi 5 and a 99 Wh battery, and it is cabled to the drone's Orin.
- **Each drone carries about 14 kg of mission payload:** 6.0 kg on the airframe, a 4.9 kg belly-mounted radar module and a 3.1 kg unallocated reserve. The payload electronics draw about 82 W in normal use, up to about 139 W. That's roughly 1% of hover power, so mass is what costs flight time, not electricity.
- **Price:** about $61k per drone on a DJI FlyCart 30 airframe, or about $87k on a US-built Harris Aerial H6HL. The command van adds about $51k: mesh mast, satellite/5G link, ops laptop, generators, spare batteries and a tent. All prices are estimates.
- **The DJI airframe is a reference, not the build.** It matches the sim's geometry, but it has no MAVLink and no open payload port, and new DJI models are on the US FCC Covered List. The recommended build is an ArduPilot hexacopter with the same 95 kg MTOW (Harris H6HL).

Labels: **est.** = our estimate, **verify** = we could not confirm the figure on a primary page. Prices are street or
reseller prices in USD as of Nov 2026 and are always approximate.

## 1. Build at a glance

| Subsystem | Selected part | Mass | Power typ / peak | Approx. price | Status |
|---|---|---|---|---|---|
| Airframe (sim reference) | DJI FlyCart 30 + 2 × DB2000 | 65.1 kg | (flight ~9.7 kW est.) | ~$22k | COTS, closed |
| Airframe (recommended) | Harris Aerial Carrier H6HL | 55 kg | (flight) | quote; $50k est. | COTS, ArduPilot |
| LiDAR | Ouster OS1-128 Rev7 + 45° bracket | 0.45 + 0.25 kg | 18 / 22 W | ~$18k est. (quote) | COTS |
| Mic array (drone) | 8 × Infineon IM69D130 on a 1.0 m ring + PDM→USB | 1.0 kg est. | 1 W | ~$600 est. | custom from COTS |
| Loudspeaker | CZI MP130 V2 (sim: DJI Zenmuse V1) | 0.55 kg | 2 / 25 W | ~$1k est. | COTS + adapter |
| Thermal (optional) | Teledyne FLIR Hadron 640R+ on a micro gimbal | 0.36 kg est. | 3 / 5 W est. | ~$8k est. | COTS + integration |
| Compute | Jetson AGX Orin 64GB, rugged carrier, 2 TB NVMe | 1.2 kg est. | 40 / 60 W | ~$4.6k est. | COTS + integration |
| Mesh radio | Doodle Labs Mesh Rider, 5.8 GHz SKU + antennas | 0.20 kg | 5 / 8 W | ~$3k est. | COTS + integration |
| Rescue hotspot | GL.iNet GL-MT300N-V2 (OpenWrt) + omni | 0.10 kg | 2 / 2.75 W | ~$60 | COTS |
| GNSS / time | Holybro H-RTK F9P Ultralight (u-blox ZED-F9P) | 0.02 kg | 0.5 W | ~$300 est. | COTS |
| Radar module link | wired link from the belly radar module to the Orin carrier (budget line kept) | 0.04 kg | 2 / 3 W | ~$40 | COTS |
| Reserve | unallocated mass, power and cost reserve (keeps the 14.0 kg budget and endurance estimate) | 3.1 kg | 0 / ~80 W est. | ~$3.5–5k | — |
| Power + structure | DC-DC, harness, payload plate, booms | 1.8 kg est. | 8 / 12 W losses | ~$1k est. | custom |
| **Belly-mounted radar module** | B210 SFCW radar, downward-facing antennas, 4 mics, Pi 5, IMU, 99 Wh | **4.9 kg** | 16 / ~23 W (own battery) | **~$5.6k est.** | **custom** |
| **Per drone, mission payload** | | **14.0 kg** | **82 / 139 W** | **~$35k** (no thermal) | |

## 2. Airframe

| | DJI FlyCart 30 | DJI Agras T50 | Harris Aerial Carrier H6HL | Freefly Alta X (Blue) |
|---|---|---|---|---|
| Layout | coaxial X8, 54-in props, 2.2 m wheelbase, 3.085 m span | same frame class, 3.085 m | hexacopter, 40-in props, 3.213 m prop tip to tip | X8 class, 2.0 m class |
| Empty / MTOW | 42.5 kg without battery, about 65 kg with 2 × DB2000 / 95 kg | 39.9 kg without battery / 92 kg (spray) | 55 kg / 95 kg | 10.4 kg / 34.9 kg |
| Max payload | 30 kg (dual battery), 40 kg (single) | 40 kg spray, 50 kg spread | 40 kg | 15.9 kg |
| Hover endurance | 29 min empty, 18 min at 30 kg | not listed | 48 min empty, about 30 min at 20 kg, about 15 min at 40 kg (chart, with a 2-min reserve) | 50 min empty, 22 min at 10 kg |
| Battery | 2 × DB2000, 1984.4 Wh, about 11.3 kg each | DB1560, about 1.57 kWh | 8 × 6S 30 Ah at 12S, about 5.3 kWh | 12S packs |
| IP / wind | IP55 / 12 m/s | IP55 on the relay only | IP43 | n/a |
| Autopilot / API | DJI, no MAVLink; Payload SDK on E-Port Lite + payload power port | DJI | **Cube Orange, ArduPilot, MAVLink native** | PX4 on Blue Cube |
| Payload bus | power port, voltage unpublished (verify) | n/a | **5 / 12 / 24 / 48 V regulated, UART / Ethernet / CAN / USB** | 3 × 10 A at 50 V |
| Price | about $16.6k base, about $20–26k usable (resellers) | n/a | quote only | $32,225 (Freefly store) |
| Sources | [specs](https://www.dji.com/flycart-30/specs), [price](https://dronesgator.com/drone/dji-flycart-30) | [specs](https://ag.dji.com/t50/specs) | [brochure](https://frontierprecision.com/wp-content/uploads/2026/07/Frontier-Precision-Harris-Aerial-H6HL-Quad-Brochure.pdf) | [product](https://freeflysystems.com/alta-x), [store](https://store.freeflysystems.com/collections/alta-x/products/alta-x-blue) |

**Choice.** The **FlyCart 30 stays the sim reference** (`heavy_sar`): it has the right size,
and the sim geometry comes from it. The **build we'd buy is the Harris H6HL** class:

1. It is **MAVLink native**. The van's coordinator can command it directly (COMMS_ARCHITECTURE.md §4). The FlyCart
   30 exposes only an **E-Port Lite**, a USB-C calibration port reached through a USB-to-TTL adapter, plus a payload
   power port ([DJI PSDK port table](https://developer.dji.com/doc/payload-sdk-tutorial/en/quick-start/drone-port.html)).
   Whether a payload computer can fly it is **verify**.
2. It has an **open payload bus** with 48 V and Ethernet, so the Orin, LiDAR and radios run from the flight pack.
3. It has the **same MTOW** (95 kg). With our 13.4 kg payload, the brochure chart suggests about 37 min (est.).
4. It **avoids DJI's regulatory risk**. On 22 Dec 2025 the FCC added foreign-made UAS, including DJI, to the
   Covered List, so **new DJI models cannot get US equipment authorisation**. Models authorised earlier stay legal,
   and DJI is challenging the listing in court
   ([overview](https://uavcoach.com/dji-ban/), [component rule](https://www.thedroneu.com/blog/fcc-dji-autel-parts-ban-october-13)).
   Many public-safety buyers in the US and elsewhere must buy NDAA-compliant or Blue UAS aircraft. Taiwan, where our
   case study is set, also restricts Chinese-made drones for government use (verify the current rule). The H6HL
   brochure carries the Blue UAS and NDAA marks.

What the H6HL costs us: IP43 instead of IP55, so it is a poorer match for rain and dust at a collapse site, and the
belly radar mount and skid clearance for rubble landings have to be integrated (verify). The Agras T50 is rejected
because it is a spray drone with no payload API. The Alta X is rejected for the radar build because 15.9 kg
leaves no margin, but it suits a light mapping-only scout.

## 3. LiDAR

| | **Ouster OS1-128 Rev7** (selected) | Hesai XT32 | Livox Mid-360 |
|---|---|---|---|
| Channels / VFoV | 128 / 45° | 32 / 31° | non-repetitive / 360° × (−7…52°) |
| Points/s | **1.31 M** in 1024×10 mode (up to 5.2 M single-return at 2048×20; the page states 10.4 M, likely dual return) | 640k single return | 200k |
| Range @10% | 90 m | 0.05–120 m (max) | 40 m |
| Mass / power | **447 g / 14–20 W** (22 W at start-up) | 800 g / 10 W typ | **265 g / 6.5 W** |
| Ingress / interface | IP68 + IP69K / Gigabit Ethernet, PTP | IP6K7 / 100BASE-TX | IP67 / Ethernet, built-in IMU |
| Price | quote only; about $18k est. | quote only | DJI store |
| Source | [ouster.com](https://ouster.com/products/hardware/os1-lidar-sensor), [Clearpath listing](https://store.clearpathrobotics.com/products/os1-rev-7), [Rev7 datasheet](https://cdn.shopify.com/s/files/1/0400/1752/6943/files/datasheet-rev7-v3p0-os1.pdf) | [hesaitech.com](https://www.hesaitech.com/product/xt32/), [manual](https://www.manualslib.com/manual/3350848/Hesai-Xt32m2x.html) | [livoxtech.com](https://www.livoxtech.com/mid-360/specs) |

**Why the OS1-128:** the density study in docs/PLATFORM.md §3 favours it with a 45° pitched mount. Its IP69K seal
survives dust and hosing down, and it outputs Ethernet with PTP time sync. The Livox Mid-360 is the low-SWaP choice
for a scout drone, and the sim's legacy fan is modelled on it. Livox is a DJI affiliate, so the same procurement
caveat applies (verify).

## 4. Through-rubble life-detection radar

**What exists off the shelf:**

| Product / module | Band | What it is | Fit for us |
|---|---|---|---|
| GSSI **LifeLocator** (III+, TRx) | **~270 MHz** (low-band UWB) | hand-placed rubble life detector used at Sichuan 2008 and Haiti 2010; TRx detects breathing at 7 m in a vendor sample | right physics, but **TRx is listed "no longer available"**; no drone interface ([GSSI](https://geophysical.com/products/lifelocator-trx), [Frontiers 2023](https://www.frontiersin.org/journals/earth-science/articles/10.3389/feart.2023.1158764/full)) |
| **SJ-3000** class (China) | 400 / 500 MHz | UWB bio-radar detectors used at Wenchuan and Yushu | same idea; no export datasheet found |
| **DN-III+** | 400 MHz | UWB life detector, 430 × 430 × 180 mm, 6.5 kg, 10 h battery (vendor claims) | **closest COTS analogue to our belly radar module**; unverified claims, no API ([vendor](https://hdx.ihongde.com/products/dn-iii-one-dimensional-radar-life-detector)) |
| Geotech **RD-400** | n/a | portable breathing/movement detector, 49 × 20 × 40 cm | hand-held only ([vendor](https://gpr.geotechru.com/product/radar-detector-rd-400/)) |
| MIT LL **MURMUR** | n/a | robot-mounted radar that detects breathing under rubble (R&D 100, 2021) | research system, not for sale ([MIT LL](https://www.ll.mit.edu/partner-us/available-technologies/motion-under-rubble-measured-using-radar-murmur)) |
| Novelda **X4** SoC | centre **7.29 or 8.748 GHz** | single-chip IR-UWB, used for respiration sensing | **rejected**: 7–9 GHz is attenuated many times more per metre in concrete than sub-1 GHz, so it suits through-wall or bedside use, not 1–3 m of rubble ([X4M06 sheet](https://intopion.com/sdesigner/product/creaca_RES/X4M06/datasheet.pdf)) |

**What we'd build (custom, honest):** a **stepped-frequency CW (SFCW) radar** from a software-defined radio.

- **SDR: Ettus USRP B210**, 70 MHz–6 GHz, 56 MHz instantaneous bandwidth, 2 × 2, USB 3, 350 g board, $2,387
  ([Ettus](https://www.ettus.com/all-products/ub210-kit/), [KB](https://kb.ettus.com/B200/B210/B200mini/B205mini)).
  The cheaper option is a Nuand **bladeRF 2.0 micro xA4** (47 MHz–6 GHz TX, 56 MHz, about $540,
  [rtl-sdr.com](https://www.rtl-sdr.com/?p=24978)).
- **Sweep:** 0.4–2.0 GHz in about 32 sub-bands of 50 MHz (est.). The 1.6 GHz synthetic bandwidth gives c/2B =
  **9.4 cm range bins**. At about 1 ms per sub-band the full sweep takes about 32 ms, about 30 Hz slow-time (est.;
  verify the B210 retune time). That is far above breathing (0.1–0.5 Hz) and heartbeat (about 1–2 Hz) rates.
  Because the radar steps through frequencies, it can **skip bands**: GNSS 1.164–1.61 GHz, local cellular, and the
  rescuers' radio channels. The Frontiers study reports that narrowband interference from rescuers' radios was the
  main problem at Wenchuan and Yushu.
- **Front-end:** 1 TX + 2 RX wideband Vivaldi / bow-tie elements in an absorber-backed cavity, pressed against the
  rubble, plus LNAs and filters. There is **no power amplifier**: UWB through-wall emissions are limited by an
  emission mask (US [47 CFR 15.509](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-F/section-15.509),
  which also limits imaging below 960 MHz to law enforcement, fire and rescue). The SDR's few dBm is already enough.
  Taiwan NCC rules: **verify**.
- **DSP:** the Pi 5 in the radar module runs range FFTs and buffers slow-time. The drone's Orin runs the vitals chain already
  modelled in `sim/radar_dsp.py` and `sim/vitals.py`.
- **Off-the-shelf vs custom:** the radio, LNAs, Pi and battery are COTS. The **antenna cavity, sweep firmware,
  calibration and the ground-coupled housing are custom**, and that is where the engineering risk sits. No product
  we found sells a sub-2 GHz breathing radar with an API for drones.

## 5. Microphones

| Use | Part | Key spec | Price | Source |
|---|---|---|---|---|
| Drone ring (8 ch, 1.0 m) | **Infineon IM69D130** MEMS, PDM | 69 dB(A) SNR, 130 dB SPL AOP, −36 dBFS | low, at chip level | [Infineon](https://www.infineon.com/cms/en/product/sensor/mems-microphones/mems-microphones-for-consumer/im69d130/) |
| Radar module (4 ch at 0.42 m) | **TDK ICS-43434** I2S MEMS | 65 dBA SNR, 120 dB AOP, 24-bit I2S, about $2.4 | [DigiKey](https://www.digikey.com.au/en/products/detail/tdk-invensense/ICS-43434/6140312) |
| Bench / prototype | **miniDSP UMA-16 v2** | 16 × Knowles SPH1668LM4H in a rectangular grid (about 42 mm pitch, verify), USB, about €370 | [miniDSP](https://www.minidsp.com/products/usb-audio-interface/uma-16-microphone-array), [review](https://audioreview.frieve.com/products/en/minidsp-uma-16/) |
| COTS radar-module fallback | **Seeed ReSpeaker XVF3800** | 4 mics, USB, on-chip DoA / beamforming / AEC, **16 kHz max**, about $50 | [Hackster](https://www.hackster.io/news/seeed-studio-taps-xmos-xvf3800-for-a-new-ai-powered-respeaker-microphone-array-512d822b3962) |

The UMA-16's 0.13 m aperture gives about 150° beams at 1 kHz, so it's a bench tool only. The 1.0 m ring gives
about 20° (docs/PLATFORM.md). The IM69D130's 130 dB AOP leaves headroom while the rotors spin down, but we still
**listen only with rotors off**. Optional: a **geophone** in the radar module base (SM-24 class, verify) to pick up tapping
through the debris, the way hand-held seismic life detectors work.

## 6. Loudspeaker

| | **DJI Zenmuse V1** (sim value) | **CZI MP130 V2** (build) | Foxtech M1 |
|---|---|---|---|
| SPL | **129 dB @1 m** on M400 (127 on M350/M300) | 120–130 dB (listings disagree: **verify**) | 125 dB max |
| Band / beam | 800 Hz–8 kHz | 80° × 50° beam, 500 m broadcast | n/a |
| Mass / power | 690 g / 30 W rated | 545–550 g / 25 W | n/a |
| Mount | **DJI SKYPORT only** (M400, M350, M300) | SkyPort + CZI adapter ring for other frames | universal |
| IP | IP54 | n/a | n/a |
| Source | [DJI](https://enterprise.dji.com/zenmuse-v1/specs) | [Dronefly](https://www.dronefly.com/loudspeaker-for-matrice-300-matrice-200-v2-series.html), [Maverick](https://maverickdrone.com/products/mp-130-speaker) | [listing](https://www.racer.lt/item/foxtech-m1-sky-speaker-megaphone-laudspeakers-system-ultra-long-range-universal-for-fpv-rc-racer-drone-quacopter-29650.html) |

Neither the FlyCart 30 nor the H6HL has a SKYPORT, so the V1 is a spec reference and the build uses an MP130-class
speaker on an adapter ring. A plain horn driver with a class-D amplifier fed by the Orin's audio output would also
work. Audio is played from cached files (vetted templates, `sim/guidance.py`), so call-outs still work without the mesh.

## 7. Compute: what runs where

| Module | AI / power | Mass | Price | Source |
|---|---|---|---|---|
| **Jetson AGX Orin 64GB** (selected) | 275 TOPS, 15–60 W, 64 GB | module plus carrier about 1.2 kg est. | module $3.3–3.6k at resellers | [NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) |
| Jetson Orin NX 16GB (fallback) | 157 TOPS (Super), 10–25 W, 16 GB | module about 26–29 g | about $890 | [Waveshare](https://www.waveshare.com/jetson-orin-nx.htm?sku=24456) |
| Raspberry Pi 5 (radar module) | CPU only, 5–12 W | about 50 g | about $100 | [raspberrypi.com](https://www.raspberrypi.com/products/raspberry-pi-5/) |

- **Drone (Orin):** KISS-ICP SLAM at 10 Hz on the OS1 and map-delta encoding; landing-patch screening; stage-1
  acoustics (denoise, VAD, YAMNet novelty, 8-ch delay-and-sum bearings); radar vitals DSP on the radar module's range
  profiles; landing-spot sequencing with the IMU seating check; speaker playback; the hotspot's portal page;
  store-and-forward on a 2 TB NVMe; and a local fallback mission when the mesh is down.
- **Radar module (Pi 5):** SFCW sweep control, range FFT, IMU, 4-mic capture, wired link to the drone.
- **Van:** coordinator, belief fusion, dashboard, tasking and broadcast APIs, portal backend, rules decider.
- **Cloud:** Gemini audio analysis and the Gemini decider.

Latencies and the no-cloud / no-mesh behaviour are in COMMS_ARCHITECTURE.md §3. The AGX rather than the NX: SLAM,
YAMNet and the vitals DSP running together would saturate the NX's 16 GB. The AGX's extra 15–35 W is about 0.3% of
hover power.

## 8. Mesh radio

| | **Doodle Labs Mini-OEM Mesh Rider** (selected) | Silvus StreamCaster LITE 5200 (SL5200) | Microhard pDDL2450 |
|---|---|---|---|
| Bands | 900 + 2.4 GHz, 1350–2510 MHz (L, M1–M6), 2.4 GHz, 5650–5755 MHz (per SKU) | L 1350–1440, S 2200–2500 MHz | 2.402–2.478 GHz |
| Throughput | 80 Mbps max (20 MHz) | up to 100 Mbps | up to 25 Mbps (8 MHz) |
| Range | 50+ km typ (line of sight, vendor) | n/a | not published |
| Tx power | 1.6 W (32 dBm) | 2 W (4 W effective with eigen-beamforming) | 27 dBm (0.5 W) at 64QAM |
| Mass / power | **34 g / 5 W avg, 8 W peak** | 26 g bare, 52 g shielded / 4–13 W | 5 g OEM, 50 g on a carrier, 165 g enclosed |
| Mesh / security | self-forming mesh, AES-128/256, FIPS 140-3 | MN-MIMO MANET (hundreds of nodes), AES-256, FIPS 140-3, Blue UAS framework | point-to-point / multipoint; mesh **verify**; optional AES |
| Price | quote | quote | quote |
| Source | [Doodle Labs](https://doodlelabs.com/product/mini-oem-smart-radio) | [datasheet](https://silvustechnologies.com/wp-content/uploads/2024/10/Silvus_SL5200_OEM_Module_Datasheet_DL1_DIGITAL.pdf), [PR](https://prnewswire.com/news-releases/silvus-technologies-unveils-streamcaster-lite-5200-302283091.html) | [brochure](https://microhardcorp.com/brochures/pDDL2450.Brochure.Rev.1.4.1.pdf) |

**Why Doodle:** it has the lowest SWaP of the three, it's widely integrated on PX4/ArduPilot drones, and it routes
Ethernet natively. **Band:** **5.8 GHz ISM** by default. It's licence-free, sits above the 0.4–2 GHz radar sweep,
stays off the 2.4 GHz hotspot, and matches `sim/comms.py` (`MESH_FREQ_MHZ = 5800`). The Mini-OEM page lists
5650–5755 MHz (Japan) but not 5.8 GHz, so a 5.8 GHz build may need the larger OEM Mesh Rider (RM-5800 class,
about 100 g, verify). With an emergency frequency assignment, the M-band SKU at 2.2–2.3 GHz trades bandwidth for
better diffraction around rubble. Silvus is the stronger pick if MN-MIMO robustness in a rubble canyon matters more than cost; it's
Motorola-owned and in the Blue UAS framework. Microhard is a cheap point-to-point link, not a mesh.

**Frequency plan** (also in the BOM JSON): radar 0.4–2.0 GHz with GNSS and cellular notches · RC link 2.4 GHz (or
902–928 MHz on the H6HL, which overlaps the sweep, so skip those steps) · hotspot 2.4 GHz channel 1 ·
mesh 5.8 GHz · GNSS receive only.

## 9. Rescue hotspot access point

**GL.iNet GL-MT300N-V2 "Mango"**: 2.4 GHz 802.11b/g/n, MT7628 at 580 MHz, 128 MB RAM, 16 MB flash, **39 g, under
2.75 W**, about $25 ([GL.iNet](https://store.gl-inet.com/ja/products/gl-mt300n-v2-mini-smart-router)). It runs
OpenWrt with [openNDS](https://openwrt.org/packages/pkgdata/opennds), a small captive-portal daemon (flash footprint:
verify). It uses a down-facing external omni antenna. Its Ethernet port goes to the Orin, which serves the 24 kB
portal page (`dashboard/portal/index.html`) and forwards submissions to the van. The link budget, phone RSSI
localisation and privacy rules are in docs/HOTSPOT.md (comms agent), and we don't repeat them here. The dual-band
Mango 2 (100 g, under 5 W) is the upgrade if a 5 GHz hotspot is wanted.

## 10. GNSS/RTK, IMU and thermal camera

- **GNSS / RTK:** both airframes have their own GNSS (the FlyCart 30 lists RTK positioning, verify; the H6HL uses a Cube with a
  Here-class GNSS). We add a **payload-side u-blox ZED-F9P** (Holybro H-RTK F9P Ultralight, 21 g, about 0.2 W
  module) for PPS time-stamping of the LiDAR (PTP), audio and radar
  ([u-blox](https://www.u-blox.com/en/product/zed-f9p-module), [Holybro](https://shop.holybro.com/products/h-rtk-f9p-ultralight)).
  An RTK base sits on the van mast; corrections go over the mesh.
- **IMU:** the autopilot has a triple-redundant IMU, and the OS1 has its own IMU for de-skewing. The radar module has a TDK
  ICM-42688-P to check the landed drone sits level on the rubble (the sim allows one IMU-checked lift-off and re-landing).
- **Thermal (optional):** **Teledyne FLIR Hadron 640R+**: 640 × 512 radiometric LWIR plus a 64 MP visible camera,
  about 56 g, IP54, USB/MIPI at 60 Hz, about $8k (AUD 8.8k listing for the 640R) ([GeoWeek](https://www.geoweeknews.com/news/teledyne-flir-camera-uav-drone-hadron-640-thermal),
  [DroneDJ](https://dronedj.com/2025/03/12/teledyne-flir-thermal-camera-drone/)). The sim's DJI H30T needs a DJI gimbal
  port. Thermal can't see through rubble. It finds surface casualties and tracks rescuers at night, and it's the
  first thing to cut if mass is short.

## 11. The belly-mounted radar module

**Concept:** a 0.50 × 0.50 × 0.25 m module mounted on the drone's underside, antenna face down. The drone lands on a
flat spot on the rubble over the acoustic lead (picked from its LiDAR map), so the face rests on the rubble surface
(`PodSpec` in `sim/platform.py`: 5 kg, 8 s to set down, 8 s to lift off, stays seated on slopes up to 30°).

| Item | Mass | Power | Price |
|---|---|---|---|
| Ettus USRP B210 board | 0.35 kg | ~5 W (verify) | $2,387 |
| RF front-end: 2 LNAs, filters, attenuators | 0.20 kg est. | 1.5 W | ~$500 est. |
| 1 TX + 2 RX wideband antennas in an absorber cavity, 0.48 m face | 1.60 kg est. | 0 | ~$1,500 est. |
| Raspberry Pi 5 + cooler | 0.05 kg | ~7 W (5–12) | ~$100 |
| 4 × ICS-43434 at the corners, windscreens | 0.08 kg | 0.1 W | ~$50 |
| ICM-42688-P IMU | 0.005 kg | ~0 | ~$20 |
| Geophone (optional) | 0.075 kg | 0 | ~$60 |
| 6S Li-ion 99 Wh pack | 0.65 kg est. | — | ~$200 |
| IP65 housing, RF-transparent base, rubber skid, mounting frame, strobe | 1.70 kg est. | 0.5 W | ~$600 est. |
| DC-DC, harness | 0.20 kg est. | ~2 W losses | ~$150 |
| **Total** | **4.91 kg** | **~16 W typ, ~23 W peak** | **~$5.6k** |

**Own battery, cabled data.** A 99 Wh pack runs the radar module for about 5 h at 16 W, which is many landings,
and the pack stays under the 100 Wh airline limit for shipping. The module sends range profiles, about 0.4 Mbps, to
the drone's Orin over a short cable. Powering it from the airframe's 48 V payload bus instead would remove the
battery; that's a v2 option.

## 12. Batteries and charging

- **FlyCart 30:** DB2000, 1984.4 Wh, about 11.3 kg, two per aircraft. The **C8000 station charges at up to 7.2 kW**,
  0–98% in about 26 min for one pack or 36 min for two ([Heliguy](https://www.heliguy.com/products/dji-c8000-intelligent-battery-station/)).
  DJI's FAQ asks for a **220 V single-phase generator of at least 8 kW** and says the D12000iE is not supported
  ([DJI FAQ](https://www.dji.com/ae/flycart-30/faq)). A DB2000 costs about €2,556 in an EU listing.
- **Swap cycle:** about 18 min of usable flight against about 26–36 min of charging means **3 sets per aircraft**
  (one flying, one charging, one cooling). For 3 drones that's 6 spare packs, about $17k est. The sim already models
  battery swaps (`mission.battery_swaps`).
- **H6HL:** 8 × 6S 30 Ah LiPo, quick-release, about 250 cycles. Charge with generic 12S chargers of a similar
  multi-kW class (verify).
- **Belly radar module:** 6S Li-ion 99 Wh, swapped on each battery-swap visit.
- **Payload electronics** run from the flight pack: about 82 W against about 9.7 kW hover (est.), which costs about
  0.2 min of endurance.

## 13. The command van

| Item | Part | Key figure | Price est. |
|---|---|---|---|
| Mesh base station | Doodle Mesh Rider (same band) + 2 × 8 dBi omni on a **telescopic mast, 10.5 m extended** (comms model, `sim/comms.py GCS_MAST_M`; Isaac draws it as a 6 m model, `isaac/van.py MAST_H`); a second battery "drop node" on a tripod at the pile edge | line of sight over the rubble; the drop node relays into the canyon between collapsed blocks | ~$12k |
| Internet | **Starlink Mini**: 1.1 kg, 25–40 W, 100+ Mbps down, about $300 ([TechSpot](https://www.techspot.com/news/103471-starlink-mini-299-dish-provides-100mbps-wifi-6.html)). **Tainan:** Starlink was **not commercially available in Taiwan** when we searched (lawmakers lifted the barrier in July 2026, [AFP via Citizen Digital](https://citizen.digital/article/taiwan-lawmakers-remove-legal-hurdles-for-starlink-to-operate-n386806)); **Chunghwa Telecom's OneWeb** service covers the island and was used in the 2024 Hualien quake ([moda.gov.tw](https://moda.gov.tw/en/press/press-releases/13056)) | | ~$0.3k + plan |
| WAN router | **Peplink MAX BR1 Pro 5G**: 5G + satellite WAN bonding, 10–30 V DC, GPS | | ~$1.5k |
| Ops station | **Panasonic Toughbook 40** (3.4 kg, 14-in) + 2 monitors; a small workstation runs the coordinator, map fusion, dashboard (`scripts/serve.py`) and the portal backend | ~470 W total | ~$8.6k |
| Power | **Honda EU2200i** (1.8 kW rated, about 21 kg) for the electronics, a **≥ 8 kW 220 V generator** for the C8000, and a 1.5 kWh LiFePO4 UPS | **generators ≥ 100 m downwind of any listening site**, since they raise the acoustic noise floor | ~$8.8k |
| Shelter | inflatable 4 × 4 m rescue / ops tent | | ~$3k |
| Spares | 6 × DB2000 | | ~$17k |
| **Van total** | | | **~$51k** |

## 14. Mass and power budget against `sim/platform.py`

Since commit 6dc3a18, `heavy_sar` in `sim/platform.py` uses this build's numbers: belly-mounted radar module, 14 kg payload,
23.9 min.

| | sim `heavy_sar` (after 6dc3a18) | Build on FlyCart 30 | Build on H6HL |
|---|---|---|---|
| Empty incl. batteries | 65.0 kg | 65.1 kg | 55.0 kg |
| SAR payload | **14.0 kg** in `pod` mode (radar resting on the rubble surface): 6 kg airframe sensors, compute and mounts + 4.9 kg belly radar module + 3.1 kg unallocated reserve | **14.0 kg** (airframe-side 6.0 + radar module 4.9 + reserve 3.1) | 13.4 kg (2.5 kg reserve, est.) |
| Radar | on the drone's underside: **0.48 × 0.48 m** face resting on the rubble once landed, **+7.6 dB** receive gain at 0.6 GHz (13.7 dBi at 1.2 GHz) | same | same |
| Takeoff mass | 79.0 kg (775 N once landed) | 79.1 kg | 68.4 kg |
| Endurance to 0% (est.) | **23.9 min** (17.9 min usable with the 25% reserve) | **23.9 min** (25.0 without the 3.1 kg reserve) | ~37 min (chart, includes a 2-min reserve) |
| Payload power | `idle_power_w` 250 W (rotors off, everything on) | 82 W typ / 139 W peak + autopilot idle (verify) | same |
| Span (prop tips) | 3.085 m | 3.085 m | 3.213 m |

**Remaining differences to flag:**

1. **Speaker and thermal mounts.** The Zenmuse V1 and H30T need DJI SKYPORT or gimbal ports that neither airframe
   has. The V1's 129 dB stays a fair acoustic proxy. The MP130's SPL needs checking.
2. **Idle power.** `idle_power_w = 250` is about 2× our payload estimate (its comment still budgets about 40 W for a
   drone-body radar). It's conservative and harmless: 10 min perched is about 1% of the pack.
3. **H6HL geometry.** The clearance radius would be 2.01 m instead of 1.94 m, and the hexacopter's six 40-in disks
   (about 4.9 m²) give about 7.7 m/s downwash at 68 kg, against 7.3 m/s for heavy_sar at 79 kg.

Before 6dc3a18, the sim carried a 0.60 × 0.50 m, 4 kg radar panel on the drone (+8.8 dB), a 12 kg payload and
24.6 min of endurance. Those numbers are superseded.

## 15. Isaac drone model: parts and mount positions

The full list is in `data/hardware_bom.json → isaac_mounts`. It uses the `isaac/drone_viz.py` frame: metres, +x
nose, +y left, +z up, origin at the skid bottom under the hub. The fuselage underside is at z 0.60 and the top at
1.00, and the arms are at z 1.10.

| Part | Position (x, y, z) | Size | Change |
|---|---|---|---|
| LidarPuck (OS1, pitched 45° nose-down) | 0.63, 0, 0.44 | r 0.043, h 0.074 | exists, shrink (real OS1 is 87 mm in diameter) |
| Gimbal + Thermal | 0.58, 0, 0.56 / 0.60, 0.13, 0.48 | 0.14 cube / 0.06 cube | exists |
| Mic0–7 | ring r 0.50 at z 1.03, 22.5° + 45°·k | r 0.03 | exists |
| SpeakerHorn (mouth down-aft) | −0.52, 0, 0.52 | r 0.07, h 0.13 | exists |
| **ComputeBox** | −0.25, 0, 1.06 | 0.25 × 0.18 × 0.08 | new |
| **MeshRadio** (on the compute box) | −0.25, 0, 1.12 | 0.07 × 0.07 × 0.03 | new, blue |
| **MeshAntennaL / R** (vertical, under the rear arms) | −0.39, ±0.39, 0.95 | r 0.01, h 0.20 | new, blue |
| **GnssMast** (puck on a 0.2 m mast) | 0.15, 0, 1.25 | r 0.04 | new |
| **WifiApAntenna** (pointing down) | −0.30, 0.20, 0.48 | r 0.012, h 0.18 | new, comms blue #3b82f6 |
| **PodLinkAntenna** (pointing down) | 0.25, 0, 0.55 | r 0.01, h 0.08 | new, blue; not needed (the radar module is cabled) |
| **Winch** prim (under the belly) | 0, 0, 0.52 | 0.30 × 0.20 × 0.15 | radar module mount housing; **replaces RadarPanel** |
| **Pod** prim = belly radar module, between the skids, face down | 0, 0, 0.30 | 0.50 × 0.50 × 0.25 | new |
| Radar module frame (`Pod*` prims): RadarFace / Mic0–3 / LiftEye / Strobe / Antenna | 0, 0, 0.01 / (±0.21, ±0.21, 0.255) / 0, 0, 0.27 / 0.18, 0, 0.26 / −0.18, 0, 0.29 | 0.48² × 0.02 / r 0.02 / … | new |

## Open items (for the orchestrator)

- `sim/platform.py`: the `estimates` tuple still lists "radar panel 0.6 x 0.5 m, 4 kg", and the `idle_power_w`
  note still counts "radar ~40 W" on the drone. These are stale text only; the values are current.
- Contract: the radar-scan commands (identifiers `lower_pod`, `raise_pod`, meaning land-and-scan and lift-off) and link-status fields are proposed in COMMS_ARCHITECTURE.md §4.
- Isaac visuals: `isaac/drone_viz.py`, `isaac/drone_mesh.py` and `isaac/pods.py` still draw a winch drum and a pod on a cable from an earlier draft. The design is a belly-mounted radar on a landed drone (§11); the render needs updating to match.

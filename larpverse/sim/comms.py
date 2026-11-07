"""Rescue hotspot and comms relay (contract v1.4): drone Wi-Fi APs, mesh backhaul to the GCS, captive portal,
consent-based phone localisation from RSSI.

Each heavy drone runs an open 2.4 GHz access point (SSID ``RESCUE-<SITE>``) backhauled over the drones' mesh
radio to the operator ground station (GCS); the GCS may add satellite / LTE internet. A survivor whose phone
joins sees a multilingual captive portal (dashboard/portal/index.html): I'm trapped / I'm safe / message. Only
devices that joined are measured (consent); there is no probe-request or MAC sniffing, devices get random
session tokens and no MAC is ever stored.

Truth / estimate split (RULES rule 9), same pattern as sim/callout.py:
  * ``phone_truth(scenario, seed)``      HIDDEN: who has a phone, battery, language, reaction time. Generator only.
  * ``RadioSim``                         SYNTHETIC observation generator: link budget through rubble (public
                                         heightmap + per-phone loss), shadow fading, association, portal taps.
  * ``Comms``                            mission facade: backhaul, hotspots, RSSI localisation (grid Bayes,
                                         log-distance model, rubble loss as a nuisance), events, state["comms"].
  * ``validate_portal`` / ``PortalStore`` stdlib-only (scripts/serve.py runs on system python): the real
                                         captive-portal API (POST /api/portal), rate limit, inbox, operator replies.

    from sim.comms import Comms
    comms = Comms(scenario, grid, gcs_xy=base_xy, sectors={pid: polygon}, seed=seed)
    events = comms.step(t, flight.poses())          # joined / status / message / located / evidence / left
    state["comms"] = comms.state()

Numbers marked (est.) are estimates; sources in docs/HOTSPOT.md. All phone data in the sim is SYNTHETIC.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import threading
import time

try:                    # numpy only for the simulation / localisation; the portal API is stdlib-only
    import numpy as np
except ImportError:     # pragma: no cover
    np = None

COMMS_COLOR = "#3b82f6"
RNG_STREAM = 123                       # separate RNG stream: default_rng([seed, 123, ...]) never shifts scenarios
PORTAL_URL = "http://10.42.0.1/"

# ----------------------------------------------------------------------------- link budget (docs/HOTSPOT.md §3)
FREQ_MHZ = 2437.0                      # Wi-Fi channel 6
AP_EIRP_DBM = 20.0                     # 100 mW EIRP (TW / EU 2.4 GHz limit; configured value)
AP_RX_GAIN_DBI = 3.0                   # drone omni (est.)
AP_SENS_DBM = -94.0                    # AP receive sensitivity at the lowest rate (est.; datasheets -94..-98)
PHONE_TX_DBM = 15.0                    # phone Wi-Fi conducted power (est. 12-18 dBm)
PHONE_SENS_DBM = -90.0                 # phone receive sensitivity, lowest rate (est.)
BODY_LOSS_DB = 3.0                     # hand / body / orientation (est.)
ASSOC_MARGIN_DB = 6.0                  # downlink AND uplink margin needed to associate (both ways, with fading)
DROP_AFTER_S = 30.0                    # a joined phone is dropped after this long without a usable link
BREAKPOINT_M = 10.0                    # two-slope path loss: free space to 10 m, then exponent N_FAR (est.)
N_FAR = 3.0                            # near-ground clutter on a debris field (est.)
SHADOW_SIGMA_DB = 6.0                  # static shadowing per drone position (rubble multipath, est.)
FAST_SIGMA_DB = 2.0                    # per-reading fading after averaging a burst (est.)
RUBBLE_DB_PER_M_MEDIAN = 20.0          # effective loss along a void-rich rubble path (est.; solid concrete ~100-250)
RUBBLE_DB_PER_M_RANGE = (6.0, 60.0)
REF_RUBBLE_M = 1.0                     # hotspots[].radius_m_est is for a phone under this much rubble
RSSI_PERIOD_S = 5.0                    # one RSSI reading per drone per 5 s for each joined device
LOC_N_EFF = 8.0                        # effective independent looks at a phone (likelihood tempering, est.)
LOC_PEAK_SIGMA_M = 6.0                 # prior: the phone is within ~6 m (1 sigma) of the RSSI peak (est.)
LOC_FLOOR_M = 1.5                      # model-error floor added to the fitted sigma
EVIDENCE_SIGMA_M = 6.0                 # a device position is "evidence" for a sector only when this tight
REPORT_SIGMA_M = 12.0                  # est_xy is published only below this (else null: too vague)
FIT_PERIOD_S = 30.0                    # refit a device at most this often, and only after 3+ new readings

# ----------------------------------------------------------------------------- mesh backhaul (est.)
MESH_FREQ_MHZ = 5800.0                 # Doodle Labs-class 5.8 GHz mesh, away from the 2.4 GHz AP
MESH_TX_DBM = 27.0
MESH_ANT_DBI = 3.0
MESH_NOISE_DBM = -95.0                 # -174 + 10 log10(20 MHz) + 6 dB NF
MESH_MIN_SNR_DB = 5.0
MESH_MAX_MBPS = 40.0                   # practical per-hop throughput, 20 MHz 2x2 (vendor 10 m figure)
MESH_OBSTRUCTION_DB_PER_M = 20.0       # pile in the way ...
MESH_OBSTRUCTION_CAP_DB = 25.0         # ... capped: diffraction over a short obstruction (est.)
GCS_MAST_M = 3.0

# ----------------------------------------------------------------------------- phones (hidden truth, est.)
P_HAS_PHONE = 0.65
P_PHONE_WORKS = 0.85                   # on, not crushed, Wi-Fi enabled
CONSCIOUS_STATES = ("calm", "distressed_tachypnea")   # can look at a phone and tap
REACT_MEAN_S = 45.0                    # notice the RESCUE SSID and join (est.)
TAP_MEAN_S = 20.0
MSG_MEAN_S = 40.0
DEMO_PHONES = {7: ("V2", "V4", "W1")}  # demo seed: these people carry a working phone (draws unchanged)
DRAIN_PCT_PER_H = (5.0, 12.0)          # searching for a cell signal under rubble drains faster (est.)

LANGS = ("zh-TW", "nan-TW", "en", "id", "vi", "tl")
LANG_PRIOR = (("zh-TW", 0.42), ("nan-TW", 0.30), ("id", 0.09), ("vi", 0.09), ("tl", 0.05), ("en", 0.05))
REVIEW = {"en": "source text", "zh-TW": "pending native-speaker review",
          "nan-TW": "pending native-speaker review (Han characters)", "id": "pending native-speaker review",
          "vi": "pending native-speaker review", "tl": "pending native-speaker review"}

# SYNTHETIC portal messages (labelled synthetic in state and docs). Index-aligned across languages.
SYNTH_MESSAGES = {
    "trapped": {
        "en": ["My leg is trapped under a slab. I can breathe.", "Two of us here, my mother is not answering.",
               "I am on the ground floor near the stairs. Water please."],
        "zh-TW": ["我的腿被水泥板壓住了，可以呼吸。", "我們這裡有兩個人，我媽媽沒有回應。", "我在一樓樓梯旁邊，請給我水。"],
        "nan-TW": ["我的跤予水泥板壓著矣，閣會當喘氣。", "阮遮有兩个人，阮老母無應。", "我佇一樓樓梯邊，拜託予我水。"],
        "id": ["Kaki saya terjepit di bawah beton. Saya bisa bernapas.", "Kami berdua di sini, ibu saya tidak menjawab.",
               "Saya di lantai dasar dekat tangga. Tolong air."],
        "vi": ["Chân tôi bị kẹt dưới tấm bê tông. Tôi vẫn thở được.", "Chúng tôi có hai người, mẹ tôi không trả lời.",
               "Tôi ở tầng trệt gần cầu thang. Xin nước."],
        "tl": ["Naipit ang binti ko sa ilalim ng semento. Nakakahinga ako.",
               "Dalawa kami rito, hindi sumasagot ang nanay ko.", "Nasa ground floor ako malapit sa hagdan. Tubig po."],
    },
    "safe": {
        "en": ["I am outside and safe. My neighbour from the 5th floor is missing."],
        "zh-TW": ["我在外面，平安。五樓的鄰居不見了。"],
        "nan-TW": ["我佇外口，平安。五樓的厝邊揣無人。"],
        "id": ["Saya di luar dan aman. Tetangga saya dari lantai 5 hilang."],
        "vi": ["Tôi ở bên ngoài và an toàn. Hàng xóm ở tầng 5 bị mất tích."],
        "tl": ["Nasa labas ako at ligtas. Nawawala ang kapitbahay ko sa ika-5 palapag."],
    },
}
# Deterministic offline fallback: exact phrasebook first, then a keyword gloss (no LLM, no network).
PHRASEBOOK = {txt: SYNTH_MESSAGES[k]["en"][i] for k in SYNTH_MESSAGES for lng in LANGS
              for i, txt in enumerate(SYNTH_MESSAGES[k][lng]) if lng != "en"}
KEYWORDS = {   # substring -> English gloss (lower-cased match); small, vetted, deliberately conservative
    "zh-TW": {"壓住": "pinned", "受傷": "injured", "流血": "bleeding", "水": "water", "呼吸": "breathing",
              "兩個人": "two people", "三個人": "three people", "小孩": "child", "樓梯": "stairs", "平安": "safe",
              "救命": "help", "痛": "pain", "不見": "missing"},
    "nan-TW": {"壓著": "pinned", "著傷": "injured", "流血": "bleeding", "水": "water", "喘氣": "breathing",
               "兩个人": "two people", "囡仔": "child", "樓梯": "stairs", "平安": "safe", "救命": "help",
               "疼": "pain", "揣無": "missing"},
    "id": {"terjepit": "pinned", "luka": "injured", "darah": "bleeding", "air": "water", "napas": "breathing",
           "berdua": "two people", "anak": "child", "tangga": "stairs", "aman": "safe", "tolong": "help",
           "sakit": "pain", "hilang": "missing"},
    "vi": {"kẹt": "pinned", "bị thương": "injured", "chảy máu": "bleeding", "nước": "water", "thở": "breathing",
           "hai người": "two people", "con": "child", "cầu thang": "stairs", "an toàn": "safe", "cứu": "help",
           "đau": "pain", "mất tích": "missing"},
    "tl": {"naipit": "pinned", "sugatan": "injured", "dugo": "bleeding", "tubig": "water", "hinga": "breathing",
           "dalawa": "two people", "bata": "child", "hagdan": "stairs", "ligtas": "safe", "tulong": "help",
           "masakit": "pain", "nawawala": "missing"},
}

# Operator replies: vetted templates only (same rule as the loudspeaker, sim/guidance.py).
REPLY_TEMPLATES = {
    "help_coming": {
        "en": "Help is coming. Your message reached the rescue team.",
        "zh-TW": "救援人員正在趕來。救援隊已收到你的訊息。",
        "nan-TW": "救援的人當咧來矣。救難隊有收著你的訊息。",
        "id": "Bantuan sedang datang. Pesan Anda sudah diterima tim penyelamat.",
        "vi": "Cứu hộ đang đến. Đội cứu hộ đã nhận được tin nhắn của bạn.",
        "tl": "Paparating na ang tulong. Natanggap ng rescue team ang mensahe mo.",
    },
    "stay_put": {
        "en": "Stay where you are and conserve your phone battery.",
        "zh-TW": "請留在原地，節省手機電量。",
        "nan-TW": "請留佇原所在，儉手機的電。",
        "id": "Tetap di tempat Anda dan hemat baterai ponsel Anda.",
        "vi": "Hãy ở yên tại chỗ và tiết kiệm pin điện thoại.",
        "tl": "Manatili ka kung nasaan ka at tipirin ang baterya ng telepono mo.",
    },
    "tap_metal": {
        "en": "If you can, tap a rhythm on metal or a pipe so we can hear you.",
        "zh-TW": "如果可以，請有節奏地敲打金屬或水管，讓我們聽到你。",
        "nan-TW": "若會使，請照節奏敲鐵仔抑是水管，予阮聽會著。",
        "id": "Jika bisa, ketuk berirama pada logam atau pipa agar kami bisa mendengar Anda.",
        "vi": "Nếu có thể, hãy gõ nhịp lên kim loại hoặc đường ống để chúng tôi nghe thấy bạn.",
        "tl": "Kung kaya mo, kumatok nang may ritmo sa metal o tubo para marinig ka namin.",
    },
    "head_to_light": {
        "en": "If you can move safely, head toward the blue light or the exit.",
        "zh-TW": "如果你能安全移動，請往藍色燈光或出口的方向走。",
        "nan-TW": "你若會當安全振動，請向藍色的光抑是出口行。",
        "id": "Jika Anda bisa bergerak dengan aman, menujulah ke lampu biru atau pintu keluar.",
        "vi": "Nếu bạn có thể di chuyển an toàn, hãy đi về phía đèn xanh dương hoặc lối ra.",
        "tl": "Kung ligtas kang makagagalaw, pumunta sa asul na ilaw o sa labasan.",
    },
}
BANNED = re.compile(r"\b(confirmed|survivors? found|person found|we found you)\b", re.I)


class CommsError(ValueError):
    pass


# ============================================================================= link model (pure functions)
def path_loss_db(d_m, f_mhz=FREQ_MHZ):
    """Two-slope log-distance path loss (dB): free space to BREAKPOINT_M, exponent N_FAR beyond. Monotone in d."""
    d = np.maximum(np.asarray(d_m, float), 1.0)
    fs = 20 * np.log10(np.minimum(d, BREAKPOINT_M)) + 20 * np.log10(f_mhz) - 27.55
    far = 10 * N_FAR * np.log10(np.maximum(d / BREAKPOINT_M, 1.0))
    out = fs + far
    return float(out) if out.ndim == 0 else out


def link_margins(d_m, rubble_db, fade_db=0.0):
    """(downlink margin at the phone, uplink margin at the AP), dB. Reciprocal channel: same loss both ways."""
    loss = path_loss_db(d_m) + rubble_db + BODY_LOSS_DB - fade_db
    down = AP_EIRP_DBM - loss - PHONE_SENS_DBM
    up = PHONE_TX_DBM + AP_RX_GAIN_DBI - loss - AP_SENS_DBM
    return down, up


def uplink_rssi(d_m, rubble_db, fade_db=0.0):
    """RSSI (dBm) of the phone's frames at the drone AP."""
    return PHONE_TX_DBM + AP_RX_GAIN_DBI - BODY_LOSS_DB - path_loss_db(d_m) - rubble_db + fade_db


def associates(d_m, rubble_db, fade_db=0.0, margin=ASSOC_MARGIN_DB):
    down, up = link_margins(d_m, rubble_db, fade_db)
    return bool(down >= margin and up >= margin)


def coverage_radius_m(z_m, rubble_db=RUBBLE_DB_PER_M_MEDIAN * REF_RUBBLE_M, margin=ASSOC_MARGIN_DB):
    """Horizontal radius (m) within which a phone under ``rubble_db`` of loss associates with an AP at height z."""
    budget = min(AP_EIRP_DBM - PHONE_SENS_DBM, PHONE_TX_DBM + AP_RX_GAIN_DBI - AP_SENS_DBM) \
        - BODY_LOSS_DB - rubble_db - margin
    lo, hi = 1.0, 5000.0
    if path_loss_db(lo) > budget:
        return 0.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if path_loss_db(mid) <= budget else (lo, mid)
    return float(math.sqrt(max(lo * lo - z_m * z_m, 0.0)))


def mesh_link(a_xyz, b_xyz, grid=None):
    """(rssi_dbm, mbps) of one mesh hop; mbps 0 when the SNR is below MESH_MIN_SNR_DB. Piles in the way (public
    heightmap) add MESH_OBSTRUCTION_DB_PER_M along the blocked part of the line."""
    a, b = np.asarray(a_xyz, float), np.asarray(b_xyz, float)
    d = float(np.linalg.norm(a - b))
    obst = 0.0
    if grid is not None:
        from .sensor_model import rubble_path
        obst = min(MESH_OBSTRUCTION_CAP_DB, MESH_OBSTRUCTION_DB_PER_M * rubble_path(grid, a, b, n=32))
    rssi = MESH_TX_DBM + 2 * MESH_ANT_DBI - path_loss_db(d, MESH_FREQ_MHZ) - obst
    snr = rssi - MESH_NOISE_DBM
    if snr < MESH_MIN_SNR_DB:
        return round(rssi, 1), 0.0
    mbps = min(MESH_MAX_MBPS, 0.5 * 20.0 * math.log2(1 + 10 ** (snr / 10)))
    return round(rssi, 1), round(mbps, 1)


def backhaul_routes(nodes, gcs_xyz, grid=None):
    """Multi-hop routes to the GCS. nodes: {drone_id: xyz}. Shared-channel relay: end-to-end throughput of a path
    is 1 / sum(1 / hop_mbps); each drone takes the path that maximises it (Dijkstra on sum of 1/mbps).
    Returns {drone_id: {"next": id|"gcs"|None, "rssi_dbm", "mbps" (hop), "e2e_mbps", "hops"}}."""
    ids = list(nodes)
    pos = dict(nodes, gcs=gcs_xyz)
    hop = {}
    for i, a in enumerate(ids + ["gcs"]):
        for b in (ids + ["gcs"])[i + 1:]:
            hop[(a, b)] = hop[(b, a)] = mesh_link(pos[a], pos[b], grid)
    cost = {"gcs": 0.0}
    nxt = {"gcs": None}
    todo = set(ids)
    while todo:                                        # Dijkstra from the GCS outwards
        best = None
        for u in todo:
            for v, c in cost.items():
                r = hop[(u, v)][1]
                if r > 0 and (best is None or c + 1.0 / r < best[0]):
                    best = (c + 1.0 / r, u, v)
        if best is None:
            break
        c, u, v = best
        cost[u], nxt[u] = c, v
        todo.discard(u)
    out = {}
    for u in ids:
        if u not in nxt:
            out[u] = {"next": None, "rssi_dbm": None, "mbps": 0.0, "e2e_mbps": 0.0, "hops": 0}
            continue
        n, v = 0, u
        while v != "gcs":
            v, n = nxt[v], n + 1
        rssi, mbps = hop[(u, nxt[u])]
        out[u] = {"next": nxt[u], "rssi_dbm": rssi, "mbps": mbps, "e2e_mbps": round(1.0 / cost[u], 1), "hops": n}
    return out


# ============================================================================= hidden truth (generator only)
def _pick(rng, prior):
    names, w = zip(*prior)
    return names[int(rng.choice(len(names), p=np.asarray(w) / sum(w)))]


def phone_truth(scenario, seed=None, grid=None):
    """HIDDEN per-person phone truth (victims + open-ground bystanders). Separate RNG stream
    default_rng([seed, 123]) so existing scenarios never change. Hook for sim/scenario.make_scenario:
    ``sc.truth["phones"] = phone_truth(sc, seed)`` (see docs/HOTSPOT.md Requests)."""
    seed = scenario.seed if seed is None else seed
    rng = np.random.default_rng([int(seed), RNG_STREAM])
    truth = scenario.truth or {}
    people = [dict(v, kind="buried") for v in truth.get("victims", [])]
    try:                                               # the speaker's bystanders, so both stories agree
        from .callout import walking_wounded
        people += [dict(b, kind="surface") for b in walking_wounded(scenario, seed)]
    except Exception:                                  # noqa: BLE001  (bystanders are optional)
        pass
    out = []
    for k, p in enumerate(people):
        has = bool(rng.uniform() < P_HAS_PHONE)
        works = bool(rng.uniform() < P_PHONE_WORKS)
        if p.get("id") in DEMO_PHONES.get(int(seed), ()):
            has = works = True
        state = (p.get("physio") or {}).get("state") or p.get("state")
        aware = p["kind"] == "surface" or state in CONSCIOUS_STATES
        lo, hi = RUBBLE_DB_PER_M_RANGE
        out.append({
            "person": p.get("id", f"P{k + 1}"), "kind": p["kind"], "pile": p.get("pile"),
            "xy": [float(p["xy"][0]), float(p["xy"][1])], "depth": float(p.get("depth") or 0.0),
            "z": p.get("z"),
            "has_phone": has, "works": works, "aware": bool(aware),
            "battery_pct": float(rng.uniform(8, 90)), "drain_pct_h": float(rng.uniform(*DRAIN_PCT_PER_H)),
            "rubble_db_per_m": float(np.clip(RUBBLE_DB_PER_M_MEDIAN * np.exp(rng.normal(0, 0.45)), lo, hi)),
            "lang": p.get("lang") or _pick(rng, LANG_PRIOR),
            "react_s": float(rng.exponential(REACT_MEAN_S)), "tap_s": float(rng.exponential(TAP_MEAN_S)),
            "msg_s": float(rng.exponential(MSG_MEAN_S)), "shares_battery": bool(rng.uniform() < 0.6),
            "sends_msg": bool(rng.uniform() < 0.7), "msg_idx": int(rng.integers(0, 3)),
            "token": "ph-" + bytes(rng.integers(0, 256, 3, dtype=np.uint8)).hex(),
        })
    return out


def attach_phone_truth(scenario, seed=None):
    """One-line hook target for sim/scenario.py: adds truth["phones"] (hidden) and returns the scenario."""
    scenario.truth["phones"] = phone_truth(scenario, seed)
    return scenario


class RadioSim:
    """SYNTHETIC observation generator. The only reader of the phone truth. Produces what real APs and the real
    portal would: association (a phone joined), RSSI of joined devices' frames at each drone, portal taps."""

    def __init__(self, phones, grid, seed=0):
        self.phones, self.grid, self.seed = phones, grid, int(seed)
        for k, ph in enumerate(self.phones):
            x, y = ph["xy"]
            surf = float(grid.height_at(x, y)) if grid is not None else 0.0
            ph["xyz"] = [x, y, float(ph["z"]) if ph.get("z") is not None else surf - ph["depth"] + 0.1]
            ph["_k"] = k
        self.joined = {}        # token -> {"t", "drone", "last_ok", "last_rssi"} while associated
        self.sent = {}          # token -> {"status": bool, "msg": bool, "t0": first join}: survives re-joins
        self.covered_since = {}
        self._fast = np.random.default_rng([self.seed, RNG_STREAM, 7])

    def _on(self, ph, t):
        return ph["has_phone"] and ph["works"] and ph["battery_pct"] - ph["drain_pct_h"] * t / 3600.0 > 0

    def battery(self, ph, t):
        return max(0, int(round(ph["battery_pct"] - ph["drain_pct_h"] * t / 3600.0)))

    def _shadow(self, ph, drone_id, xyz):
        """Static shadowing for this phone seen from this drone position (3 m bins): same geometry, same fade."""
        b = [int(math.floor(v / 3.0)) + 10_000 for v in xyz]
        dn = int(re.sub(r"\D", "", drone_id) or 0)
        sd = SHADOW_SIGMA_DB if ph["depth"] > 0.05 else SHADOW_SIGMA_DB * 0.6
        return float(np.random.default_rng([self.seed, RNG_STREAM, ph["_k"], dn, *b]).normal(0, sd))

    def link(self, ph, drone_id, xyz):
        """(rssi_at_ap_dbm, downlink_margin, uplink_margin) incl. static shadowing (no fast fading)."""
        from .sensor_model import rubble_path
        d = float(np.linalg.norm(np.asarray(xyz) - np.asarray(ph["xyz"])))
        rub = ph["rubble_db_per_m"] * (rubble_path(self.grid, xyz, ph["xyz"], n=48) if ph["depth"] > 0.05 else 0.0)
        fade = self._shadow(ph, drone_id, xyz)
        down, up = link_margins(d, rub, fade)
        return uplink_rssi(d, rub, fade), down, up

    def observe(self, t, aps):
        """aps: {drone_id: xyz} of hotspots that are on. Returns (events, readings) as the real system would see
        them: events about joined tokens only; readings [{token, drone_id, xyz, dbm, t}] of joined devices."""
        events, readings = [], []
        for ph in self.phones:
            tok = ph["token"]
            if not self._on(ph, t) or not aps:
                if tok in self.joined and t - self.joined[tok]["last_ok"] > DROP_AFTER_S:
                    events.append({"type": "left", "device": tok})
                    del self.joined[tok]
                continue
            links = {did: self.link(ph, did, xyz) for did, xyz in aps.items()}
            ok = {did: lk for did, lk in links.items() if min(lk[1], lk[2]) >= ASSOC_MARGIN_DB}
            j = self.joined.get(tok)
            if j is None:
                if not ok or not ph["aware"]:
                    self.covered_since.pop(tok, None)
                    continue
                t0 = self.covered_since.setdefault(tok, t)
                if t - t0 < ph["react_s"]:
                    continue
                best = max(ok, key=lambda d: ok[d][0])
                j = self.joined[tok] = {"t": t, "drone": best, "last_ok": t, "last_rssi": {}}
                self.sent.setdefault(tok, {"status": False, "msg": False, "t0": t})
                events.append({"type": "joined", "device": tok, "drone_id": best})
            if ok:
                j["last_ok"] = t
                j["drone"] = max(ok, key=lambda d: ok[d][0])
            elif t - j["last_ok"] > DROP_AFTER_S:
                events.append({"type": "left", "device": tok})
                del self.joined[tok]
                continue
            for did, (rssi, _down, up) in links.items():     # any AP that decodes the joined device's frames
                if up >= 0.0 and t - j["last_rssi"].get(did, -1e9) >= RSSI_PERIOD_S:
                    j["last_rssi"][did] = t
                    readings.append({"device": tok, "drone_id": did, "xyz": [round(v, 2) for v in aps[did]],
                                     "dbm": round(rssi + float(self._fast.normal(0, FAST_SIGMA_DB)), 1), "t": t})
            if not ok:
                continue
            sent = self.sent[tok]
            if not sent["status"] and t - sent["t0"] >= ph["tap_s"]:
                sent["status"] = True
                ev = {"type": "portal", "device": tok, "status": "trapped" if ph["kind"] == "buried" else "safe",
                      "lang": ph["lang"]}
                if ph["shares_battery"]:
                    ev["battery_pct"] = self.battery(ph, t)
                events.append(ev)
            if sent["status"] and not sent["msg"] and ph["sends_msg"] and t - sent["t0"] >= ph["tap_s"] + ph["msg_s"]:
                sent["msg"] = True
                kind = "trapped" if ph["kind"] == "buried" else "safe"
                lst = SYNTH_MESSAGES[kind][ph["lang"]]
                events.append({"type": "portal", "device": tok, "lang": ph["lang"],
                               "message": lst[ph["msg_idx"] % len(lst)], "synthetic": True})
        return events, readings


# ============================================================================= localisation (no truth)
def localise(readings, grid=None, depths=(0.0, 0.9, 1.5), depth_prior=(0.25, 0.4, 0.35), res=1.0, half=14.0,
             sigma_db=SHADOW_SIGMA_DB + 1.0, alpha_grid=None, alpha_sigma_ln=0.6, min_bins=4, min_spread_m=4.0,
             max_bins=32, n_line=16, floor_m=LOC_FLOOR_M, n_eff=LOC_N_EFF, peak_sigma_m=LOC_PEAK_SIGMA_M,
             nuisance="profile", mode_r_m=3.0, max_edge_mass=0.05):
    """Grid-Bayes fit of a phone position from RSSI at several drone positions.

    Model (public): dbm = PHONE_TX + AP_GAIN - BODY - PL(d) - alpha * l(drone, cell) + e, with PL the two-slope
    model, l the length of the drone->cell line below the LiDAR heightmap (cell at surface - depth, depth in
    ``depths``: on the surface or buried, both marginalised), alpha (dB/m) an unknown nuisance marginalised on a
    grid, e ~ N(0, sigma_db^2 + fast^2/n) per 3 m position bin (repeated readings from one place are averaged, not
    counted as independent; sigma_db is one dB above the generator's shadowing for model mismatch). Flat prior
    on a +-half window around the strongest position. sigma_m (per axis) includes a floor_m model-error floor.
    Uses the max_bins strongest bins (cost ~ cells x bins x n_line). Returns {"xy", "sigma_m", "n_bins", "map_xy", "peak_xy"}
    or None when the geometry is too thin (< min_bins bins)."""
    if not readings:
        return None
    bins = {}
    for r in readings:
        key = (r["drone_id"],) + tuple(int(math.floor(v / 3.0)) for v in r["xyz"])
        b = bins.setdefault(key, {"xyz": np.zeros(3), "dbm": 0.0, "n": 0})
        b["xyz"] += np.asarray(r["xyz"], float)
        b["dbm"] += r["dbm"]
        b["n"] += 1
    if len(bins) < min_bins:
        return None
    n_all = len(bins)
    bl = sorted(bins.values(), key=lambda b: -b["dbm"] / b["n"])[:max_bins]
    bins = {k: b for k, b in enumerate(bl)}
    P = np.array([b["xyz"] / b["n"] for b in bins.values()])
    if np.ptp(P[:, 0]) < min_spread_m and np.ptp(P[:, 1]) < min_spread_m:
        return None                                    # one place only: a range ring, not a position
    R = np.array([b["dbm"] / b["n"] for b in bins.values()])
    N = np.array([b["n"] for b in bins.values()], float)
    var = sigma_db ** 2 + FAST_SIGMA_DB ** 2 / N
    top = np.argsort(-R)[:5]                           # RSSI peak: power-weighted centroid of the 5 strongest
    pw = 10 ** ((R[top] - R[top].max()) / 10)          # positions; the phone is near where its signal peaks
    c = (P[top, :2] * pw[:, None]).sum(0) / pw.sum()
    xs = np.arange(c[0] - half, c[0] + half + 1e-6, res)
    ys = np.arange(c[1] - half, c[1] + half + 1e-6, res)
    X, Y = np.meshgrid(xs, ys)
    nd = len(depths)
    cx, cy = np.tile(X.ravel(), nd), np.tile(Y.ravel(), nd)
    surf = grid.height_at(cx, cy) if grid is not None else np.zeros_like(cx)
    cz = surf - np.repeat(np.asarray(depths, float), X.size)
    C = np.stack([cx, cy, cz], 1)                     # (K, 3)
    D = np.linalg.norm(C[:, None, :] - P[None, :, :], axis=2)        # (K, B)
    pred0 = PHONE_TX_DBM + AP_RX_GAIN_DBI - BODY_LOSS_DB - path_loss_db(D)
    if grid is not None:
        # samples dense near the phone end, where the rubble is (midpoint rule on a quadratic spacing)
        edges = 1.0 - np.linspace(1.0, 0.0, n_line + 1) ** 2
        mids, dts = 0.5 * (edges[1:] + edges[:-1]), np.diff(edges)
        ts = mids[None, None, :, None]
        pts = P[None, :, None, :] + ts * (C[:, None, None, :] - P[None, :, None, :])   # (K, B, n_line, 3)
        below = pts[..., 2] < grid.height_at(pts[..., 0], pts[..., 1]) - 0.02
        L = (below * dts[None, None, :]).sum(axis=2) * D   # metres of rubble on each line
    else:
        L = np.zeros_like(D)
    alphas = np.arange(0.0, 62.0, 4.0) if alpha_grid is None else np.asarray(alpha_grid, float)
    ll = np.stack([-0.5 * (((R[None, :] - (pred0 - a * L)) ** 2) / var[None, :]).sum(1) for a in alphas], 1)
    ll = ll * min(1.0, n_eff / len(R))                 # tempered: bins share geometry and fading (not independent)
    if alpha_sigma_ln:                                 # population prior on the rubble loss (lognormal, median)
        la = np.log(np.maximum(alphas, 1.0) / RUBBLE_DB_PER_M_MEDIAN)
        ll = ll - 0.5 * (la / alpha_sigma_ln) ** 2 - np.log(np.maximum(alphas, 1.0))
    ll = ll + np.repeat(np.log(np.asarray(depth_prior, float)), X.size)[:, None]
    if peak_sigma_m:                                   # Gaussian prior around the RSSI peak (robust to far modes)
        ll = ll - (0.5 * ((cx - c[0]) ** 2 + (cy - c[1]) ** 2) / peak_sigma_m ** 2)[:, None]
    m = ll.max()
    if nuisance == "profile":                          # best (alpha, depth) per cell: no Occam pull towards cells
        w = np.exp(ll - m).max(1).reshape(nd, -1).max(0)   # whose fit is insensitive to the nuisance
    else:
        w = np.exp(ll - m).sum(1).reshape(nd, -1).sum(0)   # marginalise alpha and depth
    w /= w.sum()
    cx, cy = X.ravel(), Y.ravel()
    edge = (np.abs(cx - c[0]) > half - 2 * res) | (np.abs(cy - c[1]) > half - 2 * res)
    if float(w[edge].sum()) > max_edge_mass:
        return None                                    # the phone may be outside the measured area: no estimate
    k = int(np.argmax(w))                              # the likelihood is sharp at the true cell but has a long
    near = np.hypot(cx - cx[k], cy - cy[k]) <= mode_r_m  # flat tail, so the posterior MEAN is dragged: report the
    wn = w * near                                      # centroid of the main mode, and its spread over the whole
    mx, my = float((wn * cx).sum() / wn.sum()), float((wn * cy).sum() / wn.sum())   # posterior as sigma (honest)
    vx = float((w * (cx - mx) ** 2).sum()) + res ** 2 / 12
    vy = float((w * (cy - my) ** 2).sum()) + res ** 2 / 12
    return {"xy": [round(mx, 2), round(my, 2)], "sigma_m": round(math.sqrt(0.5 * (vx + vy) + floor_m ** 2), 2),
            "n_bins": n_all, "map_xy": [round(float(cx[k]), 2), round(float(cy[k]), 2)],
            "peak_xy": [round(float(c[0]), 2), round(float(c[1]), 2)]}


def point_in_poly(x, y, poly):
    inside, n = False, len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1:
            inside = not inside
    return inside


def sector_of(xy, sectors, near_m=6.0):
    """Sector (pile) id containing xy (polygon), else the nearest sector centre within near_m, else None."""
    if xy is None or not sectors:
        return None
    best = None
    for sid, s in sectors.items():
        poly = s.get("polygon") if isinstance(s, dict) else None
        if poly and len(poly) >= 3 and point_in_poly(xy[0], xy[1], poly):
            return sid
        c = (s.get("xy") if isinstance(s, dict) else s) or (poly and [sum(p[0] for p in poly) / len(poly),
                                                                       sum(p[1] for p in poly) / len(poly)])
        d = math.hypot(xy[0] - c[0], xy[1] - c[1])
        if d <= near_m and (best is None or d < best[0]):
            best = (d, sid)
    return best[1] if best else None


# ============================================================================= translation (offline-first)
TRANSLATE_SCHEMA = {"type": "object", "properties": {"translation": {"type": "string"}, "lang": {"type": "string"}},
                    "required": ["translation"]}
TRANSLATE_SYSTEM = ("You translate short messages typed by people trapped after a building collapse into plain "
                    "English for a rescue team. Translate faithfully, keep every number, add nothing. Taiwanese "
                    "Hokkien (nan-TW) may be in Han characters or Tai-lo. Return JSON only.")


def fallback_translate(text, lang):
    """Deterministic, offline: exact phrasebook match, else a keyword gloss ("keywords: pinned, water"), else None."""
    if not text:
        return None
    if lang == "en":
        return text
    if text in PHRASEBOOK:
        return PHRASEBOOK[text]
    low = text.lower()
    hits = [g for k, g in (KEYWORDS.get(lang) or {}).items() if k in low]
    if not hits:                                       # unknown language tag: try every table
        for tbl in KEYWORDS.values():
            hits += [g for k, g in tbl.items() if k in low and g not in hits]
    nums = re.findall(r"\d+", text)
    if hits or nums:
        return "keywords: " + ", ".join(dict.fromkeys(hits + [f"number {n}" for n in nums]))
    return None


def translate_message(text, lang, use_llm=False, llm_call=None):
    """(translation, by): by = gemini | fallback | none. LLM only when use_llm and a provider answers; the LLM
    answer must keep the numbers of the source, else the fallback is used."""
    if lang == "en":
        return text, "none"
    if use_llm:
        try:
            if llm_call is None:
                from agents import llm
                llm_call = llm.call_json if llm.available() else None
            if llm_call is not None:
                out = llm_call(f"Language: {lang}. Message: {json.dumps(text, ensure_ascii=False)}",
                               TRANSLATE_SCHEMA, system=TRANSLATE_SYSTEM) or {}
                tr = out.get("translation") if isinstance(out, dict) else None
                if isinstance(tr, str) and tr.strip() and \
                        sorted(re.findall(r"\d+", text)) == sorted(re.findall(r"\d+", tr)):
                    return tr.strip()[:400], "gemini"
        except Exception:                              # noqa: BLE001  (never break the mission)
            pass
    tr = fallback_translate(text, lang)
    return tr, ("fallback" if tr else "none")


def reply_text(template, lang):
    if template not in REPLY_TEMPLATES:
        raise CommsError(f"unknown reply template {template!r}")
    return REPLY_TEMPLATES[template].get(lang) or REPLY_TEMPLATES[template]["en"]


# ============================================================================= mission facade
class Comms:
    """Hotspots + backhaul + devices for one mission. Never reads truth itself: the RadioSim generator owns it,
    exactly as the acoustic / call-out generators do. Integration: see docs/HOTSPOT.md Requests."""

    def __init__(self, scenario, grid, gcs_xy, sectors=None, seed=0, site="WEIGUAN", internet=True,
                 gcs_uplink="satellite", portal_url=PORTAL_URL, use_llm=False, llm_call=None, phones=None):
        self.grid, self.seed = grid, int(seed)
        self.gcs_xy = [float(gcs_xy[0]), float(gcs_xy[1])]
        gz = float(grid.height_at(*self.gcs_xy)) if grid is not None else 0.0
        self.gcs_xyz = [self.gcs_xy[0], self.gcs_xy[1], gz + GCS_MAST_M]
        self.sectors = sectors or {}
        self.ssid = "RESCUE-" + re.sub(r"[^A-Z0-9]+", "", str(site).upper())[:20]
        self.internet, self.gcs_uplink, self.portal_url = bool(internet), gcs_uplink, portal_url
        self.use_llm, self.llm_call = use_llm, llm_call
        if phones is None:
            phones = (getattr(scenario, "truth", None) or {}).get("phones")
            if phones is None:
                phones = phone_truth(scenario, self.seed)
        self._radio = RadioSim([dict(p) for p in phones], grid, self.seed)   # generator (truth side)
        self.devices = {}       # token -> contract device dict
        self._readings = {}     # token -> list of readings
        self._fit_t = {}
        self._evidence = set()
        self._links, self._hot = [], []
        self.submissions = 0
        self._portal_seen = set()
        self._replies_seen = set()
        self._portal_rev = None
        self.t = 0.0

    # -------------------------------------------------------------- network
    @staticmethod
    def _ap_on(d):
        return d.get("state") != "docked" and float(d.get("battery", 1.0)) > 0.05

    def _device(self, tok, t, live=False):
        d = self.devices.get(tok)
        if d is None:
            d = self.devices[tok] = {"id": tok, "first_seen": round(t, 1), "last_seen": round(t, 1),
                                     "est_xy": None, "est_sigma_m": None, "pile": None, "rssi": [],
                                     "status": "unknown", "battery_pct": None, "messages": [], "replies": []}
            if live:
                d["live"] = True
        return d

    def step(self, t, drones):
        """drones: flight.poses() [{id, pose: [x, y, z, ...], state, battery}]. Returns events (dicts with t, type,
        device, ...): joined | status | message | located | evidence | left."""
        self.t = float(t)
        aps = {d["id"]: [float(v) for v in d["pose"][:3]] for d in drones if self._ap_on(d)}
        routes = backhaul_routes(aps, self.gcs_xyz, self.grid) if aps else {}
        self._links = [{"from": did, "to": r["next"], "rssi_dbm": r["rssi_dbm"], "mbps": r["mbps"]}
                       for did, r in routes.items() if r["next"] is not None]
        raw, readings = self._radio.observe(self.t, aps)
        out = []
        for r in readings:
            d = self._device(r["device"], t)
            rl = self._readings.setdefault(r["device"], [])
            rl.append(r)
            del rl[:-3000]
            d["rssi"] = (d["rssi"] + [{"drone_id": r["drone_id"], "dbm": r["dbm"], "t": round(t, 1)}])[-12:]
            d["last_seen"] = round(t, 1)
        for ev in raw:
            tok = ev["device"]
            d = self._device(tok, t)
            if ev["type"] == "joined":
                out.append({"t": round(t, 1), "type": "joined", "device": tok, "drone_id": ev["drone_id"]})
            elif ev["type"] == "left":
                out.append({"t": round(t, 1), "type": "left", "device": tok})
            elif ev["type"] == "portal":
                out += self._portal_event(d, ev, t)
        for tok, d in self.devices.items():
            n_r = len(self._readings.get(tok, ()))
            if d.get("live") or not n_r or t - self._fit_t.get(tok, (-1e9, 0))[0] < FIT_PERIOD_S \
                    or n_r - self._fit_t.get(tok, (0, -99))[1] < 3:
                continue
            self._fit_t[tok] = (t, n_r)
            fit = localise(self._readings[tok], self.grid)
            if fit is None or fit["sigma_m"] > REPORT_SIGMA_M:
                continue
            first = d["est_xy"] is None
            d["est_xy"], d["est_sigma_m"] = fit["xy"], fit["sigma_m"]
            d["pile"] = sector_of(fit["xy"], self.sectors)
            if first:
                out.append({"t": round(t, 1), "type": "located", "device": tok, "est_xy": fit["xy"],
                            "sigma_m": fit["sigma_m"], "pile": d["pile"]})
            out += self._maybe_evidence(d, t)
        self._hot = []
        for did, xyz in aps.items():
            gz = float(self.grid.height_at(xyz[0], xyz[1])) if self.grid is not None else 0.0
            self._hot.append({"drone_id": did, "ssid": self.ssid, "on": True, "xy": [round(xyz[0], 2), round(xyz[1], 2)],
                              "z": round(xyz[2], 2),
                              "clients": sum(1 for j in self._radio.joined.values() if j["drone"] == did),
                              "radius_m_est": round(coverage_radius_m(max(xyz[2] - gz, 0.5)), 1)})
        for d in drones:
            if not self._ap_on(d):
                self._hot.append({"drone_id": d["id"], "ssid": self.ssid, "on": False,
                                  "xy": [round(float(d["pose"][0]), 2), round(float(d["pose"][1]), 2)],
                                  "z": round(float(d["pose"][2]), 2), "clients": 0, "radius_m_est": 0.0})
        return out

    def _portal_event(self, d, ev, t):
        out = []
        self.submissions += 1
        d["last_seen"] = round(t, 1)
        if ev.get("battery_pct") is not None:
            d["battery_pct"] = int(ev["battery_pct"])
        if ev.get("status") in ("trapped", "safe"):
            if d["status"] != ev["status"]:
                d["status"] = ev["status"]
                out.append({"t": round(t, 1), "type": "status", "device": d["id"], "status": ev["status"],
                            "battery_pct": d["battery_pct"]})
                out += self._maybe_evidence(d, t)
        if ev.get("message"):
            if ev.get("by"):                           # live portal: translated by the server at submission time
                tr, by = ev.get("translation"), ev["by"]
            else:
                tr, by = translate_message(ev["message"], ev.get("lang", "en"), self.use_llm, self.llm_call)
            m = {"t": round(t, 1), "text": ev["message"][:280], "lang": ev.get("lang", "en"), "translation": tr, "by": by}
            if ev.get("synthetic"):
                m["synthetic"] = True
            d["messages"].append(m)
            out.append({"t": round(t, 1), "type": "message", "device": d["id"], **{k: m[k] for k in
                                                                                    ("text", "lang", "translation", "by")}})
        return out

    def _maybe_evidence(self, d, t):
        """A located device inside a sector is new evidence (the sector may be re-searched). Once per (device,
        sector, status). Candidate region only, never a confirmed person."""
        if d["est_xy"] is None or d["pile"] is None or d["status"] == "safe" or d["est_sigma_m"] > EVIDENCE_SIGMA_M:
            return []
        key = (d["id"], d["pile"], d["status"])
        if key in self._evidence:
            return []
        self._evidence.add(key)
        return [{"t": round(t, 1), "type": "evidence", "device": d["id"], "pile": d["pile"], "status": d["status"],
                 "est_xy": d["est_xy"], "sigma_m": d["est_sigma_m"]}]

    # -------------------------------------------------------------- live portal (dashboard/data/portal.json)
    def ingest_portal(self, path, t=None):
        """Fold REAL submissions (scripts/serve.py POST /api/portal) into the device list as live devices (no
        position: a judge's phone is not in the simulated site). Returns events for new submissions."""
        t = self.t if t is None else float(t)
        try:
            st = os.stat(path)
        except OSError:
            return []
        if self._portal_rev == st.st_mtime_ns:
            return []
        self._portal_rev = st.st_mtime_ns
        doc = PortalStore.read(path)
        out = []
        for s in doc.get("submissions", []):
            if s.get("id") in self._portal_seen or not isinstance(s.get("device"), str):
                continue
            self._portal_seen.add(s.get("id"))
            d = self._device(s["device"], t, live=True)
            ev = {"status": s.get("status") if s.get("status") in ("trapped", "safe") else None,
                  "battery_pct": s.get("battery_pct"), "message": s.get("text") or None, "lang": s.get("lang", "en"),
                  "translation": s.get("translation"), "by": s.get("by")}
            out += [dict(e, live=True) for e in self._portal_event(d, ev, t)]
        return out

    def ingest_replies(self, path, t=None):
        """Fold queued operator replies (POST /api/comms/reply -> comms_replies.json) into devices[].replies.
        Items were validated by the server (vetted template, or free text with confirmed_by). Returns the count."""
        t = self.t if t is None else float(t)
        doc = _read_json(path, PortalStore.EMPTY_REPLIES)
        n = 0
        for it in doc["items"]:
            if it.get("id") in self._replies_seen or it.get("device") not in self.devices:
                continue
            if not it.get("template") and not it.get("confirmed_by"):
                continue                               # never deliver unconfirmed free text
            self._replies_seen.add(it.get("id"))
            self.devices[it["device"]]["replies"].append(
                {"t": round(t, 1), "template": it.get("template") or "free_text", "lang": it.get("lang"),
                 "confirmed_by": it.get("confirmed_by") or "operator", "text": it.get("text")})
            n += 1
        return n

    # -------------------------------------------------------------- operator replies (vetted)
    def reply(self, device_id, template, operator, t=None, lang=None):
        """Send a vetted template to a device. The language defaults to the device's last message / portal lang."""
        if not operator or not str(operator).strip():
            raise CommsError("operator name required (every reply is attributed)")
        d = self.devices.get(device_id)
        if d is None:
            raise CommsError(f"unknown device {device_id}")
        lang = lang or (d["messages"][-1]["lang"] if d["messages"] else "en")
        if lang not in LANGS:
            raise CommsError(f"unsupported language {lang!r}")
        text = reply_text(template, lang)
        r = {"t": round(self.t if t is None else float(t), 1), "template": template, "lang": lang,
             "confirmed_by": str(operator)[:40], "text": text}
        d["replies"].append(r)
        return r

    def draft_free_text(self, device_id, text, lang, llm_call=None):
        """Machine translation of operator free text for a device: returned as a DRAFT only (status
        pending_approval); nothing reaches the phone until ``confirm_draft``. Without an LLM: no draft."""
        if not text or len(text) > 280 or BANNED.search(text):
            raise CommsError("operator text must be 1-280 characters and make no 'found/confirmed' claims")
        if lang == "en":
            return {"device": device_id, "lang": lang, "text": text, "source_en": text, "status": "pending_approval",
                    "by": "operator"}
        try:
            if llm_call is None:
                from agents import llm
                llm_call = llm.call_json if llm.available() else None
        except Exception:                              # noqa: BLE001
            llm_call = None
        if llm_call is None:
            return None
        out = llm_call(f"Translate into {lang}. Message: {json.dumps(text, ensure_ascii=False)}",
                       {"type": "object", "properties": {"text": {"type": "string"}}, "required": ["text"]},
                       system="Translate a rescue team's short message faithfully; keep numbers; add nothing. JSON.")
        tr = (out or {}).get("text") if isinstance(out, dict) else None
        if not isinstance(tr, str) or not tr.strip() or BANNED.search(tr) or \
                sorted(re.findall(r"\d+", text)) != sorted(re.findall(r"\d+", tr)):
            return None
        return {"device": device_id, "lang": lang, "text": tr.strip()[:400], "source_en": text,
                "status": "pending_approval", "by": "gemini"}

    def confirm_draft(self, draft, operator, t=None):
        if not draft or draft.get("status") != "pending_approval":
            raise CommsError("not a pending draft")
        if not operator or not str(operator).strip():
            raise CommsError("operator name required to confirm a translation")
        d = self.devices.get(draft["device"])
        if d is None:
            raise CommsError("unknown device")
        r = {"t": round(self.t if t is None else float(t), 1), "template": "free_text", "lang": draft["lang"],
             "confirmed_by": str(operator)[:40], "text": draft["text"]}
        draft["status"] = "sent"
        d["replies"].append(r)
        return r

    # -------------------------------------------------------------- state (contract v1.4)
    def state(self):
        return {
            "backhaul": {"kind": "mesh", "gcs_xy": [round(v, 2) for v in self.gcs_xy], "internet": self.internet,
                         "links": list(self._links)},
            "hotspots": list(self._hot),
            "devices": [dict(d) for d in sorted(self.devices.values(), key=lambda d: d["first_seen"])],
            "portal": {"url": self.portal_url, "submissions": self.submissions},
        }


# ============================================================================= real captive-portal API (stdlib)
PORTAL_SCHEMA = "larpverse.portal/v1"
REPLIES_SCHEMA = "larpverse.comms_replies/v1"
PORTAL_STATUSES = ("trapped", "safe", "message")
MAX_MSG = 280
MAX_CONTACT = 60
MAX_BODY = 4096
MAX_ITEMS = 500
_TOKEN_RE = re.compile(r"^[a-z0-9]{6,32}$")
_DEVICE_RE = re.compile(r"^ph-[a-z0-9]{4,32}$")
_CTRL_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f]")
_T_RE = re.compile(rb'"t"\s*:\s*(-?[0-9.]+)')


class PortalError(ValueError):
    pass


def device_id(token):
    """Dashboard / state id of a portal token: 'ph-' + first 8 characters (the full token stays on the phone and
    in portal.json, so only that phone can read its inbox)."""
    return "ph-" + str(token)[:8]


def _clean(s, n):
    if s is None:
        return ""
    if not isinstance(s, str):
        raise PortalError("text fields must be strings")
    s = _CTRL_RE.sub("", s).strip()
    if len(s) > n:
        raise PortalError(f"text longer than {n} characters")
    return s


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v == v


def validate_portal(body):
    """Pure validation of one POST /api/portal body. Returns the record to store (no IP, no MAC, nothing the
    phone did not type or explicitly opt in to). Raises PortalError.
    body: {device: token, status: trapped|safe|message, lang?, message?, battery_pct?, people?,
           location?: {lat, lon, acc_m}, family?: {contact, text}}"""
    if not isinstance(body, dict):
        raise PortalError("body must be a JSON object")
    status = body.get("status")
    if status not in PORTAL_STATUSES:
        raise PortalError("status must be trapped, safe or message")
    tok = body.get("device")
    if not isinstance(tok, str) or not _TOKEN_RE.match(tok):
        raise PortalError("device must be a random token of 6-32 lowercase letters / digits")
    lang = body.get("lang") if body.get("lang") in LANGS else "und"
    msg = _clean(body.get("message"), MAX_MSG)
    bat = body.get("battery_pct")
    if bat is not None:
        if not _num(bat) or not 0 <= bat <= 100:
            raise PortalError("battery_pct must be 0-100")
        bat = int(round(bat))
    ppl = body.get("people")
    if ppl is not None and (isinstance(ppl, bool) or not isinstance(ppl, int) or not 1 <= ppl <= 50):
        raise PortalError("people must be 1-50")
    loc = body.get("location")
    if loc is not None:
        if not isinstance(loc, dict) or not (_num(loc.get("lat")) and _num(loc.get("lon"))):
            raise PortalError("location must be {lat, lon, acc_m}")
        if not (-90 <= loc["lat"] <= 90 and -180 <= loc["lon"] <= 180):
            raise PortalError("location out of range")
        acc = loc.get("acc_m")
        loc = {"lat": round(float(loc["lat"]), 6), "lon": round(float(loc["lon"]), 6),
               "acc_m": round(float(acc), 1) if _num(acc) and 0 <= acc <= 1e6 else None}
    fam = body.get("family")
    if fam is not None:
        if not isinstance(fam, dict):
            raise PortalError("family must be {contact, text}")
        contact, ftext = _clean(fam.get("contact"), MAX_CONTACT), _clean(fam.get("text"), MAX_MSG)
        fam = {"contact": contact, "text": ftext, "status": "queued_at_gcs"} if contact and ftext else None
    if status == "message" and not msg and fam is None:
        raise PortalError("message is empty")
    return {"token": tok, "status": status, "lang": lang, "text": msg, "battery_pct": bat,
            "people": ppl, "location": loc, "family": fam}


def validate_reply(body):
    """Pure validation of POST /api/comms/reply {device, template | text, lang?, confirmed_by?}. Vetted templates
    are accepted directly; free text only with confirmed_by (an operator confirmed it, e.g. a machine translation).
    Returns the queue item (without id / t). Raises PortalError."""
    if not isinstance(body, dict):
        raise PortalError("body must be a JSON object")
    dev = body.get("device")
    if not isinstance(dev, str) or not _DEVICE_RE.match(dev):
        raise PortalError("device must be a device id like ph-1a2b3c")
    lang = body.get("lang") or "en"
    if lang not in LANGS:
        raise PortalError(f"lang must be one of {', '.join(LANGS)}")
    by = _clean(body.get("confirmed_by"), 40) if body.get("confirmed_by") is not None else ""
    tpl = body.get("template")
    if tpl:
        if tpl not in REPLY_TEMPLATES:
            raise PortalError("template must be one of " + ", ".join(REPLY_TEMPLATES))
        return {"device": dev, "template": tpl, "lang": lang, "text": reply_text(tpl, lang),
                "confirmed_by": by or "operator", "source": "template"}
    text = _clean(body.get("text"), MAX_MSG)
    if not text:
        raise PortalError("give a template or text")
    if not by:
        raise PortalError("free text is sent only after an operator confirms it (confirmed_by)")
    if BANNED.search(text):
        raise PortalError("no 'found / confirmed' claims in replies")
    return {"device": dev, "template": None, "lang": lang, "text": text, "confirmed_by": by, "source": "operator"}


def _read_json(path, empty):
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
        if isinstance(doc, dict) and all(isinstance(doc.get(k), list) for k in empty if isinstance(empty[k], list)):
            return doc
    except (OSError, ValueError):
        pass
    return json.loads(json.dumps(empty))


def _save_json(path, doc):
    doc["revision"] = int(doc.get("revision") or 0) + 1
    doc["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = f"{path}.{os.getpid()}.{threading.get_ident()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def sim_time(state_path):
    """Mission time t from state.json without parsing the whole file (t is near the start)."""
    try:
        with open(state_path, "rb") as f:
            m = _T_RE.search(f.read(512))
        return round(float(m.group(1)), 1) if m else None
    except (OSError, ValueError):
        return None


class PortalStore:
    """The real captive-portal API behind scripts/serve.py.

    dashboard/data/portal.json        submissions (POST /api/portal from phones; GET /api/portal for the dashboard)
    dashboard/data/comms_replies.json operator replies queue (POST /api/comms/reply; phones read their own via
                                      GET /api/portal/inbox?device=<token>; the mission folds them into state)
    Rate limit per remote address, kept in memory as salted SHA-256 hashes only (random salt per process, never
    written to disk): no raw IP is stored anywhere. Writes are atomic (tmp + os.replace) under a lock."""

    def __init__(self, path, replies_path=None, state_path=None, per_minute=6, per_hour=40, clock=time.time,
                 salt=None, use_llm=False, llm_call=None):
        self.path = path
        self.replies_path = replies_path or os.path.join(os.path.dirname(path) or ".", "comms_replies.json")
        self.state_path = state_path or os.path.join(os.path.dirname(path) or ".", "state.json")
        self.per_minute, self.per_hour, self.clock = per_minute, per_hour, clock
        self.use_llm, self.llm_call = use_llm, llm_call
        self._salt = salt if salt is not None else os.urandom(16)
        self._hits = {}
        self._lock = threading.Lock()

    EMPTY = {"schema": PORTAL_SCHEMA, "revision": 0, "submissions": []}
    EMPTY_REPLIES = {"schema": REPLIES_SCHEMA, "revision": 0, "items": []}

    @staticmethod
    def read(path):
        return _read_json(path, PortalStore.EMPTY)

    def addr_hash(self, addr):
        return hashlib.sha256(self._salt + str(addr).encode()).hexdigest()[:16]

    def allow(self, addr):
        """Sliding-window rate limit: per_minute and per_hour per (hashed) address."""
        now, h = self.clock(), self.addr_hash(addr)
        with self._lock:
            hits = [x for x in self._hits.get(h, []) if now - x < 3600]
            if sum(1 for x in hits if now - x < 60) >= self.per_minute or len(hits) >= self.per_hour:
                self._hits[h] = hits
                return False
            self._hits[h] = hits + [now]
            return True

    def submit(self, body, remote_addr):
        """POST /api/portal -> (http_status, json). 400 invalid, 429 rate-limited. The message is translated at
        once (offline fallback; Gemini only if use_llm and a key/quota is available)."""
        try:
            rec = validate_portal(body)
        except PortalError as e:
            return 400, {"error": str(e)}
        if not self.allow(remote_addr):
            return 429, {"error": "too many messages; wait a minute"}
        tr, by = (translate_message(rec["text"], rec["lang"], self.use_llm, self.llm_call)
                  if rec["text"] else (None, "none"))
        with self._lock:
            doc = self.read(self.path)
            rec = {"id": f"s{int(self.clock() * 1000) % 10**9}-{len(doc['submissions']) + 1}",
                   "t": sim_time(self.state_path), "received_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                   "device": device_id(rec["token"]), **rec, "translation": tr, "by": by}
            doc["submissions"] = (doc["submissions"] + [rec])[-MAX_ITEMS:]
            _save_json(self.path, doc)
        return 200, {"ok": True, "id": rec["id"], "family_queued": rec["family"] is not None}

    DASH_KEYS = ("id", "t", "device", "status", "text", "lang", "translation", "by", "battery_pct")

    def dashboard(self):
        """GET /api/portal (operator, localhost only): {"submissions": [{id, t, device, status, text, lang,
        translation, by, battery_pct}]} - exactly what dashboard/comms.js polls (token, location, family stay out)."""
        return 200, {"submissions": [{k: s.get(k) for k in self.DASH_KEYS} for s in self.read(self.path)["submissions"]]}

    def reply(self, body):
        """POST /api/comms/reply -> queued in comms_replies.json (same pattern as /api/broadcast)."""
        try:
            item = validate_reply(body)
        except PortalError as e:
            return 400, {"error": str(e)}
        with self._lock:
            doc = _read_json(self.replies_path, self.EMPTY_REPLIES)
            item = {"id": f"r{len(doc['items']) + 1}-{int(self.clock()) % 100000}", "t": sim_time(self.state_path),
                    "requested_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "status": "queued", **item}
            doc["items"] = (doc["items"] + [item])[-MAX_ITEMS:]
            _save_json(self.replies_path, doc)
        return 200, item

    def inbox(self, token):
        """GET /api/portal/inbox?device=<full token>: the replies queued for this phone only."""
        if not isinstance(token, str) or not _TOKEN_RE.match(token):
            return 400, {"error": "bad device token"}
        dev = device_id(token)
        doc = _read_json(self.replies_path, self.EMPTY_REPLIES)
        return 200, {"replies": [{k: r.get(k) for k in ("id", "template", "lang", "text")}
                                 for r in doc["items"] if r.get("device") == dev][-10:]}

    def status(self):
        """GET /api/portal/status for the phone: is internet up at the GCS (family messages)? From state.json."""
        internet = False
        try:
            with open(self.state_path, encoding="utf-8") as f:
                st = json.load(f)
            internet = bool(((st.get("comms") or {}).get("backhaul") or {}).get("internet"))
        except (OSError, ValueError, AttributeError):
            pass
        return 200, {"internet": internet}

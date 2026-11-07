"""Loudspeaker messages for the heavy-lift drones: vetted templates, safe-route guidance, broadcast log.

Every spoken string comes from the fixed templates below (``MESSAGES``, ``GUIDANCE``), filled only with
numbers and words from fixed tables (compass words, distances rounded to 5/10 m). Nothing an LLM writes is
ever played automatically: an operator's new message may be machine-translated (Gemini, optional), but
the translations stay ``pending_approval`` until the operator approves them (``BroadcastLog.approve``);
``BroadcastLog.add`` refuses LLM text without an approval.

    from sim.guidance import MESSAGES, LANGS, guidance_route, guidance_texts, BroadcastLog
    route = guidance_route(grid, start_xy, goal_xy=BASE_XY, zones=[poly, ...])     # A* around rubble and zones
    texts = guidance_texts(route["legs"])                                            # {lang: text}

Bearings follow contracts/INTERFACES.md: degrees counter-clockwise from +x (east), so 0 = east, 90 = north.
Compass words are only for speech.

Language review status (``REVIEW``): English is the source. The other strings were written for this demo
and still need a native speaker's check before any field use (Taiwanese Hokkien especially: written in Han
characters with a Tai-lo romanisation for the recording; no open TTS voice speaks it).
"""
from __future__ import annotations

import hashlib
import heapq
import json
import math
import re
import time

try:                    # numpy is only needed for routing; templates, the log and the operator queue are stdlib-only
    import numpy as np  # so scripts/serve.py (system python) can import this module
except ImportError:     # pragma: no cover
    np = None

LANGS = ("zh-TW", "nan-TW", "en", "id", "vi")
LANG_NAMES = {"zh-TW": "Mandarin (Taiwan)", "nan-TW": "Taiwanese Hokkien", "en": "English",
              "id": "Indonesian", "vi": "Vietnamese"}
REVIEW = {"en": "source text",
          "zh-TW": "pending native-speaker review",
          "nan-TW": "pending native-speaker review (Han characters + Tai-lo; record a native speaker)",
          "id": "pending native-speaker review",
          "vi": "pending native-speaker review"}
KINDS = ("callout", "reassure", "guidance", "operator")

# ----------------------------------------------------------------------------- fixed messages
MESSAGES = {
    "callout": {
        "en": "This is a rescue team. If you can hear me, knock three times or shout.",
        "zh-TW": "這裡是救援隊。如果你聽得到，請敲三下或大聲喊。",
        "nan-TW": "阮是救難隊。你若聽會著，請敲三下抑是大聲喝。",
        "id": "Ini tim penyelamat. Jika Anda mendengar saya, ketuk tiga kali atau berteriak.",
        "vi": "Đây là đội cứu hộ. Nếu bạn nghe thấy tôi, hãy gõ ba lần hoặc hét lên.",
    },
    "reassure": {
        "en": "Help is coming. Stay still, save your energy, tap if you can hear me.",
        "zh-TW": "救援人員正在趕來。請保持不動，保存體力，聽得到請輕敲。",
        "nan-TW": "救援的人當咧來矣。請莫振動，保留氣力，聽會著就敲一下。",
        "id": "Bantuan sedang datang. Tetap diam, hemat tenaga Anda, ketuk jika Anda mendengar saya.",
        "vi": "Cứu hộ đang đến. Hãy nằm yên, giữ sức, gõ nhẹ nếu bạn nghe thấy tôi.",
    },
}
# Tai-lo romanisation of the Hokkien lines (for the native speaker who records them; not played).
NAN_TAILO = {
    "callout": "Guán sī kiù-lān-tuī. Lí nā thiann ē tio̍h, tshiánn khà sann-ē ia̍h-sī tuā-siann huah.",
    "reassure": "Kiù-uān ê lâng tng-teh lâi--ah. Tshiánn mài tín-tāng, pó-liû khuì-la̍t, thiann ē tio̍h tō khà--tsi̍t-ē.",
}

# Guidance: intro + leg (first) + leg (next, repeated) + destination + closing line. Slots: {dir} {dist} {dest}.
GUIDANCE = {
    "en": {"intro": "Keep clear of the rubble. ", "first": "Walk {dir} about {dist} metres",
           "next": ", then {dir} about {dist} metres", "dest": ", to the {dest}. ",
           "close": "If you are hurt, stay where you are and wave.",
           "dests": {"tent": "rescue tent"},
           "dirs": ["east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east"]},
    "zh-TW": {"intro": "請遠離瓦礫堆。", "first": "往{dir}走大約{dist}公尺",
              "next": "，再往{dir}走大約{dist}公尺", "dest": "，到{dest}。",
              "close": "如果受傷，請留在原地揮手。",
              "dests": {"tent": "救援帳篷"},
              "dirs": ["東", "東北", "北", "西北", "西", "西南", "南", "東南"]},
    "nan-TW": {"intro": "請離瓦礫堆較遠咧。", "first": "向{dir}行差不多{dist}公尺",
               "next": "，閣向{dir}行差不多{dist}公尺", "dest": "，到{dest}。",
               "close": "若有著傷，請留佇原所在擛手。",
               "dests": {"tent": "救援的帳篷"},
               "dirs": ["東爿", "東北爿", "北爿", "西北爿", "西爿", "西南爿", "南爿", "東南爿"]},
    "id": {"intro": "Menjauhlah dari reruntuhan. ", "first": "Berjalanlah ke arah {dir} sekitar {dist} meter",
           "next": ", lalu ke arah {dir} sekitar {dist} meter", "dest": ", menuju {dest}. ",
           "close": "Jika Anda terluka, tetap di tempat dan lambaikan tangan.",
           "dests": {"tent": "tenda penyelamat"},
           "dirs": ["timur", "timur laut", "utara", "barat laut", "barat", "barat daya", "selatan", "tenggara"]},
    "vi": {"intro": "Hãy tránh xa đống đổ nát. ", "first": "Đi về hướng {dir} khoảng {dist} mét",
           "next": ", rồi về hướng {dir} khoảng {dist} mét", "dest": ", đến {dest}. ",
           "close": "Nếu bị thương, hãy ở yên tại chỗ và vẫy tay.",
           "dests": {"tent": "lều cứu hộ"},
           "dirs": ["đông", "đông bắc", "bắc", "tây bắc", "tây", "tây nam", "nam", "đông nam"]},
}

# Rough speaking rate per language (s per character) when no rendered audio exists (acoustic/speaker_assets.py
# writes real durations to its manifest; callers pass those in ``durations``).
SEC_PER_CHAR = {"en": 0.072, "zh-TW": 0.24, "nan-TW": 0.26, "id": 0.068, "vi": 0.075}


def est_duration(text, lang):
    return round(0.4 + SEC_PER_CHAR.get(lang, 0.08) * len(text), 2)


# ----------------------------------------------------------------------------- compass + distances
def compass_index(bearing_deg):
    """0..7 for east, north-east, north, ... (bearing CCW from east, contract convention)."""
    return int(round((float(bearing_deg) % 360.0) / 45.0)) % 8


def round_distance(d):
    d = float(d)
    if d < 20:
        return max(5, int(5 * round(d / 5)))
    return int(10 * round(d / 10))


def guidance_texts(legs, dest="tent", langs=LANGS):
    """{lang: spoken text} for 1-3 legs [{bearing_deg, dist_m}] from the fixed templates only."""
    if not legs:
        raise ValueError("no legs")
    out = {}
    for lang in langs:
        g = GUIDANCE[lang]
        parts = [g["intro"]]
        for k, leg in enumerate(legs[:3]):
            tpl = g["first"] if k == 0 else g["next"]
            parts.append(tpl.format(dir=g["dirs"][compass_index(leg["bearing_deg"])], dist=round_distance(leg["dist_m"])))
        parts.append(g["dest"].format(dest=g["dests"][dest]))
        parts.append(g["close"])
        out[lang] = "".join(parts).strip()
    return out


# ----------------------------------------------------------------------------- polygons
def points_in_polygon(x, y, poly):
    """Even-odd rule, vectorised over points."""
    p = np.asarray(poly, float).reshape(-1, 2)
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    inside = np.zeros(np.broadcast(x, y).shape, bool)
    xj, yj = p[-1]
    for xi, yi in p:
        cond = ((yi > y) != (yj > y)) & (x < (xj - xi) * (y - yi) / ((yj - yi) if yj != yi else 1e-12) + xi)
        inside ^= cond
        xj, yj = xi, yi
    return inside


# ----------------------------------------------------------------------------- A* route
RUBBLE_RISE_M = 0.4         # cell this far above local ground = rubble (LiDAR heightmap, public)
RUBBLE_MARGIN_M = 2.0       # keep this clear of rubble edges
GROUND_WIN_M = 15.0         # local-ground window (minimum filter)
BLOCK_COST = 60.0           # per-metre cost inside rubble / tasked zones (finite: someone inside can still get out)
ROUTE_RES = 1.0             # A* cell (m)


def cost_map(grid, zones=(), res=ROUTE_RES):
    """(cost[h, w], origin, res, rubble_mask, zone_mask) on a ``res`` grid from the site heightmap.

    Rubble = cells more than RUBBLE_RISE_M above the local ground (minimum filter over GROUND_WIN_M),
    dilated by RUBBLE_MARGIN_M; tasked search zones (polygons) are avoided the same way. Both cost
    BLOCK_COST per metre instead of being forbidden, so a start inside them gets the shortest way out."""
    from scipy.ndimage import binary_dilation, minimum_filter, uniform_filter
    step = max(1, int(round(res / grid.res)))
    z = np.asarray(grid.z, float)
    h, w = z.shape[0] // step, z.shape[1] // step
    zc = z[:h * step, :w * step].reshape(h, step, w, step).max(axis=(1, 3))
    r = res if step > 1 else grid.res
    ground = uniform_filter(minimum_filter(zc, size=max(3, int(GROUND_WIN_M / r))), size=5)
    rubble = zc - ground > RUBBLE_RISE_M
    k = max(1, int(round(RUBBLE_MARGIN_M / r)))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    rubble = binary_dilation(rubble, structure=(xx * xx + yy * yy) <= k * k)
    X = grid.origin[0] + (np.arange(w) + 0.5) * r
    Y = grid.origin[1] + (np.arange(h) + 0.5) * r
    XX, YY = np.meshgrid(X, Y)
    zmask = np.zeros_like(rubble)
    for poly in zones or ():
        zmask |= points_in_polygon(XX, YY, poly)
    cost = np.where(rubble | zmask, BLOCK_COST, 1.0)
    return cost, (float(grid.origin[0]), float(grid.origin[1])), r, rubble, zmask


def astar(cost, start, goal):
    """8-connected A* on a cost grid (per-cell traversal cost x step length). start/goal = (i, j) = (col, row)."""
    h, w = cost.shape
    INF = float("inf")
    g = np.full((h, w), INF)
    came = -np.ones((h, w), np.int64)
    si, sj = start
    gi, gj = goal
    g[sj, si] = 0.0
    heap = [(math.hypot(gi - si, gj - sj), 0.0, si, sj)]
    nbrs = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    while heap:
        _, gc, i, j = heapq.heappop(heap)
        if (i, j) == (gi, gj):
            break
        if gc > g[j, i]:
            continue
        for di, dj, L in nbrs:
            ni, nj = i + di, j + dj
            if not (0 <= ni < w and 0 <= nj < h):
                continue
            c = gc + L * 0.5 * (cost[j, i] + cost[nj, ni])
            if c < g[nj, ni]:
                g[nj, ni] = c
                came[nj, ni] = j * w + i
                heapq.heappush(heap, (c + math.hypot(gi - ni, gj - nj), c, ni, nj))
    if not np.isfinite(g[gj, gi]):
        return None, INF
    path = [(gi, gj)]
    while path[-1] != (si, sj):
        p = int(came[path[-1][1], path[-1][0]])
        path.append((p % w, p // w))
    return path[::-1], float(g[gj, gi])


def _rdp(pts, eps):
    if len(pts) < 3:
        return pts
    a, b = np.asarray(pts[0]), np.asarray(pts[-1])
    ab = b - a
    n = np.hypot(*ab) or 1e-9
    d = [abs(ab[0] * (p[1] - a[1]) - ab[1] * (p[0] - a[0])) / n for p in pts[1:-1]]
    k = int(np.argmax(d)) + 1
    if d[k - 1] > eps:
        return _rdp(pts[:k + 1], eps)[:-1] + _rdp(pts[k:], eps)
    return [pts[0], pts[-1]]


def path_legs(pts, max_legs=3, min_leg_m=4.0):
    """Polyline -> spoken legs [{bearing_deg, dist_m, compass}]: simplify (RDP), merge legs with the same
    8-point compass word, drop short legs; coarsen until at most ``max_legs`` remain."""
    legs = []
    for eps in (1.5, 3.0, 5.0, 8.0, 12.0, 20.0, 40.0):
        poly = _rdp([tuple(p) for p in pts], eps)
        legs = []
        for a, b in zip(poly[:-1], poly[1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            d = math.hypot(dx, dy)
            if d < 1e-6:
                continue
            brg = math.degrees(math.atan2(dy, dx)) % 360.0
            if legs and compass_index(legs[-1]["bearing_deg"]) == compass_index(brg):
                L = legs[-1]
                vx = L["dist_m"] * math.cos(math.radians(L["bearing_deg"])) + dx
                vy = L["dist_m"] * math.sin(math.radians(L["bearing_deg"])) + dy
                L.update(bearing_deg=math.degrees(math.atan2(vy, vx)) % 360.0, dist_m=math.hypot(vx, vy))
            else:
                legs.append({"bearing_deg": brg, "dist_m": d})
        if len(legs) > 1:
            legs = [L for L in legs if L["dist_m"] >= min_leg_m] or legs[:1]
        if len(legs) <= max_legs:
            break
    for L in legs:
        L["bearing_deg"] = round(L["bearing_deg"], 1)
        L["dist_m"] = round(L["dist_m"], 1)
        L["compass"] = GUIDANCE["en"]["dirs"][compass_index(L["bearing_deg"])]
    return legs


def guidance_route(grid, start_xy, goal_xy, zones=(), res=ROUTE_RES, cmap=None):
    """Safe walking route from ``start_xy`` to ``goal_xy`` avoiding rubble mounds and tasked zones.

    Returns {path: [[x, y], ...], legs, heading_deg, length_m, crosses_blocked_m, start_in_rubble,
    ok}. ``cmap``: a precomputed ``cost_map`` (reuse it for several tracks). Public map data only."""
    cost, origin, r, rubble, zmask = cmap if cmap is not None else cost_map(grid, zones, res)
    h, w = cost.shape

    def cell(xy):
        return (int(np.clip((xy[0] - origin[0]) / r, 0, w - 1)), int(np.clip((xy[1] - origin[1]) / r, 0, h - 1)))

    s, gl = cell(start_xy), cell(goal_xy)
    path, _ = astar(cost, s, gl)
    if path is None:
        return {"ok": False, "path": [], "legs": [], "heading_deg": None, "length_m": None}
    pts = [[origin[0] + (i + 0.5) * r, origin[1] + (j + 0.5) * r] for i, j in path]
    pts[0], pts[-1] = [float(start_xy[0]), float(start_xy[1])], [float(goal_xy[0]), float(goal_xy[1])]
    blocked = sum(r for i, j in path if cost[j, i] > 1.0)
    legs = path_legs(pts)
    length = float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:])))
    return {"ok": True, "path": [[round(x, 1), round(y, 1)] for x, y in pts[::2] + [pts[-1]]], "legs": legs,
            "heading_deg": legs[0]["bearing_deg"] if legs else None, "length_m": round(length, 1),
            "crosses_blocked_m": round(blocked, 1), "start_in_rubble": bool(cost[s[1], s[0]] > 1.0)}


# ----------------------------------------------------------------------------- broadcast log
SOURCES = ("template", "operator", "llm_translation")
BANNED = re.compile(r"\b(confirmed|survivors? found|person found|we found you)\b", re.I)


class BroadcastError(ValueError):
    pass


def audio_key(lang, text):
    return hashlib.sha1(f"{lang}\0{text}".encode()).hexdigest()[:12]


class BroadcastLog:
    """Every broadcast, played or pending. Rules enforced here (tests check them):
    * kind in KINDS, lang in LANGS, text non-empty;
    * source ``template`` texts must equal a fixed message or a ``guidance_texts`` output (checked by
      the caller passing ``template=``); ``operator`` texts were typed by the operator;
    * ``llm_translation`` (and anything else not template/operator-typed) is never played without
      ``approved_by``: ``add`` stores it as ``pending_approval`` and only ``approve`` releases it."""

    def __init__(self, audio_lookup=None):
        self.items = []
        self.audio_lookup = audio_lookup or (lambda lang, text: None)
        self._n = 0

    def _id(self):
        self._n += 1
        return f"b{self._n}"

    def add(self, *, t, drone_id, kind, lang, text, text_en, target=None, heading_deg=None, source="template",
            template=None, approved_by=None, duration_s=None, status=None, group=None):
        if kind not in KINDS:
            raise BroadcastError(f"unknown kind {kind!r}")
        if lang not in LANGS:
            raise BroadcastError(f"unsupported language {lang!r}")
        if not text or not str(text).strip():
            raise BroadcastError("empty text")
        if source not in SOURCES:
            raise BroadcastError(f"unknown source {source!r}")
        if source == "template":
            if template in MESSAGES:
                if MESSAGES[template].get(lang) != text:
                    raise BroadcastError("template text does not match the vetted string")
            elif template != "guidance":
                raise BroadcastError(f"unknown template {template!r}")
        needs_ok = source == "llm_translation"
        st = status or ("pending_approval" if needs_ok and not approved_by else "scheduled")
        if st in ("scheduled", "played") and needs_ok and not approved_by:
            raise BroadcastError("LLM text is never broadcast without operator approval")
        au = self.audio_lookup(lang, text)
        url, au_status = au if isinstance(au, tuple) else (au, "rendered" if au else "no audio (text only)")
        b = {"id": self._id(), "t": round(float(t), 1), "drone_id": drone_id, "kind": kind, "lang": lang,
             "text": str(text), "text_en": str(text_en), "audio_url": url, "audio_status": au_status,
             "target": target, "source": source, "template": template, "status": st,
             "approved_by": approved_by, "duration_s": duration_s if duration_s is not None else est_duration(text, lang),
             "review": REVIEW.get(lang) if source == "template" else None, "group": group}
        if heading_deg is not None:
            b["heading_deg"] = round(float(heading_deg), 1)
        self.items.append(b)
        return b

    def approve(self, bid, operator, t=None):
        """Operator approval of a pending (machine-translated) broadcast; returns the item."""
        b = next((x for x in self.items if x["id"] == bid), None)
        if b is None:
            raise BroadcastError(f"unknown broadcast {bid}")
        if b["status"] != "pending_approval":
            raise BroadcastError(f"{bid} is {b['status']}, not pending approval")
        if not operator:
            raise BroadcastError("approval needs an operator name")
        b.update(status="scheduled", approved_by=str(operator)[:40])
        if t is not None:
            b["t"] = round(float(t), 1)
        return b

    def reject(self, bid, operator):
        b = next(x for x in self.items if x["id"] == bid)
        b.update(status="rejected", approved_by=None, rejected_by=str(operator)[:40])
        return b

    def playable(self, t=None):
        return [b for b in self.items if b["status"] in ("scheduled", "played")
                and (b["source"] != "llm_translation" or b["approved_by"]) and (t is None or b["t"] <= t)]

    def state(self, t):
        """Broadcast list as of time t: scheduled items with t <= now become 'played'; future ones hidden;
        pending / rejected operator drafts are listed so the dashboard can show them for approval."""
        out = []
        for b in self.items:
            if b["status"] in ("scheduled", "played"):
                if b["t"] > t:
                    continue
                out.append(dict(b, status="played"))
            else:
                out.append(dict(b))
        return out


# ----------------------------------------------------------------------------- operator messages
TRANSLATE_SCHEMA = {
    "type": "object",
    "properties": {"translations": {"type": "array", "items": {
        "type": "object", "properties": {"lang": {"type": "string"}, "text": {"type": "string"}},
        "required": ["lang", "text"]}}},
    "required": ["translations"],
}
TRANSLATE_SYSTEM = ("You translate short public-address messages for a disaster rescue team's loudspeaker. "
                    "Translate faithfully and plainly; keep every number, direction and place name; do not add "
                    "anything, do not soften or strengthen the message, never claim anyone is found or confirmed. "
                    "Taiwanese Hokkien (nan-TW) is written in Han characters. Return JSON only.")


def validate_translation(src, lang, text):
    """Cheap checks on a machine translation; returns a list of problems (empty = passes)."""
    probs = []
    if not text or not text.strip():
        return ["empty"]
    n_src, n = len(src), len(text)
    lo, hi = (0.15, 1.6) if lang in ("zh-TW", "nan-TW") else (0.5, 2.5)
    if not lo * n_src <= n <= hi * n_src + 10:
        probs.append(f"length {n} vs source {n_src}")
    if sorted(re.findall(r"\d+", src)) != sorted(re.findall(r"\d+", text)):
        probs.append("numbers differ from the source")
    if BANNED.search(text):
        probs.append("banned claim")
    return probs


def translate_operator_message(text, langs=LANGS, src_lang="en", llm_call=None):
    """Machine translation of an operator-typed message (optional Gemini via agents.llm). Returns
    {lang: {"text", "problems", "by"}}; the source language keeps the operator's text. Never broadcasts:
    the caller stores these as pending drafts for the operator to approve. ``llm_call`` (tests) replaces
    agents.llm.call_json."""
    out = {src_lang: {"text": text, "problems": [], "by": "operator"}}
    want = [lng for lng in langs if lng != src_lang]
    if not want:
        return out
    if llm_call is None:
        from agents import llm
        llm_call = llm.call_json
    prompt = (f"Source language: {src_lang}. Message: {json.dumps(text, ensure_ascii=False)}\n"
              f"Translate into: {', '.join(f'{lng} ({LANG_NAMES[lng]})' for lng in want)}.")
    res = llm_call(prompt, TRANSLATE_SCHEMA, system=TRANSLATE_SYSTEM) or {}
    got = {}
    for it in res.get("translations") or []:
        if isinstance(it, dict) and it.get("lang") in want and isinstance(it.get("text"), str):
            got[it["lang"]] = it["text"].strip()
    for lng in want:
        if lng in got:
            out[lng] = {"text": got[lng], "problems": validate_translation(text, lng, got[lng]), "by": "llm"}
        else:
            out[lng] = {"text": None, "problems": ["no translation (LLM unavailable or failed)"], "by": "none"}
    return out


def operator_drafts(log: BroadcastLog, *, t, drone_id, text, src_lang="en", langs=LANGS, target=None,
                    operator=None, translate=True, llm_call=None):
    """Create the broadcasts for an operator message. The operator's own text (``src_lang``) is scheduled
    at once when ``operator`` is given (they typed it and pressed send); every machine translation is added
    as ``pending_approval`` (or skipped if it failed validation) and needs ``log.approve``."""
    if not text or len(text) > 300:
        raise BroadcastError("operator text must be 1-300 characters")
    if src_lang not in LANGS:
        raise BroadcastError(f"unsupported source language {src_lang!r}")
    group = f"op-{int(time.time() * 1000) % 10**8}"
    tr = translate_operator_message(text, langs, src_lang, llm_call) if translate else {
        src_lang: {"text": text, "problems": [], "by": "operator"}}
    made = []
    for lng in [src_lang] + [x for x in langs if x != src_lang]:
        r = tr.get(lng)
        if not r or not r["text"]:
            continue
        if lng == src_lang:
            made.append(log.add(t=t, drone_id=drone_id, kind="operator", lang=lng, text=r["text"], text_en=text,
                                target=target, source="operator", approved_by=operator,
                                status=None if operator else "pending_approval", group=group))
        elif not r["problems"]:
            made.append(log.add(t=t, drone_id=drone_id, kind="operator", lang=lng, text=r["text"], text_en=text,
                                target=target, source="llm_translation", group=group))
    return made, tr


# ----------------------------------------------------------------------------- operator queue (ops server <-> mission)
QUEUE_SCHEMA = "larpverse.broadcast_queue/v1"


def queue_load(path):
    try:
        with open(path) as f:
            doc = json.load(f)
        if isinstance(doc, dict) and isinstance(doc.get("items"), list):
            return doc
    except (OSError, ValueError):
        pass
    return {"schema": QUEUE_SCHEMA, "revision": 0, "items": []}


def _queue_save(path, doc):
    import os
    doc["revision"] = int(doc.get("revision") or 0) + 1
    doc["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False, indent=1)
    os.replace(path + ".tmp", path)
    return doc


def queue_post(path, body, llm_call=None):
    """POST /api/broadcast. body: {drone_id | zone, kind: operator|callout|reassure, text?, src_lang?, langs?,
    translate?, operator}. callout / reassure play the vetted templates (text ignored, no approval needed);
    operator: the typed text is queued at once, machine translations as pending_approval. Returns the new items."""
    if not isinstance(body, dict):
        raise BroadcastError("body must be an object")
    kind = body.get("kind", "operator")
    if kind not in ("operator", "callout", "reassure"):
        raise BroadcastError("kind must be operator, callout or reassure (guidance is computed by the mission)")
    drone, zone = body.get("drone_id"), body.get("zone")
    if not drone and not zone:
        raise BroadcastError("give drone_id or zone")
    operator = str(body.get("operator") or "").strip()[:40]
    if not operator:
        raise BroadcastError("operator name required (every broadcast is attributed)")
    langs = tuple(x for x in (body.get("langs") or LANGS) if x in LANGS) or LANGS
    target = {"type": "zone", "id": str(zone)[:24]} if zone else {"type": "drone", "id": str(drone)[:8]}
    log = BroadcastLog()
    if kind in MESSAGES:
        made = [log.add(t=0, drone_id=drone, kind=kind, lang=lng, text=MESSAGES[kind][lng],
                        text_en=MESSAGES[kind]["en"], target=target, template=kind, approved_by=operator)
                for lng in langs]
        tr = None
    else:
        made, tr = operator_drafts(log, t=0, drone_id=drone, text=str(body.get("text") or "").strip(),
                                   src_lang=body.get("src_lang", "en"), langs=langs, target=target,
                                   operator=operator, translate=bool(body.get("translate", True)), llm_call=llm_call)
    doc = queue_load(path)
    stamp = time.strftime("%H%M%S")
    for b in made:
        b["id"] = f"op{stamp}-{len(doc['items']) + 1}"
        b["requested_by"] = operator
        b["t"] = None                                  # the mission stamps sim time when it plays the item
        doc["items"].append(b)
    _queue_save(path, doc)
    return {"items": made, "translation_problems": {k: v["problems"] for k, v in (tr or {}).items() if v["problems"]}}


def queue_decide(path, bid, operator, approve=True):
    """POST /api/broadcast/approve {id, operator, approve}: release or reject a pending item."""
    if not operator:
        raise BroadcastError("operator name required")
    doc = queue_load(path)
    b = next((x for x in doc["items"] if x["id"] == bid), None)
    if b is None:
        raise BroadcastError(f"unknown id {bid}")
    if b["status"] != "pending_approval":
        raise BroadcastError(f"{bid} is {b['status']}")
    if approve:
        b.update(status="scheduled", approved_by=str(operator)[:40])
    else:
        b.update(status="rejected", rejected_by=str(operator)[:40])
    _queue_save(path, doc)
    return b

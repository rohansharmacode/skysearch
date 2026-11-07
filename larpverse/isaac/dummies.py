"""Victim dummies: rescue-training-manikin bodies partly buried in local rubble, one per scenario victim, plus
the manifest's distractor sources (dog, generator, settling rubble) and HIDDEN dummies for the live change.

    from isaac import dummies as DM
    man = DM.load_manifest()                         # assets/audio/dummies/manifest.json (None if missing)
    info = DM.build_dummies(stage, sc, man)          # /World/Dummies/<id>, SpatialAudio + larpverse:* attrs
    rows = DM.read_dummies(stage)                    # live loop, every listen: [{id, xyz, active, clip, physio, ...}]
    DM.set_active(stage, "H1_C3_vi", True)           # the unscripted change (or flip larpverse:active in the GUI)

Isaac is the source of truth for the take: ``read_dummies`` returns each dummy prim's live WORLD position (so
dragging a dummy in the viewport moves its radar / acoustic source), whether it is active, its current clip
and its breathing condition.  Inactive dummies are invisible to every sensor model (the live loop skips them);
their geometry stays (they are buried), the SpatialAudio gain is 0.

Per dummy prim (Xform at the breathing point, absolute z from the scenario: ``isaac.site.victim_z``):
  larpverse:id, larpverse:victim (scenario id or ""), larpverse:pile, larpverse:kind (voice | weak_voice | knock
  | silent | dog | generator | settling), larpverse:human (bool), larpverse:active (bool), larpverse:hidden
  (bool), larpverse:clip (repo-relative wav, "" if silent), larpverse:clips / larpverse:reply_clips (string[]),
  larpverse:schedule (JSON: the manifest's idle / after_callout blocks), larpverse:physio (breathing state for
  the radar generator, "none" for non-humans), larpverse:physio_json, larpverse:depth, larpverse:pose,
  larpverse:spl_db_1m;  children Body/ (manikin meshes), Cover/ (visual-only slabs and chunks: NO colliders,
  so the LiDAR never sees the dummy or its cover), Audio (UsdMedia.SpatialAudio at the head).

Display / sensor-source only: the brain never reads these prims (docs/agents/RULES.md rule 9: only observation
generators and evaluators read the truth).  The manikin look: navy coveralls, smooth tan vinyl head and hands,
black boots, dusty; no injuries, no blood.  Poses follow the breathing state (calm: supine with an arm up by
the head, knocking; distressed: curled on the side; unconscious: prone; irregular: curled).
"""
from __future__ import annotations

import json
import math
import os
import zlib

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MANIFEST = os.path.join(REPO, "assets", "audio", "dummies", "manifest.json")
AUDIO_DIR = "assets/audio/dummies"
NONHUMAN = {"dog", "generator", "settling", "wind", "siren"}

# --------------------------------------------------------------------------------------------- poses
# Joint positions (m). Lying poses: void floor at z = 0, head towards +x. Seated: floor at z = 0, facing +x.
POSES = {
    "seated": dict(pelvis=(0, 0, .50), neck=(-.05, 0, 1.0), head=(-.02, 0, 1.18),
                   sh_l=(-.04, .19, .95), sh_r=(-.04, -.19, .95), el_l=(.15, .24, .72), el_r=(.15, -.24, .72),
                   wr_l=(.40, .18, .80), wr_r=(.40, -.18, .80), hip_l=(0, .10, .50), hip_r=(0, -.10, .50),
                   kn_l=(.45, .12, .52), kn_r=(.45, -.12, .52), an_l=(.48, .13, .08), an_r=(.48, -.13, .08),
                   toe_l=(.64, .13, .04), toe_r=(.64, -.13, .04)),
    "supine": dict(pelvis=(0, 0, .12), neck=(.52, 0, .13), head=(.70, 0, .12),
                   sh_l=(.47, -.19, .12), sh_r=(.47, .19, .12), el_l=(.22, -.28, .08), el_r=(.72, .30, .08),
                   wr_l=(-.02, -.30, .07), wr_r=(.95, .24, .07), hip_l=(0, -.10, .12), hip_r=(0, .10, .12),
                   kn_l=(-.40, -.13, .32), kn_r=(-.45, .12, .11), an_l=(-.78, -.13, .08), an_r=(-.88, .13, .10),
                   toe_l=(-.95, -.13, .05), toe_r=(-.92, .13, .27)),
    "prone": dict(pelvis=(0, 0, .13), neck=(.52, 0, .15), head=(.70, .03, .13),
                  sh_l=(.47, .19, .13), sh_r=(.47, -.19, .13), el_l=(.62, .36, .07), el_r=(.25, -.30, .07),
                  wr_l=(.88, .30, .05), wr_r=(.05, -.33, .05), hip_l=(0, .10, .13), hip_r=(0, -.10, .13),
                  kn_l=(-.38, .28, .08), kn_r=(-.45, -.12, .08), an_l=(-.80, .24, .12), an_r=(-.88, -.13, .11),
                  toe_l=(-.90, .24, .03), toe_r=(-.96, -.13, .03)),
    "curled": dict(pelvis=(0, 0, .20), neck=(.47, -.05, .22), head=(.64, -.12, .20),
                   sh_l=(.42, -.02, .04), sh_r=(.42, -.02, .38), el_l=(.50, -.26, .07), el_r=(.36, -.30, .33),
                   wr_l=(.66, -.30, .10), wr_r=(.56, -.36, .28), hip_l=(0, 0, .10), hip_r=(0, 0, .30),
                   kn_l=(.20, -.42, .10), kn_r=(.26, -.44, .30), an_l=(-.20, -.40, .08), an_r=(-.14, -.44, .26),
                   toe_l=(-.24, -.52, .08), toe_r=(-.18, -.56, .24)),
}
POSE_FOR_STATE = {"calm": "supine", "distressed_tachypnea": "curled", "unconscious_slow": "prone",
                  "irregular_apneic": "curled"}
CHEST_F = 0.55                     # breathing point = pelvis + 0.55 (neck - pelvis): the prim origin

MATS = {
    "vinyl": dict(rgb=(0.66, 0.55, 0.47), rough=0.5),
    "coverall": dict(rgb=(0.15, 0.19, 0.28), rough=0.92),
    "coverall_dusty": dict(rgb=(0.30, 0.31, 0.33), rough=0.95),
    "boots": dict(rgb=(0.04, 0.04, 0.04), rough=0.6),
    "slab": dict(rgb=(0.55, 0.53, 0.50), rough=0.95),
    "slab_dark": dict(rgb=(0.42, 0.40, 0.38), rough=0.95),
    "rebar": dict(rgb=(0.35, 0.20, 0.12), rough=0.7, metal=0.6),
    "dog_fur": dict(rgb=(0.42, 0.30, 0.18), rough=0.95),
    "dog_dark": dict(rgb=(0.10, 0.08, 0.07), rough=0.8),
    "gen_yellow": dict(rgb=(0.85, 0.66, 0.10), rough=0.55),
    "gen_frame": dict(rgb=(0.08, 0.08, 0.09), rough=0.5),
}


def mannequin(A, J, M, group="Body", mats=None, vest=None, headset=None, detail=1):
    """Add a manikin body for joints ``J`` (pose dict) transformed by 4x4 ``M`` to Assembly ``A``."""
    mats = mats or dict(skin="vinyl", cloth="coverall", boots="boots")
    n, m = (12, 8) if detail else (7, 5)
    rings = 3 if detail else 2
    J = {k: (M @ np.r_[np.asarray(v, float), 1.0])[:3] for k, v in J.items()}
    skin, cloth, boots = A.g(mats["skin"], group), A.g(mats["cloth"], group), A.g(mats["boots"], group)
    pelvis, neck, head = J["pelvis"], J["neck"], J["head"]
    spine = neck - pelvis
    up = spine / np.linalg.norm(spine)
    lat = J["sh_l"] - J["sh_r"]
    lat = lat - np.dot(lat, up) * up
    lat /= np.linalg.norm(lat)
    front = np.cross(lat, up)

    def ell(g, c, a, f, u, upv=up, latv=lat):
        frv = np.cross(latv, upv)
        g.ellipsoid(c, (latv * a, frv * f, upv * u), n + 2, m + 1)
    ell(cloth, pelvis + up * 0.02, 0.17, 0.115, 0.13)
    ell(cloth, pelvis + spine * 0.38, 0.155, 0.11, 0.17)
    ell(cloth, pelvis + spine * 0.70, 0.19, 0.12, 0.19)
    if vest:
        ell(A.g(vest, group), pelvis + spine * 0.60, 0.197, 0.127, 0.25)
    hd = head - neck
    hd /= np.linalg.norm(hd)
    lat_h = lat - np.dot(lat, hd) * hd
    lat_h /= np.linalg.norm(lat_h)
    skin.capsule(neck - up * 0.04, neck + (head - neck) * 0.45, 0.05, 0.048, n, rings)
    ell(skin, head, 0.082, 0.098, 0.112, upv=hd, latv=lat_h)
    fr_h = np.cross(lat_h, hd)
    skin.ellipsoid(head + fr_h * 0.095 - hd * 0.005, (lat_h * 0.014, fr_h * 0.02, hd * 0.025), 6, 4)   # nose
    for s in ("l", "r"):
        sh, el, wr = J["sh_" + s], J["el_" + s], J["wr_" + s]
        cloth.sphere(0.058, sh, n, m)
        cloth.capsule(sh, el, 0.052, 0.045, n, rings)
        cloth.capsule(el, wr, 0.043, 0.036, n, rings)
        d = (wr - el) / np.linalg.norm(wr - el)
        skin.capsule(wr, wr + d * 0.12, 0.033, 0.028, n, rings)
        hip, kn, an, toe = J["hip_" + s], J["kn_" + s], J["an_" + s], J["toe_" + s]
        cloth.capsule(hip, kn, 0.085, 0.062, n, rings)
        cloth.capsule(kn, an, 0.058, 0.045, n, rings)
        dl = (an - kn) / np.linalg.norm(an - kn)
        boots.capsule(an - dl * 0.06, an, 0.05, 0.05, n, rings)
        boots.capsule(an, toe, 0.05, 0.042, n, rings)
    if headset:
        g = A.g(headset, group)
        for sg in (-1, 1):
            g.cyl(0.04, 0.03, head + lat_h * sg * 0.093, axis=lat_h, n=10)
        top = head + hd * 0.125
        for sg in (-1, 1):
            mid = head + lat_h * sg * 0.075 + hd * 0.09
            g.capsule(head + lat_h * sg * 0.093 + hd * 0.03, mid, 0.008, 0.008, 6, 1)
            g.capsule(mid, top, 0.008, 0.008, 6, 1)
        g.capsule(head + lat_h * 0.1, head + lat_h * 0.06 + fr_h * 0.09 - hd * 0.06, 0.006, 0.006, 6, 1)
    return J


def _cover(A, J, rng, depth, group="Cover", shift=(0, 0, 0)):
    """Visual-only broken slabs over the legs / one side, chunks around, rebar; kept below the surface.
    ``J`` = pose joints (void floor at z = 0); everything is moved by ``shift`` (pose -> prim frame)."""
    from isaac.van import Geo, T
    geos = {}

    class _G:                                       # collect per material, merge shifted at the end
        def g(self, mat, _group=None):
            return geos.setdefault(mat, Geo())
    real, A = A, _G()
    pel = np.asarray(J["pelvis"], float)
    chest_z = (pel + CHEST_F * (np.asarray(J["neck"]) - pel))[2]
    zmax = chest_z + max(0.2, depth) - 0.08
    from isaac.van import Rx, Ry, Rz
    ax = np.asarray(J["neck"], float) - pel
    yaw = math.degrees(math.atan2(ax[1], ax[0]))
    Rb = Rz(yaw)
    legs = (np.asarray(J["kn_l"]) + np.asarray(J["kn_r"]) + np.asarray(J["an_l"]) + np.asarray(J["an_r"])) / 4
    top = max(J["kn_l"][2], J["kn_r"][2]) + 0.18
    c1 = np.array([legs[0], legs[1], min(top, zmax)])
    R1 = Rb @ Ry(rng.uniform(-9, 9)) @ Rx(rng.uniform(-12, 12))
    A.g("slab", group).box((1.05, 0.85, 0.12), c1, R=R1)
    for k in range(5):
        A.g("rebar", group).cyl_between(c1 + R1 @ (rng.uniform(-0.5, 0.5), -0.43, rng.uniform(-0.03, 0.03)),
                                        c1 + R1 @ (rng.uniform(-0.5, 0.5), -0.6 - rng.uniform(0, 0.2), rng.uniform(-0.1, 0.1)),
                                        0.007, n=5, caps=False)
    for sx, sy in ((-0.45, -0.35), (0.45, 0.35), (0.45, -0.38)):          # supports under the slab corners
        p = c1 + R1 @ (sx, sy, 0)
        h = max(0.06, p[2] - 0.06)
        A.g("slab_dark", group).box((0.22, 0.2, h), (p[0], p[1], h / 2), R=Rz(rng.uniform(0, 90)))
    side = rng.choice([-1, 1])
    lat = Rb @ (0, 1, 0)
    c2 = pel + 0.45 * ax + lat * side * 0.42
    c2[2] = min(0.30, zmax)
    A.g("slab_dark", group).box((0.8, 0.55, 0.10), c2, R=Rb @ Rx(side * -24) @ Ry(rng.uniform(-6, 6)))
    for _ in range(9):                                                      # chunks around the body
        a = rng.uniform(0, 2 * math.pi)
        r = rng.uniform(0.45, 0.85)
        c = pel + 0.2 * ax + np.array([math.cos(a) * r * 1.3, math.sin(a) * r, 0])
        s = rng.uniform(0.08, 0.26, 3)
        c[2] = min(s[2] / 2 * 0.9, zmax)
        A.g("slab" if rng.random() < 0.5 else "slab_dark", group).box(s, c, R=Rz(rng.uniform(0, 180)) @ Rx(rng.uniform(-20, 20)))
    for mat, g in geos.items():
        real.g(mat, group).merge(g, M=T(shift))


def _dog(A, group="Body"):
    """Medium dog standing on the pile surface (distractor); origin on the ground under its chest."""
    f, d = A.g("dog_fur", group), A.g("dog_dark", group)
    f.ellipsoid((0, 0, 0.48), ((0.36, 0, 0), (0, 0.15, 0), (0, 0, 0.16)), 14, 9)
    f.capsule((0.26, 0, 0.55), (0.40, 0, 0.70), 0.08, 0.07, 10, 2)
    f.ellipsoid((0.46, 0, 0.74), ((0.12, 0, 0), (0, 0.09, 0), (0, 0, 0.09)), 12, 8)
    f.capsule((0.52, 0, 0.72), (0.64, 0, 0.69), 0.05, 0.035, 8, 2)
    d.sphere(0.022, (0.665, 0, 0.695), 8, 5)
    for sy in (-1, 1):
        d.ellipsoid((0.42, sy * 0.07, 0.84), ((0.03, 0, 0), (0, 0.035, 0), (0, 0, 0.06)), 8, 5)
        for lx in (0.22, -0.24):
            f.capsule((lx, sy * 0.08, 0.42), (lx + 0.02, sy * 0.085, 0.04), 0.045, 0.032, 8, 2)
            d.sphere(0.035, (lx + 0.04, sy * 0.085, 0.03), 8, 5)
    f.capsule((-0.34, 0, 0.55), (-0.55, 0, 0.66), 0.03, 0.02, 8, 2)


def _small_generator(A, group="Body"):
    y, k = A.g("gen_yellow", group), A.g("gen_frame", group)
    y.box((0.58, 0.42, 0.40), (0, 0, 0.26))
    k.box((0.2, 0.01, 0.12), (0.1, -0.215, 0.3))
    for sx in (-0.31, 0.31):
        k.cyl_between((sx, -0.23, 0.05), (sx, -0.23, 0.5), 0.015, n=6)
        k.cyl_between((sx, 0.23, 0.05), (sx, 0.23, 0.5), 0.015, n=6)
        k.cyl_between((sx, -0.23, 0.5), (sx, 0.23, 0.5), 0.015, n=6)
    k.cyl(0.025, 0.12, (-0.18, 0.1, 0.52), n=8)


# --------------------------------------------------------------------------------------------- manifest
def load_manifest(path=MANIFEST):
    """The dummy-audio manifest (schema larpverse.dummy_audio/v1) or None if it is not there."""
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def clip_index(manifest):
    return {c["id"]: c for c in (manifest or {}).get("clips", []) if isinstance(c, dict) and "id" in c}


def clip_path(manifest, clip_id):
    """Repo-relative wav path of a clip id ('' if unknown)."""
    c = clip_index(manifest).get(clip_id)
    if not c:
        return ""
    return f"{AUDIO_DIR}/{c.get('file', clip_id + '.wav')}"


def _playlist(manifest, block):
    if not block:
        return []
    return [p for p in (clip_path(manifest, c) for c in block.get("playlist", [])) if p]


def _hash01(s):
    return (zlib.crc32(s.encode()) & 0xFFFF) / 65535.0


def _pile_map(sc):
    return {p.name: p for p in sc.piles}


def plan_dummies(sc, manifest=None):
    """Pure python: the dummy list for a scenario (no pxr). Each entry: id, prim, victim, pile, kind, human,
    xyz, yaw, pose, physio, physio_json, depth, active, hidden, clip, clips, reply_clips, schedule, spl."""
    from isaac.site import victim_z
    man = manifest or {}
    profiles = [p for p in man.get("profiles", []) if isinstance(p, dict)]
    piles = _pile_map(sc)
    clips = clip_index(man)
    by_pile = {p.get("pile"): p for p in profiles if p.get("scenario_kind") not in NONHUMAN}
    by_kind = {}
    for p in profiles:
        if p.get("scenario_kind") not in NONHUMAN:
            by_kind.setdefault(p.get("scenario_kind"), []).append(p)
    used_kind = {}
    out = []

    def entry(did, prim_name, victim, pile, kind, xyz, physio, depth, prof, active, hidden, physio_json=None):
        idle = (prof or {}).get("idle")
        reply = (prof or {}).get("after_callout")
        lst = _playlist(man, idle)
        spl = None
        if idle and idle.get("playlist"):
            spl = clips.get(idle["playlist"][0], {}).get("suggested_spl_db_1m")
        human = kind not in NONHUMAN
        pose = "none"
        if human:
            pose = POSE_FOR_STATE.get(physio or "calm", "curled")
        return dict(id=prim_name, prim=f"/World/Dummies/{prim_name}", victim=victim or "", pile=pile or "",
                    kind=kind, human=human, xyz=tuple(float(v) for v in xyz),
                    yaw=float(360.0 * _hash01(prim_name)), pose=pose, physio=physio or "none",
                    physio_json=json.dumps(physio_json) if physio_json else "", depth=float(depth),
                    active=bool(active), hidden=bool(hidden), clip=lst[0] if lst else "", clips=lst,
                    reply_clips=_playlist(man, reply),
                    schedule=json.dumps({"idle": idle, "after_callout": reply, "story": (prof or {}).get("story", "")}),
                    spl=float(spl) if spl else 0.0, profile=(prof or {}).get("dummy_id", ""))

    for v in sc.truth.get("victims", []):
        kind = v.get("sound") or "silent"
        prof = by_pile.get(v.get("pile"))
        if prof is not None and prof.get("scenario_kind") == kind:
            name = (prof.get("prim") or f"/World/Dummies/{prof['dummy_id']}").rsplit("/", 1)[-1]
        else:
            cands = by_kind.get(kind) or []
            k = used_kind.get(kind, 0)
            prof = cands[k % len(cands)] if cands else None
            used_kind[kind] = k + 1
            name = v["id"]
        x, y = v["xy"]
        ph = v.get("physio") or {}
        out.append(entry(v["id"], name, v["id"], v.get("pile"), kind, (x, y, victim_z(sc, v)),
                         ph.get("state") or (prof or {}).get("physio"), v.get("depth", 0.0), prof, True, False, ph))
    # non-human sources of the manifest (dog, generator, settling): only where their pile exists (grid)
    sounds = {(s.get("pile"), s.get("kind")): s for s in sc.truth.get("sounds", []) if isinstance(s, dict)}
    for prof in profiles:
        kind = prof.get("scenario_kind")
        if kind not in NONHUMAN or prof.get("pile") not in piles:
            continue
        s = sounds.get((prof.get("pile"), kind))
        p = piles[prof["pile"]]
        x, y = (s or {}).get("xy") or (p.x, p.y)
        name = (prof.get("prim") or f"/World/Dummies/{prof['dummy_id']}").rsplit("/", 1)[-1]
        out.append(entry(name, name, "", prof["pile"], kind, (x, y, float(sc.ground_height(x, y))), None,
                         0.0, prof, True, False))
    # hidden dummies (inactive at start): the profile's pile when it exists, else an empty sector near the start
    taken = {e["pile"] for e in out}
    spare = _spare_sectors(sc, taken)
    for k, prof in enumerate(h for h in man.get("hidden_dummies", []) if isinstance(h, dict)):
        pile = prof.get("pile") if prof.get("pile") in piles else (spare[k] if k < len(spare) else None)
        if pile is None:
            continue
        name = (prof.get("prim") or f"/World/Dummies/{prof['dummy_id']}").rsplit("/", 1)[-1]
        if pile != prof.get("pile"):
            parts = name.split("_")
            name = "_".join([parts[0], pile] + parts[2:]) if len(parts) >= 2 else f"{name}_{pile}"
        x, y = _hidden_xy(sc, piles[pile])
        depth = float(prof.get("burial_depth_m", 0.5))
        out.append(entry(name, name, "", pile, prof.get("scenario_kind", "voice"),
                         (x, y, float(sc.ground_height(x, y)) - depth), prof.get("physio"), depth, prof,
                         bool(prof.get("active_at_start", False)), True,
                         {"state": prof.get("physio")} if prof.get("physio") else None))
    return out


def _spare_sectors(sc, taken):
    """Sectors with no victim and no source, nearest to the start pile first."""
    piles = _pile_map(sc)
    start = piles.get(sc.truth.get("start_pile")) or next(iter(piles.values()))
    vp = {v.get("pile") for v in sc.truth.get("victims", [])} | set(taken)
    free = [p for p in sc.piles if p.name not in vp and p.h > 1.0]
    free.sort(key=lambda p: math.hypot(p.x - start.x, p.y - start.y))
    return [p.name for p in free]


def _hidden_xy(sc, p):
    """On the pile face looking towards staging: 30 % of the half-size from the centre."""
    from isaac.site import base_xy
    bx, by = base_xy(sc)
    d = np.array([bx - p.x, by - p.y], float)
    d /= np.linalg.norm(d) + 1e-9
    return float(p.x + d[0] * 0.3 * p.sx / 2 * 2), float(p.y + d[1] * 0.3 * p.sy / 2 * 2)


# --------------------------------------------------------------------------------------------- USD
def build_dummies(stage, sc, manifest=None, root="/World/Dummies", cover=True, audio=True):
    """Build every planned dummy under ``root``. ``manifest``: load_manifest() result (None = no clips, no hidden
    dummies, no distractors). Returns {"dummies": plan, "tris": total}."""
    from pxr import Sdf, UsdMedia

    from isaac.van import Assembly, Looks, T, count_tris, xform
    plan = plan_dummies(sc, manifest)
    xform(stage, root)
    looks = Looks(stage, root + "/Looks")
    S = Sdf.ValueTypeNames
    for e in plan:
        path = e["prim"] if e["prim"].startswith(root) else f"{root}/{e['id']}"
        e["prim"] = path
        x = xform(stage, path, e["xyz"], e["yaw"])
        A = Assembly(stage, path, looks, MATS)
        rng = np.random.default_rng(zlib.crc32(e["id"].encode()))
        head_local = (0.0, 0.0, 0.4)
        if e["human"]:
            J = POSES[e["pose"]]
            pel, nk = np.asarray(J["pelvis"], float), np.asarray(J["neck"], float)
            chest = pel + CHEST_F * (nk - pel)
            M = T(-chest)
            low = e["depth"] > 1.4 and not e["hidden"]
            cl = "coverall_dusty" if rng.random() < 0.5 else "coverall"
            Jw = mannequin(A, J, M, mats=dict(skin="vinyl", cloth=cl, boots="boots"), detail=0 if low else 1)
            head_local = tuple(Jw["head"])
            if cover:
                _cover(A, J, rng, e["depth"], shift=-chest)
        elif e["kind"] == "dog":
            _dog(A)
            head_local = (0.5, 0.0, 0.75)
        elif e["kind"] == "generator":
            _small_generator(A)
        A.flush()
        prim = x.GetPrim()
        for name, typ, val in (("id", S.String, e["id"]), ("victim", S.String, e["victim"]),
                               ("pile", S.String, e["pile"]), ("kind", S.String, e["kind"]),
                               ("human", S.Bool, e["human"]), ("active", S.Bool, e["active"]),
                               ("hidden", S.Bool, e["hidden"]), ("clip", S.String, e["clip"]),
                               ("clips", S.StringArray, e["clips"]), ("reply_clips", S.StringArray, e["reply_clips"]),
                               ("schedule", S.String, e["schedule"]), ("physio", S.String, e["physio"]),
                               ("physio_json", S.String, e["physio_json"]), ("depth", S.Float, e["depth"]),
                               ("pose", S.String, e["pose"]), ("spl_db_1m", S.Float, e["spl"]),
                               ("profile", S.String, e["profile"])):
            prim.CreateAttribute(f"larpverse:{name}", typ).Set(val)
        if audio:
            xform(stage, path + "/Audio", head_local)                   # ops first; Define then sets the type
            au = UsdMedia.SpatialAudio.Define(stage, path + "/Audio")
            au.CreateFilePathAttr(Sdf.AssetPath(os.path.join(REPO, e["clip"]) if e["clip"] else ""))
            au.CreateAuralModeAttr(UsdMedia.Tokens.spatial)
            sched = json.loads(e["schedule"]).get("idle") or {}
            au.CreatePlaybackModeAttr(UsdMedia.Tokens.loopFromStart if sched.get("mode") == "loop"
                                      else UsdMedia.Tokens.onceFromStart)
            au.CreateGainAttr(1.0 if (e["active"] and e["clip"]) else 0.0)
    return {"dummies": plan, "tris": count_tris(stage, root)}


def build_ambient(stage, sc, manifest=None, root="/World/Ambient"):
    """Ambient beds from the manifest: non-spatial wind, a distant spatial siren ~150 m beyond the site edge."""
    from pxr import Sdf, UsdMedia

    from isaac.site import bounds
    from isaac.van import xform
    man = manifest or {}
    xform(stage, root)
    x0, y0, x1, y1 = bounds(sc)
    n = 0
    for a in man.get("ambient", []):
        path = clip_path(man, a.get("clip", ""))
        if not path:
            continue
        p = f"{root}/{a.get('id', 'amb%d' % n)}"
        xform(stage, p, (x1 + 150.0, (y0 + y1) / 2, 5.0) if a.get("aural") == "spatial" else (0, 0, 0))
        au = UsdMedia.SpatialAudio.Define(stage, p)
        au.CreateFilePathAttr(Sdf.AssetPath(os.path.join(REPO, path)))
        au.CreateAuralModeAttr(UsdMedia.Tokens.spatial if a.get("aural") == "spatial" else UsdMedia.Tokens.nonSpatial)
        au.CreatePlaybackModeAttr(UsdMedia.Tokens.loopFromStart)
        au.CreateGainAttr(float(a.get("gain", 0.3)))
        n += 1
    return n


def read_dummies(stage, root="/World/Dummies"):
    """Every dummy under ``root``: [{id, xyz (world, live), active, clip, physio, kind, human, hidden, pile,
    victim, clips, reply_clips, schedule, depth, pose, spl_db_1m, prim}] sorted by id. Pure read, no edits."""
    from pxr import Usd, UsdGeom
    top = stage.GetPrimAtPath(root)
    if not top.IsValid():
        return []
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    rows = []
    for p in top.GetChildren():
        a = p.GetAttribute("larpverse:id")
        if not a or not a.IsValid():
            continue

        def g(name, default=None):
            at = p.GetAttribute("larpverse:" + name)
            v = at.Get() if at and at.IsValid() else None
            return default if v is None else v
        t = cache.GetLocalToWorldTransform(p).ExtractTranslation()
        rows.append(dict(id=g("id"), prim=str(p.GetPath()), xyz=(float(t[0]), float(t[1]), float(t[2])),
                         active=bool(g("active", False)), clip=str(g("clip", "")), physio=str(g("physio", "")),
                         kind=str(g("kind", "")), human=bool(g("human", True)), hidden=bool(g("hidden", False)),
                         pile=str(g("pile", "")), victim=str(g("victim", "")), clips=list(g("clips", []) or []),
                         reply_clips=list(g("reply_clips", []) or []), schedule=str(g("schedule", "")),
                         depth=float(g("depth", 0.0)), pose=str(g("pose", "")), spl_db_1m=float(g("spl_db_1m", 0.0))))
    rows.sort(key=lambda r: r["id"])
    return rows


def set_active(stage, dummy_id, on=True, clip=None, root="/World/Dummies"):
    """Switch a dummy on/off for every sensor (and its SpatialAudio gain); optionally change its clip
    (repo-relative wav). Returns False if the dummy does not exist."""
    from pxr import Sdf, UsdMedia
    p = stage.GetPrimAtPath(f"{root}/{dummy_id}")
    if not p.IsValid():
        return False
    p.GetAttribute("larpverse:active").Set(bool(on))
    if clip is not None:
        p.GetAttribute("larpverse:clip").Set(str(clip))
    cur = p.GetAttribute("larpverse:clip").Get() or ""
    au = UsdMedia.SpatialAudio(stage.GetPrimAtPath(f"{root}/{dummy_id}/Audio"))
    if au:
        if clip is not None:
            au.GetFilePathAttr().Set(Sdf.AssetPath(os.path.join(REPO, clip) if clip else ""))
        au.GetGainAttr().Set(1.0 if (on and cur) else 0.0)
    return True

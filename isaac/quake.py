"""A physically simulated earthquake for isaac/earthquake_scene.py (--collapse).

Sequence (seconds of physics before the drones launch):
  0 .. PRE           the building stands; the upper floor over the damaged zone is still up
  PRE .. PRE+SHAKE   strong ground motion. In the building's frame a horizontal ground acceleration a(t) is
                     exactly equivalent to tilting gravity, so gravity = (-a_x, -a_y, -9.81): unanchored
                     furniture slides and topples, loose blocks fall off walls, and the camera shakes.
                     Upper-floor slab panels, wall tops and the concrete mass above the rubble zone are
                     released in stages, nearest to the epicentre of the damage first, and fall under PhysX.
  .. +SETTLE         dust hangs, debris settles; then the mission (scenario.step) starts.

The pieces are irregular broken concrete (isaac/rubble.py): chipped blocks, fractured lumps, broken slab pieces
with bent rebar, ragged wall tops, a rubble mound height field and thousands of gravel stones. Each falling piece
collides as its own convex hull. `Q.bursts` lists (t, x, y, z, size) where debris lets go, for volumetric dust.

Everything that falls is a PhysX rigid body. Where it ends up is tied to the map the swarm searches:
  * slab panels land on crushed contents (an invisible ledge at the void height from the map, with visible
    debris posts), so a void that is 0.5 m high in the map is 0.5 m high in Isaac. While they fall, panels
    are guided horizontally (the surrounding structure keeps a pancaking slab roughly in place);
    the vertical fall is free.
  * rubble cells get a static heap base; the concrete blocks that rain onto it make the top of the pile.
  * desks stay where the map has them (people shelter under them).
All static building parts get colliders so falling pieces hit walls, heaps and desks.

The pieces only need Isaac Lab's spawners + RigidObject (verified API, docs/ISAAC6_API.md); setting gravity at
run time is attempted three ways and the one that works is printed.
"""
from __future__ import annotations
import math
import os
import sys
import numpy as np
from scipy.ndimage import binary_dilation, binary_erosion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rubble as RB  # noqa: E402

OUT_CLEAR = 99.0


def _np(x):
    """torch / warp / numpy -> numpy."""
    try:
        import warp as wp
        if isinstance(x, wp.array):
            x = wp.to_torch(x)
    except Exception:
        pass
    if hasattr(x, "detach"):
        x = x.detach()
    if hasattr(x, "cpu"):
        x = x.cpu()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def rectangles(mask, value, tol=0.15, max_side=5):
    """Greedy split of a boolean mask into rectangles of similar `value` (e.g. clearance), max side cells."""
    m = mask.copy()
    H, W = m.shape
    out = []
    for r in range(H):
        for c in range(W):
            if not m[r, c]:
                continue
            v0 = value[r, c]
            ok = lambda rr, cc: m[rr, cc] and abs(value[rr, cc] - v0) < tol
            w = 1
            while c + w < W and w < max_side and ok(r, c + w):
                w += 1
            h = 1
            while r + h < H and h < max_side and all(ok(r + h, c + k) for k in range(w)):
                h += 1
            m[r:r + h, c:c + w] = False
            out.append((r, c, h, w))
    return out


class UsdWriter:
    """Writes the irregular rubble meshes straight into the USD stage (pxr), with PhysX physics schemas.
    Rigid pieces: Xform root (RigidBodyAPI + MassAPI, translate/orient ops) -> child Mesh `geo` (convex-hull
    collider) and optional child Mesh `rebar` (visual). Static pieces: Mesh with a triangle-mesh collider."""

    def __init__(self, log=print):
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt
        self.Gf, self.Sdf, self.UsdGeom, self.UsdPhysics, self.UsdShade, self.Vt = Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt
        try:
            from pxr import PhysxSchema
            self.Physx = PhysxSchema
        except Exception:
            self.Physx = None
        self.st = omni.usd.get_context().get_stage()
        self.log = log
        self._phys_mat = None

    def xform(self, path, pos=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0)):
        G = self.UsdGeom
        x = G.Xform.Define(self.st, path)
        x.AddTranslateOp(precision=G.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*[float(v) for v in pos]))
        x.AddOrientOp(precision=G.XformOp.PrecisionDouble).Set(self.Gf.Quatd(float(quat[0]), self.Gf.Vec3d(*[float(v) for v in quat[1:]])))
        x.AddScaleOp(precision=G.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(1.0, 1.0, 1.0))
        return x.GetPrim()

    def mesh(self, path, P, F, rgb=(0.5, 0.5, 0.5), collider=None, smooth=False):
        """collider: None (visual), 'convexHull' (dynamic) or 'none' (static triangle mesh)."""
        G, Vt = self.UsdGeom, self.Vt
        m = G.Mesh.Define(self.st, path)
        P = np.asarray(P, np.float32); F = np.asarray(F, np.int32)
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P))
        m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1)))
        m.CreateSubdivisionSchemeAttr("none")
        if smooth:
            m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(RB.smooth_normals(P, F))); m.SetNormalsInterpolation("vertex")
        else:
            m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(RB.normals(P, F))); m.SetNormalsInterpolation("faceVarying")
        lo, hi = P.min(0), P.max(0)
        m.CreateExtentAttr(Vt.Vec3fArray([self.Gf.Vec3f(*map(float, lo)), self.Gf.Vec3f(*map(float, hi))]))
        m.CreateDisplayColorAttr(Vt.Vec3fArray([self.Gf.Vec3f(*map(float, rgb))]))
        prim = m.GetPrim()
        if collider is not None:
            self.UsdPhysics.CollisionAPI.Apply(prim)
            self.UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(collider)
            self.UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.phys_mat(), self.UsdShade.Tokens.weakerThanDescendants, "physics")
        return prim

    def rigid(self, prim, mass, max_depen=2.0, ang_damp=0.6, lin_damp=0.05):
        self.UsdPhysics.RigidBodyAPI.Apply(prim)
        self.UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        if self.Physx is not None:
            try:
                api = self.Physx.PhysxRigidBodyAPI.Apply(prim)
                api.CreateMaxDepenetrationVelocityAttr(float(max_depen))
                api.CreateAngularDampingAttr(float(ang_damp))
                api.CreateLinearDampingAttr(float(lin_damp))
            except Exception:
                pass

    def phys_mat(self):
        """Rough concrete: high friction, almost no bounce, so broken pieces don't roll far."""
        if self._phys_mat is None:
            mat = self.UsdShade.Material.Define(self.st, "/World/Physics/concrete")
            api = self.UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            api.CreateStaticFrictionAttr(1.0); api.CreateDynamicFrictionAttr(0.85); api.CreateRestitutionAttr(0.05)
            self._phys_mat = mat
        return self._phys_mat

    def preview_material(self, path, rgb, roughness=0.6, metallic=0.0):
        S, Sdf = self.UsdShade, self.Sdf
        mat = S.Material.Define(self.st, path)
        sh = S.Shader.Define(self.st, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(self.Gf.Vec3f(*map(float, rgb)))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def bind(self, paths, mat):
        for p in paths:
            prim = self.st.GetPrimAtPath(p)
            if prim.IsValid():
                self.UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, self.UsdShade.Tokens.strongerThanDescendants)

    def set_visible(self, path, visible):
        prim = self.st.GetPrimAtPath(path)
        if prim.IsValid():
            im = self.UsdGeom.Imageable(prim)
            im.MakeVisible() if visible else im.MakeInvisible()


def _yaw_quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


class Quake:
    PRE, SHAKE, SETTLE = 2.0, 8.0, 2.5

    def __init__(self, scen, cfg, sim_utils, W3, wall_h=3.0, pga=0.45, seed=0, log=print, dust=1.0, reveal=True):
        self.s, self.cfg, self.su, self.W3, self.wall_h = scen, cfg, sim_utils, W3, wall_h
        self.pga, self.log = pga, log
        self.dust_scale = float(dust)          # debug-draw dust opacity (0 = off, e.g. when Flow dust is used)
        self.reveal_t = self.PRE + 2.4 if reveal else None    # rubble mounds/gravel appear as the upper floor comes down
        self._revealed = False
        self.usd = None
        self.rebar_paths = []
        self.bursts = []                       # (t, x, y, z, size): where/when debris lets go, for Flow dust emitters
        self.rng = np.random.default_rng(seed + 101)
        self.duration = self.PRE + self.SHAKE + self.SETTLE
        self.t = 0.0
        self.groups = {}            # name -> dict(obj=RigidObject, n, release (n,), kick (n,6), guide bool, target)
        self.gravity_mode = None
        self.dust = np.zeros((0, 7))       # x y z vx vy vz age
        self.posts = []
        b = scen.b
        c, rid = b.clear, b.room_id
        inside = c < OUT_CLEAR
        desk = np.zeros_like(inside)
        for (r, x) in getattr(b, "desk_cells", []):
            desk[r:r + 2, x:x + 3] |= np.isclose(c[r:r + 2, x:x + 3], 0.75)
        self.desk = desk
        self.collapsed = inside & (rid >= 0) & (c < 2.0) & ~desk
        self.rubble = inside & (rid >= 0) & (c < 0.12)
        self.panel_mask = self.collapsed & (c >= 0.12)
        self.walls = inside & (c < 0.12) & (rid < 0)
        self.damage = binary_dilation(self.collapsed, iterations=4) & inside
        rr, cc = np.nonzero(self.collapsed)
        self.epi = np.array([(cc.mean() + 0.5) * cfg.cell, (rr.mean() + 0.5) * cfg.cell]) if len(rr) else np.array([0.0, 0.0])
        self.spread = max(1.0, float(np.hypot((cc + 0.5) * cfg.cell - self.epi[0], (rr + 0.5) * cfg.cell - self.epi[1]).max())) if len(rr) else 1.0
        # rooms that lose their ceiling (for the dashboard banner)
        self.rooms_hit = sorted(int(k) for k in np.unique(rid[self.collapsed]) if k > 0
                                and self.collapsed[rid == k].mean() > 0.15)
        # ground motion: band-limited, two horizontal axes, peak `pga` g
        self.freqs = np.array([0.9, 1.4, 2.1, 2.9, 3.8])
        self.ph = self.rng.uniform(0, 2 * np.pi, (2, len(self.freqs)))
        self.amp = 1.0 / self.freqs ** 0.5
        tt = np.linspace(0, self.SHAKE, 2000)
        self.norm = np.abs(self._raw(tt)).max(axis=1) + 1e-9

    # ------------------------------------------------------------------ ground motion
    def _raw(self, tau):
        tau = np.atleast_1d(tau)
        return np.stack([(self.amp[None] * np.sin(2 * np.pi * self.freqs[None] * tau[:, None] + self.ph[k][None])).sum(1)
                         for k in range(2)])

    def envelope(self, tau):
        if tau < 0 or tau > self.SHAKE:
            return 0.0
        return min(1.0, tau / 1.2) * (1.0 if tau < 4.5 else math.exp(-(tau - 4.5) / 1.3))

    def accel(self, t):
        """Horizontal ground acceleration (m/s^2, world X/Y) at quake time t."""
        tau = t - self.PRE
        e = self.envelope(tau)
        if e == 0:
            return np.zeros(2)
        a = self._raw(tau)[:, 0] / self.norm
        return self.pga * 9.81 * e * np.array([a[0], 0.7 * a[1]])

    def camera_offset(self, t):
        """Visible ground displacement for the camera (the building moves with the ground)."""
        tau = t - self.PRE
        e = self.envelope(tau)
        if e == 0:
            return np.zeros(3)
        return e * np.array([0.10 * math.sin(2 * math.pi * 1.7 * tau + 0.3) + 0.04 * math.sin(2 * math.pi * 4.3 * tau),
                             0.08 * math.sin(2 * math.pi * 1.3 * tau + 1.1) + 0.03 * math.sin(2 * math.pi * 3.7 * tau),
                             0.03 * math.sin(2 * math.pi * 2.9 * tau)])

    @property
    def done(self):
        return self.t >= self.duration

    @property
    def shaking(self):
        return self.envelope(self.t - self.PRE) > 0.05

    def banner(self):
        tau = self.t - self.PRE
        rooms = ", ".join(str(r) for r in self.rooms_hit) or "the east wing"
        if tau < 0:
            return "Before the earthquake · the drone team waits on the coordinator outside"
        if tau < self.SHAKE:
            return f"EARTHQUAKE · strong shaking ({self.pga:.2f} g) · upper floor collapsing onto rooms {rooms}"
        return "Shaking stopped · dust settling · launching the drone team"

    # ------------------------------------------------------------------ building (static parts, with colliders)
    def build_static(self, boxes, collide):
        """Intact walls (boxes), damaged walls with ragged broken tops and exposed rebar (meshes), rubble mounds
        as a height field, gravel, desks. `boxes(name, mask, z0, z1, rgb)` builds a batch of boxes and returns
        its prim paths; `collide(paths)` gives them static colliders. Meshes get triangle-mesh colliders."""
        b, cfg, H, rng = self.s.b, self.cfg, self.wall_h, self.rng
        c, cell, W3 = b.clear, cfg.cell, self.W3
        if self.usd is None:
            self.usd = UsdWriter(self.log)
        U = self.usd
        wall_ok = self.walls & ~self.damage
        wall_dmg = self.walls & self.damage
        paths = boxes("walls", wall_ok, 0.0, H, (0.62, 0.62, 0.60))
        if paths:
            collide(paths)
        # damaged walls: straight runs (horizontal first, the rest vertical), one ragged mesh per run
        from scipy.ndimage import gaussian_filter
        brk = np.clip(gaussian_filter(rng.uniform(1.7, 2.8, c.shape), 0.8), 1.8, 2.75)
        self.wall_top = np.full(c.shape, H)
        U.xform("/World/Building/walls/broken")
        used = np.zeros_like(wall_dmg)
        runs = []
        for r in range(c.shape[0]):
            x = 0
            while x < c.shape[1]:
                if wall_dmg[r, x]:
                    x0 = x
                    while x < c.shape[1] and wall_dmg[r, x]:
                        x += 1
                    if x - x0 >= 2:
                        runs.append(("h", [(r, k) for k in range(x0, x)])); used[r, x0:x] = True
                else:
                    x += 1
        rem = wall_dmg & ~used
        for x in range(c.shape[1]):
            r = 0
            while r < c.shape[0]:
                if rem[r, x]:
                    r0 = r
                    while r < c.shape[0] and rem[r, x]:
                        r += 1
                    runs.append(("v", [(k, x) for k in range(r0, r)]))
                else:
                    r += 1
        self.wall_dir = {}                                     # cell -> "h" (wall runs along map x) or "v" (along map y)
        for kind, cells in runs:
            for (r, x) in cells:
                if len(cells) == 1:                            # lone cell: follow whichever neighbour wall it joins
                    nb_h = any(0 <= x + d < c.shape[1] and self.walls[r, x + d] for d in (-1, 1))
                    nb_v = any(0 <= r + d < c.shape[0] and self.walls[r + d, x] for d in (-1, 1))
                    kind = "h" if nb_h and not nb_v else "v"
                self.wall_dir[(r, x)] = kind
        for i, (kind, cells) in enumerate(runs):
            n = len(cells)
            L = n * cell
            tops = np.array([brk[rc] for rc in cells])
            (P, F), bars, prof, xs = RB.wall_run(rng, L, cell, tops)
            for k, rc in enumerate(cells):                     # where the falling top piece of each cell sits
                sel = (xs >= -L / 2 + k * cell - 1e-6) & (xs <= -L / 2 + (k + 1) * cell + 1e-6)
                self.wall_top[rc] = float(prof[sel].max()) + 0.01
            (r0, x0), (r1, x1) = cells[0], cells[-1]
            mx, my = (x0 + x1 + 1) / 2 * cell, (r0 + r1 + 1) / 2 * cell
            yaw = 0.0 if kind == "h" else -math.pi / 2               # local +x along increasing column / row
            root = f"/World/Building/walls/broken/w{i:03d}"
            U.xform(root, W3((mx, my), 0.0), _yaw_quat(yaw))
            U.mesh(root + "/geo", P, F, (0.60, 0.59, 0.56), collider="none")
            if len(bars[0]):
                U.mesh(root + "/rebar", *bars, (0.25, 0.14, 0.09), smooth=True)
                self.rebar_paths.append(root + "/rebar")
        # rubble mounds: one height field over the rubble cells (static triangle-mesh collider)
        heap = 1.2 + 0.9 * np.random.default_rng(cfg.seed + 11).random(c.shape)
        U.xform("/World/Building/rubble")
        P, F, self.heap_z = RB.heap(0.55 * heap, self.rubble, cell, rng)
        if len(F):
            U.mesh("/World/Building/rubble/heap", P, F, (0.42, 0.39, 0.35), collider="none", smooth=True)
        rr, cc = np.nonzero(self.rubble)
        self.heap_base = np.zeros(c.shape)
        if len(rr):
            self.heap_base[rr, cc] = self.heap_z((cc + 0.5) * cell, -(rr + 0.5) * cell)
        # gravel: small stones on the mounds and scattered over the floor of the damaged zone (visual only)
        xy, z0, small = [], [], []
        for k in range(len(rr)):
            for _ in range(6):
                x, y = (cc[k] + rng.random()) * cell, -(rr[k] + rng.random()) * cell
                xy.append((x, y)); z0.append(float(self.heap_z(x, y))); small.append(False)
        floor = self.damage & ~self.walls & ~self.rubble & ~self.desk & (c < OUT_CLEAR)
        fr, fc = np.nonzero(floor)
        for k in range(len(fr)):
            for _ in range(rng.integers(0, 3)):
                xy.append(((fc[k] + rng.random()) * cell, -(fr[k] + rng.random()) * cell)); z0.append(0.0); small.append(True)
        parts = []
        for (x, y), z, sm in zip(xy, z0, small):
            sz = rng.uniform(0.015, 0.045) if sm else rng.uniform(0.03, 0.11)
            Pp, Fp = RB.rock(rng, sz, sz * rng.uniform(0.6, 1.0), sz * rng.uniform(0.35, 0.7), n=9)
            Pp = Pp + np.array([x, y, z - Pp[:, 2].min() - 0.3 * sz])
            parts.append((Pp, Fp))
        if parts:
            P, F = RB.merge(parts)
            U.mesh("/World/Building/rubble/gravel", P, F, (0.47, 0.44, 0.40))
        self.log(f"[quake] static: {len(runs)} broken wall runs, rubble mound over {len(rr)} cells, {len(parts)} stones of gravel")
        # gaps too tight for any drone under the collapse: low solid base (boxes)
        tight = (c >= 0.12) & (c < 0.25) & self.collapsed
        paths = boxes("tight", tight, 0.0, np.clip(c, 0.12, 0.25), (0.45, 0.41, 0.36))
        if paths:
            collide(paths)
        paths = boxes("desks", self.desk, 0.70, 0.76, (0.55, 0.42, 0.30))
        if paths:
            collide(paths)
        legs = np.zeros_like(self.desk)
        for (r, x) in getattr(b, "desk_cells", []):
            if self.desk[r, x]:
                legs[r, x] = legs[r, x + 2] = True
        boxes("desk_legs", legs, 0.0, 0.70, (0.25, 0.25, 0.27), per_cell=True, thin=0.06)
        if self.reveal_t is not None:
            U.set_visible("/World/Building/rubble", False)

    # ------------------------------------------------------------------ dynamic parts
    def _spawn_group(self, name, items, RigidObject, RigidObjectCfg):
        """items: list of dict(size, pos, rgb, mass, release, kick(6), guide, target(3)). Spawned now, before sim.reset()."""
        su = self.su
        if not items:
            return
        for i, it in enumerate(items):
            cfgb = su.CuboidCfg(size=tuple(float(v) for v in it["size"]),
                                visual_material=su.PreviewSurfaceCfg(diffuse_color=tuple(it["rgb"]), roughness=0.9),
                                rigid_props=su.RigidBodyPropertiesCfg(max_depenetration_velocity=2.0),
                                mass_props=su.MassPropertiesCfg(mass=float(it["mass"])),
                                collision_props=su.CollisionPropertiesCfg())
            cfgb.func(f"/World/Quake/{name}/p_{i:04d}", cfgb, translation=tuple(float(v) for v in it["pos"]))
        self._register(name, items, RigidObject, RigidObjectCfg)

    def _spawn_mesh_group(self, name, items, RigidObject, RigidObjectCfg):
        """Irregular rigid pieces: items carry mesh=(P, F) in the piece frame (+ optional rebar=(P, F)), pos, quat."""
        if not items:
            return
        U = self.usd
        U.xform(f"/World/Quake/{name}")
        for i, it in enumerate(items):
            root = f"/World/Quake/{name}/p_{i:04d}"
            prim = U.xform(root, it["pos"], it.get("quat", (1.0, 0.0, 0.0, 0.0)))
            U.rigid(prim, it["mass"], ang_damp=it.get("ang_damp", 0.6))
            U.mesh(root + "/geo", *it["mesh"], it["rgb"], collider="convexHull")
            if it.get("rebar") is not None and len(it["rebar"][0]):
                U.mesh(root + "/rebar", *it["rebar"], (0.25, 0.14, 0.09), smooth=True)
                self.rebar_paths.append(root + "/rebar")
        self._register(name, items, RigidObject, RigidObjectCfg)

    def _register(self, name, items, RigidObject, RigidObjectCfg):
        obj = None
        try:
            obj = RigidObject(RigidObjectCfg(prim_path=f"/World/Quake/{name}/p_.*", spawn=None))
        except Exception as e:
            self.log(f"[quake] RigidObject for {name} failed ({type(e).__name__}: {e}); pieces fall freely from the start")
        self.groups[name] = dict(obj=obj, n=len(items),
                                 release=np.array([it.get("release", 0.0) for it in items]),
                                 kick=np.array([it.get("kick", np.zeros(6)) for it in items], float),
                                 guide=np.array([it.get("guide", False) for it in items]),
                                 target=np.array([it.get("target", it["pos"]) for it in items], float),
                                 released=np.zeros(len(items), bool), hold=None)
        self.log(f"[quake] {name}: {len(items)} rigid bodies")

    def build_dynamic(self, RigidObject, RigidObjectCfg):
        b, cfg, H, W3, rng = self.s.b, self.cfg, self.wall_h, self.W3, self.rng
        cell = cfg.cell
        c = b.clear
        if self.usd is None:
            self.usd = UsdWriter(self.log)
        dist = lambda x, y: float(np.hypot(x - self.epi[0], y - self.epi[1])) / self.spread
        grey = lambda base, j=0.04: tuple(np.clip(np.array(base) + rng.normal(0, j), 0, 1))

        def put(mesh, x, y, z_bot):
            """World position of a piece whose mesh bottom should sit at z_bot."""
            return W3((x, y), z_bot - float(mesh[0][:, 2].min()))

        # 1) upper-floor slab: broken slab pieces with rebar over the voids; each lands at the void height in the map
        panels, frags, ledges = [], [], []
        T = 0.18
        for (r, x, h, w) in rectangles(self.panel_mask, c, max_side=4):
            zr = float(np.median(c[r:r + h, x:x + w]))
            cx, cy = (x + w / 2) * cell, (r + h / 2) * cell
            sx, sy = w * cell - 0.02, h * cell - 0.02
            d = dist(cx, cy)
            rel = self.PRE + 2.4 + 1.6 * d + rng.uniform(0, 0.3)
            kick = np.r_[0, 0, -0.5, rng.normal(0, 0.4, 3)]
            mesh = RB.slab(rng, sx, sy, T)
            panels.append(dict(mesh=mesh, rebar=RB.rebar(rng, sx, sy, 0.0), pos=put(mesh, cx, cy, H), rgb=grey((0.56, 0.54, 0.51), 0.02),
                               mass=2400 * sx * sy * T, release=rel, kick=kick, guide=True,
                               target=put(mesh, cx, cy, zr)))
            ledges.append((cx, cy, sx, sy, zr))
            # loose concrete lying on the slab: rides it down (pancake debris)
            for _ in range(1 + int(sx * sy / 0.6)):
                sz = rng.uniform(0.10, 0.28)
                m2 = RB.rock(rng, sz * 1.2, sz * rng.uniform(0.8, 1.2), sz * rng.uniform(0.5, 0.8))
                fx, fy = cx + rng.uniform(-0.35, 0.35) * sx, cy + rng.uniform(-0.35, 0.35) * sy
                frags.append(dict(mesh=m2, pos=put(m2, fx, fy, H + T + 0.03), rgb=grey((0.52, 0.49, 0.45)),
                                  mass=2400 * 0.45 * sz ** 3, release=rel + 0.02, kick=kick + np.r_[rng.normal(0, 0.3, 2), 0, rng.normal(0, 1.5, 3)]))
        # 2) the concrete mass above the rubble zone: a packed layer of chipped blocks (reads as the upper floor
        #    before the quake), broken lumps on top, and smaller pieces; a cell's stack lets go together
        chunks = []
        rr, cc = np.nonzero(self.rubble)
        for k in range(len(rr)):
            x, y = (cc[k] + 0.5) * cell, (rr[k] + 0.5) * cell
            base = self.PRE + 1.8 + 1.4 * dist(x, y) + rng.uniform(0, 0.35)
            z = H + 0.02
            stack = [("block", cell * 0.96, cell * 0.96, rng.uniform(0.25, 0.38))]
            if rng.random() < 0.45:
                stack.append(("rock", cell * 0.85, cell * 0.8, rng.uniform(0.22, 0.34)))
            for _ in range(rng.integers(1, 4)):
                sz = rng.uniform(0.10, 0.24)
                stack.append(("rock", sz * 1.2, sz * rng.uniform(0.8, 1.1), sz * rng.uniform(0.5, 0.8)))
            for j, (kind, a_, b_, h_) in enumerate(stack):
                mesh = RB.block(rng, a_, b_, h_) if kind == "block" else RB.rock(rng, a_, b_, h_)
                ext = np.ptp(mesh[0], 0)
                ox, oy = (0.0, 0.0) if j < 2 else tuple(rng.uniform(-1, 1, 2) * max(0.0, (cell - ext[0]) / 2 - 0.01))
                chunks.append(dict(mesh=mesh, pos=put(mesh, x + ox, y + oy, z), rgb=grey((0.50, 0.47, 0.43)),
                                   mass=2400 * 0.6 * a_ * b_ * h_, release=base + 0.04 * j,
                                   kick=np.r_[rng.normal(0, 0.6, 2), -1.0, rng.normal(0, 2.5, 3)]))
                z += float(ext[2]) + 0.01
        # 3) wall tops over damaged walls: the part above the ragged break falls off, often in two pieces
        tops = []
        wr, wc = np.nonzero(self.walls & self.damage)
        for k in range(len(wr)):
            r, x = wr[k], wc[k]
            h0 = float(self.wall_top[r, x])
            hh = H - h0 - 0.01
            if hh < 0.12:
                continue
            px, py = (x + 0.5) * cell, (r + 0.5) * cell
            # Push across the wall (along its normal), to the side facing the collapse. Pushing straight at
            # the epicentre jammed the tops of walls that run towards it into their neighbours.
            if self.wall_dir.get((int(r), int(x)), "v") == "h":
                away = np.array([0.0, 1.0 if self.epi[1] >= py else -1.0])
            else:
                away = np.array([1.0 if self.epi[0] >= px else -1.0, 0.0])
            rel = self.PRE + rng.uniform(0.8, 3.8) + 0.3 * ((r + x) % 2)     # neighbours never leave together
            parts = [hh] if hh < 0.55 else [hh * (u := rng.uniform(0.4, 0.6)), hh * (1 - u) - 0.01]
            zb = h0
            for j, ph in enumerate(parts):
                mesh = RB.block(rng, cell - 0.02, cell - 0.02, ph, chip=0.12)
                # Shaking alone can't slide them off: the tilt (0.45 g) is below concrete friction. So the
                # top is pushed off the wall towards the collapse and tipped over the edge.
                v = rng.uniform(2.4, 3.4)
                tip = np.array([away[1], away[0], 0.0]) * rng.uniform(2.5, 4.0)    # world axis to topple about (the wall's own axis)
                tops.append(dict(mesh=mesh, pos=put(mesh, px, py, zb), rgb=grey((0.60, 0.59, 0.56), 0.015),
                                 mass=2000 * cell * cell * ph, release=rel - 0.25 * j,
                                 kick=np.r_[v * away[0], -v * away[1], 0.6, tip + rng.normal(0, 0.8, 3)]))
                zb += float(np.ptp(mesh[0][:, 2])) + 0.01
        # 4) unanchored furniture in open parts of rooms: slides and topples in the shaking (never held)
        furn = []
        open_room = (c >= 2.0) & (c < OUT_CLEAR) & (b.room_id > 0)
        near_wall = open_room & binary_dilation(c < 0.12, iterations=2) & ~binary_dilation(c < 0.12, iterations=1)
        kinds = [("bookshelf", (0.9, 0.35, 1.9), 45, (0.45, 0.33, 0.22), near_wall),
                 ("cabinet", (0.45, 0.6, 1.3), 55, (0.55, 0.57, 0.60), near_wall),
                 ("chair", (0.5, 0.5, 0.9), 9, (0.12, 0.12, 0.14), binary_erosion(open_room, iterations=2)),
                 ("printer", (0.6, 0.5, 0.45), 25, (0.85, 0.85, 0.83), binary_erosion(open_room, iterations=2))]
        for rid in sorted(k for k in b.rooms if k > 0):
            for kind, size, mass, rgb, where in kinds:
                cand = np.argwhere(where & (b.room_id == rid))
                if len(cand) == 0 or rng.random() < 0.35:
                    continue
                r, x = cand[rng.integers(len(cand))]
                yaw_long = rng.random() < 0.5
                sz = (size[1], size[0], size[2]) if yaw_long else size
                furn.append(dict(size=sz, pos=W3(((x + 0.5) * cell, (r + 0.5) * cell), size[2] / 2 + 0.01), rgb=rgb,
                                 mass=mass, release=0.0, kind=kind))
        from types import SimpleNamespace
        self._ns = SimpleNamespace(panels=panels, chunks=chunks, tops=tops, furn=furn, frags=frags)
        for name, items in (("panels", panels), ("fragments", frags), ("chunks", chunks), ("walltops", tops)):
            self._spawn_mesh_group(name, items, RigidObject, RigidObjectCfg)
        self._spawn_group("furniture", furn, RigidObject, RigidObjectCfg)
        # invisible ledges the panels land on (crushed furniture/debris), plus visible debris posts
        su = self.su
        for i, (cx, cy, sx, sy, zr) in enumerate(ledges):
            led = su.CuboidCfg(size=(sx * 0.92, sy * 0.92, 0.04), visible=False, collision_props=su.CollisionPropertiesCfg())
            led.func(f"/World/Quake/ledges/l_{i:04d}", led, translation=tuple(W3((cx, cy), zr - 0.02)))
            for (ox, oy) in ((-0.35, -0.35), (0.35, 0.35)):
                if zr > 0.2:
                    self.posts.append(((cx + ox * sx, cy + oy * sy), zr))
        n_all = sum(len(v) for v in (panels, frags, chunks, tops, furn))
        self.log(f"[quake] {n_all} rigid bodies: {len(panels)} slab pieces (+rebar), {len(frags)} loose pieces on them, "
                 f"{len(chunks)} concrete blocks/lumps, {len(tops)} wall-top pieces, {len(furn)} furniture; "
                 f"epicentre of damage ({self.epi[0]:.1f}, {self.epi[1]:.1f}) m; rooms hit {self.rooms_hit}")

    def posts_boxes(self, boxes_xy=None):
        """Crushed-debris stacks under the slab corners (visual only, one merged mesh; boxes_xy is unused)."""
        if not self.posts:
            return
        rng, parts = self.rng, []
        for (xy, zr) in self.posts:
            x, y = self.W3(xy)[:2]
            z = 0.0
            while z < zr - 0.04:
                s = min(rng.uniform(0.12, 0.2), zr - z + 0.02)
                P, F = RB.rock(rng, rng.uniform(0.14, 0.2), rng.uniform(0.12, 0.18), max(s, 0.05), n=12)
                P = P + np.array([x + rng.normal(0, 0.015), y + rng.normal(0, 0.015), z - P[:, 2].min()])
                parts.append((P, F))
                z += float(np.ptp(P[:, 2])) * 0.9
        P, F = RB.merge(parts)
        self.usd.mesh("/World/Building/rubble/posts", P, F, (0.40, 0.37, 0.33))

    def after_look(self):
        """Rebar is steel: rebind it after look.apply() (which binds concrete on everything under /World/Quake)."""
        if not self.rebar_paths or self.usd is None:
            return
        try:
            mat = self.usd.preview_material("/World/Looks/rebar_steel", (0.22, 0.12, 0.07), roughness=0.55, metallic=0.7)
            self.usd.bind(self.rebar_paths, mat)
            self.log(f"[quake] rusty steel on {len(self.rebar_paths)} rebar meshes")
        except Exception as e:
            self.log(f"[quake] rebar material skipped ({type(e).__name__}: {e})")

    # ------------------------------------------------------------------ runtime
    def after_reset(self, sim):
        self.sim = sim
        for name, g in self.groups.items():
            o = g["obj"]
            if o is None:
                continue
            try:
                o.update(0.0)
                g["hold"] = _np(self._pose(o)).copy()
            except Exception as e:
                self.log(f"[quake] {name}: cannot read poses ({type(e).__name__}: {e}); no hold")
                g["obj"] = None

    @staticmethod
    def _pose(o):
        d = o.data
        for a in ("root_pose_w", "root_link_pose_w"):
            if hasattr(d, a):
                return getattr(d, a)
        st = d.root_state_w
        return st[:, :7]

    @staticmethod
    def _vel(o):
        d = o.data
        for a in ("root_vel_w", "root_com_vel_w"):
            if hasattr(d, a):
                return getattr(d, a)
        return d.root_state_w[:, 7:13]

    def _write(self, o, ids, pose=None, vel=None):
        import torch
        dev = getattr(o, "device", None) or "cuda:0"
        idt = torch.as_tensor(np.asarray(ids, dtype=np.int64), device=dev)
        if pose is not None:
            p = torch.as_tensor(np.asarray(pose, np.float32), device=dev)
            for fn in ("write_root_pose_to_sim", "write_root_link_pose_to_sim"):
                if hasattr(o, fn):
                    getattr(o, fn)(p, env_ids=idt); break
        if vel is not None:
            v = torch.as_tensor(np.asarray(vel, np.float32), device=dev)
            for fn in ("write_root_velocity_to_sim", "write_root_com_velocity_to_sim"):
                if hasattr(o, fn):
                    getattr(o, fn)(v, env_ids=idt); break

    def set_gravity(self, g):
        g = [float(v) for v in g]
        if self.gravity_mode in (None, "physics_sim_view"):
            try:
                import carb
                for owner in (self.sim, getattr(self.sim, "physics_manager", None), getattr(self.sim, "_physics_manager", None)):
                    if owner is None:
                        continue
                    for att in ("physics_sim_view", "_physics_sim_view", "get_physics_sim_view"):
                        v = getattr(owner, att, None)
                        v = v() if callable(v) and att.startswith("get") else v
                        if v is not None and hasattr(v, "set_gravity"):
                            v.set_gravity(carb.Float3(*g))
                            if self.gravity_mode is None:
                                self.log(f"[quake] ground motion via physics_sim_view.set_gravity ({type(owner).__name__}.{att})")
                            self.gravity_mode = "physics_sim_view"
                            return True
            except Exception as e:
                if self.gravity_mode is None:
                    self.log(f"[quake] physics_sim_view gravity failed: {type(e).__name__}: {e}")
        if self.gravity_mode in (None, "usd"):
            try:
                import omni.usd
                from pxr import UsdPhysics, Gf
                stage = omni.usd.get_context().get_stage()
                path = getattr(getattr(self.sim, "cfg", None), "physics_prim_path", "/physicsScene")
                scene = UsdPhysics.Scene(stage.GetPrimAtPath(path))
                mag = float(np.linalg.norm(g))
                scene.GetGravityDirectionAttr().Set(Gf.Vec3f(*(np.array(g) / mag)))
                scene.GetGravityMagnitudeAttr().Set(mag)
                if self.gravity_mode is None:
                    self.log(f"[quake] ground motion via USD physics scene gravity ({path}); check that furniture slides")
                self.gravity_mode = "usd"
                return True
            except Exception as e:
                if self.gravity_mode is None:
                    self.log(f"[quake] USD gravity failed: {type(e).__name__}: {e}")
        if self.gravity_mode is None:
            self.log("[quake] gravity can't be changed at run time: applying ground motion as velocity changes instead")
        self.gravity_mode = "velocity"
        return False

    def pre_step(self, dt):
        """Call every physics frame before sim.step() while not done."""
        t = self.t
        a = self.accel(t)
        if self.gravity_mode != "velocity":
            self.set_gravity((-a[0], a[1], -9.81))       # map +y is world -Y (W3 flips y)
        for name, g in self.groups.items():
            o = g["obj"]
            if o is None or g["hold"] is None:
                continue
            due = ~g["released"] & (g["release"] <= t)
            held = np.nonzero(~g["released"] & ~due)[0]
            if len(held):
                self._write(o, held, pose=g["hold"][held], vel=np.zeros((len(held), 6)))
            new = np.nonzero(due)[0]
            if len(new):
                g["released"][new] = True
                self._write(o, new, vel=g["kick"][new])
                if name in ("panels", "chunks", "walltops"):
                    self._emit_dust(g["hold"][new, :3], n_each={"panels": 4, "chunks": 1, "walltops": 1}[name])
                    for p in g["hold"][new, :3]:
                        self.bursts.append((t, float(p[0]), float(p[1]), float(p[2]), {"panels": 1.0, "chunks": 0.4, "walltops": 0.3}[name]))
            live = np.nonzero(g["released"])[0]
            if len(live) and (self.gravity_mode == "velocity" and self.envelope(t - self.PRE) > 0 or g["guide"].any()):
                try:
                    v = _np(self._vel(o))[live].copy()
                    if self.gravity_mode == "velocity":
                        v[:, 0] += -a[0] * dt; v[:, 1] += a[1] * dt
                    gd = g["guide"][live]
                    if gd.any():
                        p = _np(self._pose(o))[live][gd]
                        tgt = g["target"][live][gd]
                        v[gd, 0:2] = np.clip((tgt[:, 0:2] - p[:, 0:2]) * 3.0, -1.5, 1.5)
                        v[gd, 3:6] *= 0.6
                        landed = p[:, 2] < tgt[:, 2] + 0.05
                        v[np.nonzero(gd)[0][landed], 0:2] *= 0.2
                    self._write(o, live, vel=v)
                except Exception as e:
                    if not getattr(self, "_warned", False):
                        self.log(f"[quake] guide/pseudo-force skipped ({type(e).__name__}: {e})")
                        self._warned = True
        if self.reveal_t is not None and not self._revealed and t >= self.reveal_t:
            self.usd.set_visible("/World/Building/rubble", True)        # mounds and gravel appear as the floor comes down
            self._revealed = True
        self._step_dust(dt)
        self.t += dt
        if self.done and self.gravity_mode != "velocity":
            self.set_gravity((0.0, 0.0, -9.81))

    def post_step(self, dt):
        for g in self.groups.values():
            if g["obj"] is not None:
                try:
                    g["obj"].update(dt)
                except Exception:
                    pass

    # ------------------------------------------------------------------ dust (drawn as debug-draw points)
    def _emit_dust(self, pos, n_each=10):
        rng = self.rng
        P = np.repeat(np.asarray(pos, float), n_each, axis=0)
        P[:, 2] = np.minimum(P[:, 2], 1.0) + rng.uniform(-0.2, 0.6, len(P))
        V = np.c_[rng.normal(0, 0.9, (len(P), 2)), rng.uniform(0.1, 1.2, len(P))]
        self.dust = np.r_[self.dust, np.c_[P, V, np.zeros(len(P))]][-3500:]

    def _step_dust(self, dt):
        if not len(self.dust):
            return
        d = self.dust
        d[:, 0:3] += d[:, 3:6] * dt
        d[:, 3:6] *= (1 - 0.9 * dt)
        d[:, 5] -= 0.05 * dt
        d[:, 2] = np.maximum(d[:, 2], 0.05)
        d[:, 6] += dt
        self.dust = d[d[:, 6] < 20.0]

    def dust_points(self, t_mission=0.0):
        """(points, rgba, sizes) for debug draw; dust lingers into the first ~20 s of the mission."""
        if not len(self.dust) or self.dust_scale <= 0:
            return [], [], []
        d = self.dust                         # positions freeze once the mission runs; the dust only fades
        age = d[:, 6] + t_mission
        alpha = np.clip(0.16 * (1 - age / 20.0), 0, 0.16) * min(self.dust_scale, 1.0 / 0.16)
        keep = alpha > 0.02
        pts = [tuple(map(float, p)) for p in d[keep, 0:3]]
        cols = [(0.66, 0.62, 0.56, float(a)) for a in alpha[keep]]
        sizes = [float(s) for s in np.clip(8 + 1.5 * age[keep], 8, 22)]
        return pts, cols, sizes

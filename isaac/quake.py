"""A physically simulated earthquake for isaac/earthquake_scene.py (--collapse).

Sequence (seconds of physics before the drones launch):
  0 .. PRE           the building stands; the upper floor over the damaged zone is still up
  PRE .. PRE+SHAKE   strong ground motion. In the building's frame a horizontal ground acceleration a(t) is
                     exactly equivalent to tilting gravity, so gravity = (-a_x, -a_y, -9.81): unanchored
                     furniture slides and topples, loose blocks fall off walls, and the camera shakes.
                     Upper-floor slab panels, wall tops and the concrete mass above the rubble zone are
                     released in stages, nearest to the epicentre of the damage first, and fall under PhysX.
  .. +SETTLE         dust hangs, debris settles; then the mission (scenario.step) starts.

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
import numpy as np
from scipy.ndimage import binary_dilation, binary_erosion

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


class Quake:
    PRE, SHAKE, SETTLE = 2.0, 8.0, 2.5

    def __init__(self, scen, cfg, sim_utils, W3, wall_h=3.0, pga=0.45, seed=0, log=print):
        self.s, self.cfg, self.su, self.W3, self.wall_h = scen, cfg, sim_utils, W3, wall_h
        self.pga, self.log = pga, log
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
        """Walls (jagged where damaged), rubble heap bases, desks. `boxes(name, mask, z0, z1, rgb)` builds a batch
        and returns its prim paths; `collide(paths)` gives them static colliders."""
        b, cfg, H = self.s.b, self.cfg, self.wall_h
        c = b.clear
        wall_ok = self.walls & ~self.damage
        wall_dmg = self.walls & self.damage
        self.wall_top = np.where(wall_dmg, self.rng.uniform(1.9, 2.7, c.shape), H)
        heap = 1.2 + 0.9 * np.random.default_rng(cfg.seed + 11).random(c.shape)
        self.heap_base = 0.55 * heap
        tight = (c >= 0.12) & (c < 0.25) & self.collapsed
        for name, mask, z0, z1, rgb in [
            ("walls", wall_ok, 0.0, H, (0.62, 0.62, 0.60)),
            ("walls_broken", wall_dmg, 0.0, self.wall_top, (0.60, 0.59, 0.56)),
            ("rubble_base", self.rubble, 0.0, self.heap_base, (0.42, 0.39, 0.35)),
            ("tight_base", tight, 0.0, np.clip(c, 0.12, 0.25), (0.45, 0.41, 0.36)),
            ("desks", self.desk, 0.70, 0.76, (0.55, 0.42, 0.30)),
        ]:
            paths = boxes(name, mask, z0, z1, rgb, per_cell=(name == "walls_broken"))
            if paths:
                collide(paths)
        # desk legs (visual)
        legs = np.zeros_like(self.desk)
        for (r, x) in getattr(b, "desk_cells", []):
            if self.desk[r, x]:
                legs[r, x] = legs[r, x + 2] = True
        boxes("desk_legs", legs, 0.0, 0.70, (0.25, 0.25, 0.27), per_cell=True, thin=0.06)

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
        dist = lambda x, y: float(np.hypot(x - self.epi[0], y - self.epi[1])) / self.spread
        # 1) upper-floor slab panels over voids: fall and land at the void height from the map
        panels, ledges = [], []
        T = 0.18
        for (r, x, h, w) in rectangles(self.panel_mask, c):
            zr = float(np.median(c[r:r + h, x:x + w]))
            cx, cy = (x + w / 2) * cell, (r + h / 2) * cell
            sx, sy = w * cell - 0.03, h * cell - 0.03
            d = dist(cx, cy)
            panels.append(dict(size=(sx, sy, T), pos=W3((cx, cy), H + T / 2), rgb=(0.56, 0.54, 0.51),
                               mass=2400 * sx * sy * T, release=self.PRE + 2.4 + 1.6 * d + rng.uniform(0, 0.3),
                               kick=np.r_[0, 0, -0.5, rng.normal(0, 0.4, 3)], guide=True, target=W3((cx, cy), zr + T / 2)))
            ledges.append((cx, cy, sx, sy, zr))
        # 2) concrete mass above the rubble zone (upper floor + upper-storey wall remains), two layers of blocks
        chunks = []
        rr, cc = np.nonzero(self.rubble)
        for k in range(len(rr)):
            x, y = (cc[k] + 0.5) * cell, (rr[k] + 0.5) * cell
            for layer in (0, 1):
                if layer == 1 and rng.random() < 0.55:
                    continue
                s = np.array([cell * rng.uniform(0.8, 1.0), cell * rng.uniform(0.8, 1.0), rng.uniform(0.25, 0.4)])
                z = H + 0.02 + layer * 0.42 + s[2] / 2
                d = dist(x, y)
                chunks.append(dict(size=s, pos=W3((x + rng.normal(0, 0.03), y + rng.normal(0, 0.03)), z),
                                   rgb=tuple(np.clip(np.array([0.50, 0.47, 0.43]) + rng.normal(0, 0.04), 0, 1)),
                                   mass=2400 * s.prod(), release=self.PRE + 1.8 + 1.4 * d + 0.25 * layer + rng.uniform(0, 0.35),
                                   kick=np.r_[rng.normal(0, 0.6, 2), -1.0, rng.normal(0, 2.5, 3)]))
        # 3) wall tops over damaged walls: the part above the jagged break falls off
        tops = []
        wr, wc = np.nonzero(self.walls & self.damage)
        for k in range(len(wr)):
            r, x = wr[k], wc[k]
            h0 = float(self.wall_top[r, x])
            if H - h0 < 0.15:
                continue
            px, py = (x + 0.5) * cell, (r + 0.5) * cell
            away = np.array([px - self.epi[0], py - self.epi[1]])
            away = -away / (np.linalg.norm(away) + 1e-6)            # wall tops fall towards the collapse
            tops.append(dict(size=(cell - 0.02, cell - 0.02, H - h0 - 0.02), pos=W3((px, py), (H + h0) / 2),
                             rgb=(0.60, 0.59, 0.56), mass=2000 * cell * cell * (H - h0),
                             release=self.PRE + rng.uniform(0.8, 3.8),
                             kick=np.r_[1.2 * away[0], -1.2 * away[1], 0.3, rng.normal(0, 1.5, 3)]))
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
                                 mass=mass, release=0.0))
        from types import SimpleNamespace
        self._ns = SimpleNamespace(panels=panels, chunks=chunks, tops=tops, furn=furn)
        for name, items in (("panels", panels), ("chunks", chunks), ("walltops", tops), ("furniture", furn)):
            self._spawn_group(name, items, RigidObject, RigidObjectCfg)
        # invisible ledges the panels land on (crushed furniture/debris), plus visible debris posts
        su = self.su
        for i, (cx, cy, sx, sy, zr) in enumerate(ledges):
            led = su.CuboidCfg(size=(sx * 0.92, sy * 0.92, 0.04), visible=False, collision_props=su.CollisionPropertiesCfg())
            led.func(f"/World/Quake/ledges/l_{i:04d}", led, translation=tuple(W3((cx, cy), zr - 0.02)))
            for (ox, oy) in ((-0.35, -0.35), (0.35, 0.35)):
                if zr > 0.2:
                    self.posts.append(((cx + ox * sx, cy + oy * sy), zr))
        self.log(f"[quake] {len(panels)} slab panels, {len(chunks)} concrete blocks, {len(tops)} wall tops, "
                 f"{len(furn)} furniture pieces; epicentre of damage ({self.epi[0]:.1f}, {self.epi[1]:.1f}) m; rooms hit {self.rooms_hit}")

    def posts_boxes(self, boxes_xy):
        """Visible crushed-debris posts under panel corners (visual only)."""
        if self.posts:
            boxes_xy("debris_posts", [p for p, _ in self.posts], [z for _, z in self.posts], (0.36, 0.33, 0.30))

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
        if not len(self.dust):
            return [], [], []
        d = self.dust                         # positions freeze once the mission runs; the dust only fades
        age = d[:, 6] + t_mission
        alpha = np.clip(0.16 * (1 - age / 20.0), 0, 0.16)
        keep = alpha > 0.02
        pts = [tuple(map(float, p)) for p in d[keep, 0:3]]
        cols = [(0.66, 0.62, 0.56, float(a)) for a in alpha[keep]]
        sizes = [float(s) for s in np.clip(8 + 1.5 * age[keep], 8, 22)]
        return pts, cols, sizes

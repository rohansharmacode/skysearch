"""In-Isaac perception HUD for the single-take stream capture (``run_mission.py --hud overlay|window``).

``HudModel`` (pure python + numpy) turns the mission state + the live world into panel texts and arrays; it is what
tests check and what is written to ``<out>/hud.json`` every update. ``make_hud`` returns a view: ``OmniHud`` draws the
model with omni.ui, either over the viewport (``overlay``: a ``ViewportWindow.get_frame`` layer, so it is inside the
streamed viewport image) or as a window docked right of the viewport (``window``); ``NullHud`` otherwise. Every omni
import is guarded: in --dry-run and tests the HUD is a no-op that still fills the model.

Panels (colours as everywhere: red radar, yellow human audio, teal acoustic priority, purple LiDAR, green dispatch):
  header    LIVE, sim clock, time-lapse factor ("sped up xN"), phase, pile
  lidar     points/s and map coverage (2 m cells of the sectors with at least one LiDAR return)
  listen    per-drone bearing cones of the latest listen + the top clip (class, language, transcript, human score)
  radar     the pod scan's own slow-time displacement (vitals.identify on its IF cube; last 10 s window, replayed at
            the take pace) + range-bin intensity; "analysing breathing... (n s)" until 45 s of dwell, then the
            whole-dwell verdict (early verdicts flip). No live scan: the synthetic library's reference stream for the
            latest ping's condition, labelled "NOT this scan". Red / white on dark, like docs/img/radar_signatures.png
  vitals    the condition label of the latest ping (e.g. "irregular breathing with pauses - check first") + the
            "aid for rescuers, not a diagnosis" note
  decision  the agent's latest dispatch / re-plan line and the dig order
  event     the live stage event (teammate's hotkey, viewport drag, operator zone): what changed, not what the
            mission knows
"""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np

AID_NOTE = "aid for rescuers, not a diagnosis"
VERDICT_AFTER_S = 45.0          # radar verdicts flip early in a dwell: show "analysing" before this (or the final)
SIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "radar_signatures")
_SIG_CACHE = {}


def load_signature(cls):
    """assets/radar_signatures/<class>.json (radar-signature-hud-v1), cached; None if missing."""
    if cls not in _SIG_CACHE:
        try:
            with open(os.path.join(SIG_DIR, f"{cls}.json")) as f:
                _SIG_CACHE[cls] = json.load(f)
        except (OSError, ValueError):
            _SIG_CACHE[cls] = None
    return _SIG_CACHE[cls]
CONDITION_TEXT = {"irregular_apneic": "irregular breathing with pauses - check first",
                  "unconscious_slow": "slow regular breathing (possibly unconscious) - check first",
                  "distressed_tachypnea": "rapid shallow breathing - possible injury / distress",
                  "calm": "calm regular breathing - stable, waiting",
                  "unknown": "breathing pattern unclear - rescan"}
COL = {"red": 0xFF3C3CE8, "radar_bg": 0xE0100C0B, "yellow": 0xFF33CCFF, "teal": 0xFFB4C814, "purple": 0xFFFF6B9B, "green": 0xFF5EC522,
       "white": 0xFFEEEEEE, "grey": 0xFFAAAAAA, "bg": 0xB0141414, "blue": 0xFFF6823B}   # omni.ui is 0xAABBGGRR


def _short(s, n):
    s = " ".join(str(s or "").split())
    return s if len(s) <= n else s[: n - 3] + "..."


def _poly_mask(X, Y, poly):
    P = np.asarray(poly, float)
    xs, ys = P[:, 0], P[:, 1]
    inside = np.zeros(X.shape, bool)
    for k in range(len(P)):
        x0, y0, x1, y1 = xs[k - 1], ys[k - 1], xs[k], ys[k]
        cond = (y1 > Y) != (y0 > Y)
        xi = (x0 - x1) * (Y - y1) / np.where(y0 == y1, 1e-12, y0 - y1) + x1
        inside ^= cond & (X < xi)
    return inside


class HudModel:
    def __init__(self, bounds=None, piles=None, speed=1.0, cell=2.0, sped_up_label=True):
        self.speed, self.sped_up_label = float(speed), sped_up_label
        self.cell = float(cell)
        self.bounds = None
        self.mask = None
        if bounds is not None:
            self._grid(bounds, piles or [])
        self._pts_t, self._pts_n, self.rate = None, 0, 0.0
        self.panels = {}
        self.updates = 0

    def _grid(self, bounds, piles):
        x0, y0, x1, y1 = (float(v) for v in bounds)
        self.bounds = (x0, y0, x1, y1)
        self.nx, self.ny = max(1, int(math.ceil((x1 - x0) / self.cell))), max(1, int(math.ceil((y1 - y0) / self.cell)))
        X, Y = np.meshgrid(x0 + (np.arange(self.nx) + 0.5) * self.cell, y0 + (np.arange(self.ny) + 0.5) * self.cell)
        m = np.zeros(X.shape, bool)
        for p in piles:
            poly = p.get("polygon") if isinstance(p, dict) else None
            if poly and len(poly) >= 3:
                m |= _poly_mask(X, Y, poly)
            elif isinstance(p, dict) and p.get("xy"):
                m |= np.hypot(X - p["xy"][0], Y - p["xy"][1]) < 9.0
        self.mask = m if m.any() else np.ones(X.shape, bool)
        self.hit = np.zeros(X.shape, bool)

    # ------------------------------------------------------------------ LiDAR
    def add_points(self, pts):
        if self.bounds is None or pts is None or len(pts) == 0:
            return
        P = np.asarray(pts, float)
        i = ((P[:, 0] - self.bounds[0]) / self.cell).astype(int)
        j = ((P[:, 1] - self.bounds[1]) / self.cell).astype(int)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        self.hit[j[ok], i[ok]] = True

    def coverage(self):
        if self.mask is None:
            return 0.0
        return float((self.hit & self.mask).sum() / max(1, self.mask.sum()))

    def lidar_rate(self, total, now):
        """Points per VIDEO second (the stream's clock), exponentially smoothed."""
        if self._pts_t is not None and now > self._pts_t:
            r = (total - self._pts_n) / (now - self._pts_t)
            self.rate += (r - self.rate) * min(1.0, (now - self._pts_t) / 1.5)
        self._pts_t, self._pts_n = now, total
        return self.rate

    # ------------------------------------------------------------------ build
    def update(self, state, live=None, lidar_total=0, now=None, speed=None, sim_t=None):
        st = state or {}
        if self.bounds is None and (st.get("site") or {}).get("bounds"):
            self._grid(st["site"]["bounds"], st.get("piles") or [])
        now = time.monotonic() if now is None else now
        sp = self.speed if speed is None else float(speed)
        lj = live.to_json() if live is not None and hasattr(live, "to_json") else (live or {})
        mi = st.get("mission") or {}
        ph = st.get("phase") or "-"
        pile = mi.get("active_pile") or "-"
        P = {}
        lapse = f"time-lapse x{sp:g} (sped up)" if self.sped_up_label and sp > 1.01 else "real time"
        P["header"] = {"text": f"LIVE  t {float(st.get('t') or 0):.0f} s  |  {lapse}  |  {ph}  |  pile {pile}"}
        rate = self.lidar_rate(int(lidar_total or 0), now)
        cov = self.coverage()
        P["lidar"] = {"pts_per_s": round(rate), "total": int(lidar_total or 0), "coverage": round(cov, 3),
                      "text": f"LiDAR  {rate / 1000:.0f}k pts/s  |  {int(lidar_total or 0) / 1e6:.2f} M pts  |  "
                              f"map {100 * cov:.0f}% of the sectors"}
        P["listen"] = self._listen(st, lj)
        P["radar"] = self._radar(st, lj, float(st.get("t") or 0.0) if sim_t is None else float(sim_t))
        P["vitals"] = self._vitals(st)
        P["decision"] = self._decision(st, lj)
        P["event"] = self._event(lj)
        P["source"] = {"text": self._sources(st, lj)}
        self.panels = P
        self.updates += 1
        return P

    def _listen(self, st, lj=None):
        w = ((lj or {}).get("acoustic") or {}).get("waiting")
        if w:
            return {"cones": [], "top": None, "waiting": w,
                    "text": f"Listen  -  real acoustic pipeline analysing the rotors-off listen ({w['s']:.0f} s); "
                            "mission clock held until it answers"}
        clips = st.get("acoustic_clips") or []
        if not clips:
            return {"cones": [], "top": None, "text": "Listen  -  no listen yet (rotors-off listens after each radar phase)"}
        n = max(c.get("listen", 0) for c in clips)
        last = [c for c in clips if c.get("listen") == n]
        cones = []
        for c in last:
            if (c.get("scores") or {}).get("human", 0) < 0.4:
                continue
            for b in c.get("bearings") or [{"drone_id": c.get("drone_id"), "bearing_deg": c.get("bearing_deg"),
                                            "width_deg": c.get("width_deg")}]:
                if b.get("bearing_deg") is None:
                    continue
                cones.append({"drone": b.get("drone_id"), "bearing": round(float(b["bearing_deg"])),
                              "width": round(float(b.get("width_deg") or 0)), "pile": c.get("pile_id")})
        top = max(last, key=lambda c: c.get("priority") or 0)
        lang = top.get("lang") or top.get("language") or (top.get("gemini") or {}).get("language") or "-"
        hs = (top.get("scores") or {}).get("human")
        tx = _short(top.get("transcript"), 70)
        line = (f"Top clip  {top.get('class', '?')}  |  lang {lang}  |  human {hs if hs is not None else '-'}  |  "
                f"priority {top.get('priority')} -> {top.get('pile_id')}")
        cone_txt = "   ".join(f"{c['drone']} {c['bearing']}° ±{c['width'] // 2}° ({c['pile']})" for c in cones[:6]) or "no human-like bearing"
        return {"cones": cones, "top": {"class": top.get("class"), "lang": lang, "human": hs, "transcript": tx,
                                        "pile": top.get("pile_id"), "priority": top.get("priority")},
                "text": f"Listen {n}  cones: {cone_txt}\n{line}" + (f"\n\"{tx}\"" if tx else "")}

    def _radar(self, st, lj, t):
        """The pod scan's own slow-time signal (vitals.identify on the actual IF cube), replayed from the moment
        the dwell's data is in, at the take pace; the verdict only after VERDICT_AFTER_S of dwell (early partial
        verdicts flip, docs/RADAR_SIGNATURES.md). Without a live scan: the library's reference stream for the
        latest ping's condition, labelled as such (pre-viz / fallback)."""
        sc = (lj or {}).get("last_scan")
        if sc and sc.get("sig") and sc["sig"].get("wave_mm"):
            sig = sc["sig"]
            p = min(max(t - float(sc["t"]), 0.0), float(sig["dwell_s"]))
            head = (f"Radar  pod {sc.get('drone_id')} scan {sc.get('scan_id')} at {sc.get('pile')}  |  dwell "
                    f"{sig['dwell_s']:.0f} s at the take pace  |  return at {sig['range_m']:.1f} m  |  quality {sc.get('quality')}")
            return self._play(sig["wave_mm"], sig.get("fps", 10.0), p, sig["dwell_s"], sig["final"], head,
                              sc.get("range_db"), "live")
        ref = self._library(st, t)
        if ref is not None:
            return ref
        if sc:
            hz = sc.get("breathing_hz")
            return {"wave": sc.get("wave"), "range_db": sc.get("range_db"), "verdict": None, "source": "live",
                    "text": f"Radar  {sc.get('drone_id')} scan {sc.get('scan_id')} at {sc.get('pile')}  |  quality "
                            f"{sc.get('quality')}  |  " + (f"{60 * hz:.0f}/min" if hz else "no breathing-like return")}
        scans = st.get("scans") or []
        if not scans:
            return {"wave": None, "range_db": None, "verdict": None, "source": None, "text": "Radar  -  no scan yet"}
        s_ = scans[-1]
        hz = s_.get("breathing_hz")
        return {"wave": None, "range_db": None, "verdict": None, "source": "state",
                "text": f"Radar  {s_.get('drone_id')} scan {s_.get('scan_id')}  |  quality {s_.get('quality')}  |  "
                        + (f"{60 * hz:.0f}/min" if hz else "no breathing-like return")}

    def _play(self, wave, fps, p, dwell, final, head, range_db, source):
        k = int(p * fps)
        win = wave[max(0, k - int(10 * fps)):k + 1]
        gate = min(VERDICT_AFTER_S, float(dwell))
        if p < gate - 1e-6:
            verdict, line = None, f"analysing breathing... ({p:.0f} s)"
        else:
            verdict = final
            if final.get("source") == "human_breathing":
                lab = CONDITION_TEXT.get(final.get("condition")) or final.get("text")
                rate = f", {final['rate_bpm']:.0f}/min" if final.get("rate_bpm") else ""
                line = f"{lab}{rate} (confidence {final.get('conf')})  |  {AID_NOTE}"
            else:
                line = f"{final.get('source')}: {_short(final.get('text'), 90)} (not dug for)"
        return {"wave": win, "range_db": range_db, "verdict": verdict, "source": source, "elapsed_s": round(p, 1),
                "text": f"{head}\n{line}"}

    def _library(self, st, t):
        pings = [c for c in (st.get("pings") or []) if (c.get("vitals") or {}).get("condition") not in (None, "unknown")]
        if not pings:
            return None
        c = max(pings, key=lambda c: c.get("t_found") or 0)
        cond = c["vitals"]["condition"]
        ref = load_signature(cond)
        if ref is None:
            return None
        n = len(ref["wave_mm"])
        p = min(max(t - float(c.get("t_found") or 0.0), 0.0), n / ref["fps"])
        out = self._play(ref["wave_mm"], ref["fps"], p, n / ref["fps"], {**ref["final"], "rate_bpm": None},
                         f"Radar  reference signature '{cond}' (synthetic library, NOT this scan) for {c['id']}",
                         None, "library")
        return out

    def _vitals(self, st):
        pings = [c for c in (st.get("pings") or []) if c.get("vitals")]
        if not pings:
            return {"label": None, "text": f"Vitals  -  no ping with a breathing pattern yet  ({AID_NOTE})", "note": AID_NOTE}
        c = max(pings, key=lambda c: c.get("t_found") or 0)
        v = c["vitals"]
        cond = v.get("condition") or {"regular_calm": "calm", "rapid_shallow": "distressed_tachypnea",
                                      "slow_regular": "unconscious_slow",
                                      "irregular_with_pauses": "irregular_apneic"}.get(v.get("pattern"), "unknown")
        label = CONDITION_TEXT.get(cond) or v.get("condition_text") or cond
        return {"label": label, "ping": c["id"], "rate_bpm": v.get("rate_bpm"), "hint": v.get("triage_hint"),
                "note": AID_NOTE,
                "text": f"Vitals {c['id']}: {label}  |  {v.get('rate_bpm')}/min, hint {v.get('triage_hint')}  "
                        f"(confidence {v.get('pattern_conf')})\n{AID_NOTE}; candidate region, not a confirmed person"}

    def _decision(self, st, lj):
        dis = st.get("dispatch") or []
        rp = (lj or {}).get("replans") or []
        line = "Agent  -  searching"
        if dis:
            d = dis[-1]
            if d.get("to_pile"):
                line = f"Agent  dispatching d1-d3 to {d['to_pile']}: {_short(d.get('reason'), 120)}"
            else:
                line = f"Agent  stop: {_short(d.get('reason'), 120)}"
        if rp and (not dis or rp[-1]["t"] >= dis[-1].get("t", 0)):
            line = f"Agent  RE-PLAN t {rp[-1]['t']:.0f}: {_short(rp[-1]['why'], 120)}"
        pings = sorted((c for c in (st.get("pings") or []) if c.get("dig_score") is not None),
                       key=lambda c: -c["dig_score"])
        dig = "  ".join(f"{k + 1}. {c['id']} ({c['dig_score']:.2f})" for k, c in enumerate(pings[:4])) or "none yet"
        return {"line": line, "dig_order": [c["id"] for c in pings], "text": f"{line}\nDig order: {dig}"}

    def _event(self, lj):
        ev = (lj or {}).get("last_event")
        if not ev:
            return {"text": "", "event": None}
        return {"event": ev, "text": f"STAGE EVENT t {ev['t']:.0f}: dummy {ev['id']} {ev['what']} near {ev['sector']} "
                                     f"({ev['cause']}) - the drones must hear / scan it to know"}

    @staticmethod
    def _sources(st, lj):
        mi = st.get("mission") or {}
        src = ((lj or {}).get("last_listen") or {}).get("source") or mi.get("acoustic_source") or "-"
        synth = "SYNTHETIC generator (not for the take)" if str(src).startswith("stub") else "acoustic pipeline"
        return (f"Sources: radar = synthetic IF + real DSP; acoustic = {synth} ({src}); victims from the Isaac stage. "
                "Pings are candidate regions.")

    def to_json(self):
        return {"schema": "larpverse.hud/v1", "updates": self.updates, "panels": self.panels}


# =============================================================================== views
class NullHud:
    ok = False
    kind = "off"

    def __init__(self, *a, **k):
        self.draws = 0

    def draw(self, panels):
        self.draws += 1

    def close(self):
        pass


class OmniHud:
    """omni.ui view of the model. ``overlay``: a frame over the active viewport (inside the streamed image);
    ``window``: a window docked right of the viewport. Falls back overlay -> window -> no-op."""

    WAVE_N, RANGE_N = 160, 120

    def __init__(self, kind="overlay", log=None, on_button=None):
        self.ok, self.kind, self.draws = False, kind, 0
        say = log or (lambda m: print(f"[hud] {m}", flush=True))
        import omni.ui as ui                   # ImportError on the dry run -> make_hud returns NullHud
        self.ui, self.lbl, self.plots = ui, {}, {}
        self.win = None
        frame = None
        if kind == "overlay":
            try:
                from omni.kit.viewport.utility import get_active_viewport_window
                vw = get_active_viewport_window()
                frame = vw.get_frame("larpverse_hud") if vw is not None else None
            except Exception as e:             # noqa: BLE001
                say(f"viewport overlay unavailable ({type(e).__name__}: {e}); using a docked window")
                frame = None
            if frame is None:
                kind = self.kind = "window"
        if kind == "window":
            self.win = ui.Window("Larpverse perception", width=480, height=980)
            frame = self.win.frame
        with frame:
            if self.kind == "overlay":
                with ui.HStack():
                    ui.Spacer()
                    with ui.ZStack(width=ui.Pixel(470)):
                        ui.Rectangle(style={"background_color": COL["bg"], "border_radius": 6})
                        self._panels(ui, on_button)
            else:
                with ui.ZStack():
                    ui.Rectangle(style={"background_color": COL["bg"]})
                    self._panels(ui, on_button)
        if self.win is not None:
            try:
                vp = ui.Workspace.get_window("Viewport")
                if vp is not None:
                    self.win.dock_in(vp, ui.DockPosition.RIGHT, 0.3)
            except Exception as e:             # noqa: BLE001
                say(f"window not docked ({type(e).__name__}: {e}); drag it next to the viewport")
        self.ok = True
        say(f"perception HUD up ({self.kind})")

    def _panels(self, ui, on_button):
        def label(key, color, size=15, wrap=True):
            self.lbl[key] = ui.Label("", word_wrap=wrap, height=0, style={"color": COL[color], "font_size": size})

        with ui.VStack(spacing=5, height=0):
            label("header", "white", 17)
            label("event", "blue", 15)
            label("lidar", "purple")
            label("listen", "yellow")
            label("radar", "white")             # radar: red / white on dark (docs/img/radar_signatures.png)
            ui.Label("chest displacement, last 10 s (mm, slow time)", height=0, style={"color": COL["red"], "font_size": 12})
            self.plots["wave"] = ui.Plot(ui.Type.LINE, -1.0, 1.0, *([0.0] * self.WAVE_N), height=64,
                                         style={"color": COL["white"], "background_color": COL["radar_bg"]})
            ui.Label("range-bin intensity (dB)", height=0, style={"color": COL["red"], "font_size": 12})
            self.plots["range"] = ui.Plot(ui.Type.HISTOGRAM, 0.0, 1.0, *([0.0] * self.RANGE_N), height=48,
                                          style={"color": COL["red"], "background_color": COL["radar_bg"]})
            label("vitals", "teal", 16)
            label("decision", "green", 15)
            label("source", "grey", 12)
            if on_button is not None:
                ui.Button("Unscripted: switch on a hidden dummy", height=28, clicked_fn=on_button)

    @staticmethod
    def _fit(vals, n):
        a = np.asarray([v for v in (vals or []) if v is not None], float)
        if a.size == 0:
            return [0.0] * n, (0.0, 1.0)
        a = np.interp(np.linspace(0, a.size - 1, n), np.arange(a.size), a)
        lo, hi = float(a.min()), float(a.max())
        if hi - lo < 1e-9:
            hi = lo + 1.0
        return a.tolist(), (lo, hi)

    def draw(self, panels):
        for k, lab in self.lbl.items():
            lab.text = (panels.get(k) or {}).get("text", "")
        rd = panels.get("radar") or {}
        for key, src, n in (("wave", rd.get("wave"), self.WAVE_N), ("range", rd.get("range_db"), self.RANGE_N)):
            vals, (lo, hi) = self._fit(src, n)
            p = self.plots[key]
            if key == "wave":
                m = max(abs(lo), abs(hi))
                p.scale_min, p.scale_max = -m, m
            else:
                p.scale_min, p.scale_max = lo, hi
            p.set_data(*vals)
        self.draws += 1

    def close(self):
        try:
            if self.win is not None:
                self.win.visible = False
        except Exception:                      # noqa: BLE001
            pass


def make_hud(kind="off", log=None, on_button=None):
    """``off`` | ``overlay`` | ``window`` | ``auto`` (= overlay). Never raises: NullHud without omni.ui."""
    if kind in (None, "off", "none"):
        return NullHud()
    try:
        return OmniHud("overlay" if kind == "auto" else kind, log=log, on_button=on_button)
    except Exception as e:                     # noqa: BLE001  (dry run / API drift: the take falls back to split-screen)
        if log:
            log(f"HUD unavailable ({type(e).__name__}: {e}); no-op (use the split-screen dashboard)")
        return NullHud()


class Hud:
    """Model + view + hud.json writer, throttled to ``hz`` (video clock)."""

    def __init__(self, kind="off", out_dir=None, bounds=None, piles=None, speed=1.0, hz=4.0, log=None, on_button=None):
        self.model = HudModel(bounds, piles, speed)
        self.view = make_hud(kind, log, on_button)
        self.out, self.period, self._last = out_dir, 1.0 / max(hz, 0.1), -1e9

    def update(self, state, live=None, lidar_total=0, now=0.0, speed=None, force=False, sim_t=None):
        if not force and now - self._last < self.period:
            return None
        self._last = now
        panels = self.model.update(state, live, lidar_total, now, speed, sim_t=sim_t)
        self.view.draw(panels)
        if self.out:
            path = os.path.join(self.out, "hud.json")
            try:
                with open(path + ".tmp", "w") as f:
                    json.dump(self.model.to_json(), f, separators=(",", ":"), ensure_ascii=False)
                os.replace(path + ".tmp", path)
            except OSError:
                pass
        return panels

    def add_points(self, pts):
        self.model.add_points(pts)

    def close(self):
        self.view.close()

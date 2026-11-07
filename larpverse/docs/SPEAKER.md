# Loudspeaker: call-out and listen, reassurance, route guidance (contract v1.3)

The heavy-lift drones carry a small PA loudspeaker. They use it three ways:

1. **Call-out and listen.** This is the standard USAR practice: get silence, call out, listen.
2. **Reassurance.** After a ping, or after someone responds, the drone tells them help is coming.
3. **Route guidance.** People who can walk are told how to get to the rescue tent.

Every spoken string is a fixed template. An LLM never writes text that is played without an operator approving it.

| Piece | File | Status |
|---|---|---|
| Protocol, response generator (truth side), analyser (measurements only), triage fusion, mission facade | `sim/callout.py` | committed |
| Templates (5 languages), A* route guidance, broadcast log, operator translation drafts, operator queue | `sim/guidance.py` | committed |
| Audio rendering: Piper, espeak-ng, or opt-in Gemini TTS | `acoustic/speaker_assets.py` | committed |
| Tests (17, about 4 s) | `tests/test_callout.py` | committed |
| Mission integration | patch below, for `sim/grid_mission.py` | **orchestrator / mission builder applies it** |
| Operator API | patch below, for `scripts/serve.py` | **orchestrator applies it** |
| Contract section | text below, to append to `contracts/INTERFACES.md` | **orchestrator appends it**. The file has another agent's uncommitted edit (assoc-v8), so per RULES rule 3 it was not edited here. |

## 1. Protocol (per listen phase, drones landed)

| Step | What happens | Time |
|---|---|---|
| (a) silence | Rotors have been off since landing. The existing passive listen (26 s) is used as the quiet baseline: the background rate of human-like sounds per sector. Ground crews are asked to stop work over the radio. This is logged, not broadcast. | the last 24 s of the passive listen |
| (b) message | The drone nearest the target (a ping, or else the pile centre) plays the call-out in zh-TW, nan-TW, en, id and vi, with a 2.5 s listen gap after each language. | about 30 s (from the rendered WAV durations, or an estimate) |
| (c) listen | All drones listen for 20 s. | 20 s |
| reassure | "Help is coming ..." in 5 languages, aimed at pings and responding sectors in earshot. The same target is reassured at most once every 10 min. | about 30 s |
| guidance | A spoken route is played to each new ambulatory track. | about 60 s |

The drones stay landed until the follow-up broadcasts end, because the listener has to be close enough to hear them. Results become visible in `state` when the listen window ends.

**Analysis** (`analyse_callout`) uses detections and public pile centres only:

1. Group detections of the same event across drones. Grouping is bearing-consistent, so two sources that overlap in time stay separate.
2. Assign each group to a sector. The bearing error, after allowing for the pile's angular extent, must be ≤ 12°, and the next-best sector must be ≥ 4° worse.
3. Count the human-like events after the first message. Compare the count with the sector's baseline rate (Poisson tail). A pattern the sector was not making before the call-out is compared against a floor rate of 0.01/s instead. Two examples: the instructed knock ×3 after spontaneous fast knocking, or a voice after silence.
4. Add time-lock, multi-drone and knock-×3 bonuses.
5. Decide the result:

| Result | Condition | Text |
|---|---|---|
| `responded` (AVPU "V") | confidence ≥ 0.6 and (≥ 2 multi-drone events, or a multi-drone knock ×3) | as reported |
| `possible_response` | weaker evidence | as reported |
| `nonhuman_only` | only dog or debris sounds | as reported |
| `no_response` | nothing | always says "does not rule out an unconscious person" |

A human-like event heard by ≥ 2 drones becomes an **open-ground track** under three conditions:

- it triangulates with σ ≤ 3 m (bearings-only Fisher information);
- it lies more than 9 m + 2σ from every pile centre;
- it repeats (≥ 2 calls).

**Triage fusion** (`fuse_triage`) combines the condition from `sim/vitals.py` with the call-out result. The `triage_hint` values and their ranks are imported from `sim/vitals.py`, not copied. The rule follows START: someone who cannot follow a command is IMMEDIATE.

| `vitals.condition` | No response to the call-out | Responded (AVPU V) |
|---|---|---|
| unconscious_slow (hint `urgent`) | **immediate**: reduced consciousness is more likely than deep rest. This is exactly the check the radar asks for. | stays `urgent`, re-check |
| irregular_apneic | **immediate** | at least `immediate (check)` |
| distressed_tachypnea | **immediate** | hint kept (rate-based) |
| calm | `immediate (check)`: they may simply not hear | `delayed` |
| unknown | unchanged | unchanged |

Three rules hold in every case:

- No response never lowers a hint.
- A response never lowers an abnormal condition, because the call-out works at sector level, not per person.
- If the message may not have reached the sector, nothing changes.

The output is `{triage_hint, condition, avpu, reason, changed_from, rank}`.

**Association boost** (`association_boost`). A time-locked response is a timing reference. It multiplies the odds on a clip's reliability by a likelihood ratio of 1 + 3c, where c is the response confidence. In the mission patch this re-scores that listen's clips for the responding pile. `response_priority` turns a response into a priority on the acoustic scale, using the same formula.

## 2. Response model (hidden truth, generator only, labelled assumptions)

Shared assumptions:

- Speaker 110 dB SPL at 1 m. A person understands the message at ≥ 55 dB.
- Rubble loss 3 dB/m (as in `acoustic/scene.py`), plus 0–8 dB of path variability.
- Ambient noise 45 dB with rotors off. Detection is a logistic around 6 dB SNR.
- Bearing noise σ = 3° + 30°/SNR.
- Response latency is counted from the end of the first segment in a language the person understands. Language priors for Tainan: zh 0.45, nan 0.35, id 0.08, vi 0.08, en 0.04.
- People repeat a response every 3–6 s until they stop, so a bout masked by the loudspeaker is usually followed by another.

| Hidden state | P(respond \| heard) | Response (level at 1 m) | Latency | P(stop) per bout |
|---|---|---|---|---|
| calm | 0.90 | knock ×3 (88 dB) or shout (90 dB) | 1–4 s | 0.10 |
| distressed_tachypnea | 0.60 | fast irregular knocking ×4–7 (84 dB) or weak voice (72 dB) | 2–8 s | 0.35 |
| unconscious_slow | 0 | none, and no spontaneous sounds | – | – |
| irregular_apneic | 0 | none, and no spontaneous sounds | – | – |
| ambulatory (synthetic bystander) | 0.90 | shout | 1–3 s | 0.20 |

Distractors:

- Dogs bark at the loudspeaker (p 0.7) and also bark on their own.
- Debris settles at random.
- The classifier makes some confusions: knock → debris 12%, debris → knock 15%, crying → voice 30%, dog → voice 5%.
- Bystanders (`walking_wounded`) are people who walked out of the reported collapse: one on open ground 13–20 m from the start pile, plus a second elsewhere 40% of the time.

**Analyser outcome by hidden state** (evaluator view; 30 seeds; condition-only scenario (4da97d0); drones ringed 10 m around each occupied pile):

| Hidden state | responded | possible | no response | non-human only |
|---|---|---|---|---|
| calm (35) | 31 | 2 | 2 | 0 |
| distressed_tachypnea (27) | 9 | 7 | 11 | 0 |
| irregular_apneic (30) | **0** | 0 | 30 | 0 |
| unconscious_slow (28) | **0** | 2 | 26 | 0 |
| empty pile (150) | **0** | 10 | 98 | 42 |

Open-ground tracks over 270 listens:

- 31 true tracks, median position error 1.0 m;
- **0 false tracks**;
- recall 26% for bystanders within 22 m. This is conservative by design.

**Caveat.** All of this is synthetic, an event-level model, not rendered audio. In this scenario a victim's physio state and their sound role are drawn independently. The generator therefore suppresses spontaneous sounds from unconscious and apnoeic people, but the passive acoustic scene still plays them. See Requests.

## 3. Messages (fixed; non-English pending native-speaker review)

The guidance column shows seed 7's actual broadcast to bystander track 2-T1 at (−31, −12), heading 271.6° (contract convention: counter-clockwise from east, so this is south).

| Lang | Call-out | Reassure | Guidance |
|---|---|---|---|
| zh-TW | 這裡是救援隊。如果你聽得到，請敲三下或大聲喊。 | 救援人員正在趕來。請保持不動，保存體力，聽得到請輕敲。 | 請遠離瓦礫堆。往南走大約15公尺，再往東南走大約40公尺，到救援帳篷。如果受傷，請留在原地揮手。 |
| nan-TW | 阮是救難隊。你若聽會著，請敲三下抑是大聲喝。 | 救援的人當咧來矣。請莫振動，保留氣力，聽會著就敲一下。 | 請離瓦礫堆較遠咧。向南爿行差不多15公尺，閣向東南爿行差不多40公尺，到救援的帳篷。若有著傷，請留佇原所在擛手。 |
| en | This is a rescue team. If you can hear me, knock three times or shout. | Help is coming. Stay still, save your energy, tap if you can hear me. | Keep clear of the rubble. Walk south about 15 metres, then south-east about 40 metres, to the rescue tent. If you are hurt, stay where you are and wave. |
| id | Ini tim penyelamat. Jika Anda mendengar saya, ketuk tiga kali atau berteriak. | Bantuan sedang datang. Tetap diam, hemat tenaga Anda, ketuk jika Anda mendengar saya. | Menjauhlah dari reruntuhan. Berjalanlah ke arah selatan sekitar 15 meter, lalu ke arah tenggara sekitar 40 meter, menuju tenda penyelamat. Jika Anda terluka, tetap di tempat dan lambaikan tangan. |
| vi | Đây là đội cứu hộ. Nếu bạn nghe thấy tôi, hãy gõ ba lần hoặc hét lên. | Cứu hộ đang đến. Hãy nằm yên, giữ sức, gõ nhẹ nếu bạn nghe thấy tôi. | Hãy tránh xa đống đổ nát. Đi về hướng nam khoảng 15 mét, rồi về hướng đông nam khoảng 40 mét, đến lều cứu hộ. Nếu bị thương, hãy ở yên tại chỗ và vẫy tay. |

The Taiwanese lines also have a Tâi-lô romanisation, `NAN_TAILO`, for the native speaker who records them. Guidance slots take only:

- an 8-point compass word from a fixed table;
- a distance rounded to 5 m (under 20 m) or to 10 m;
- the destination "rescue tent".

**Audio** (`.venv-audio/bin/python -m acoustic.speaker_assets [--state state.json] [--engine gemini]`), written to `acoustic/data/speaker/` (gitignored):

| Language | Engine | Result |
|---|---|---|
| en | Piper lessac | 4.5 s |
| zh-TW | Piper zh_CN huayan | Mandarin pronunciation; low quality, tones partly lost |
| id | Piper news_tts | rendered |
| vi | Piper vivos | rendered |
| nan-TW | none | No open voice exists. Gemini TTS (`gemini-2.5-flash-preview-tts`, REST, opt-in) rendered the reassurance line, marked **unvetted** because Hokkien is not officially supported. The call-out line returned no audio. Record a native speaker. |

Each broadcast carries `audio_url` and `audio_status`.

**Operator messages.** The operator types text, and it is queued under their name. Gemini translations (optional, `agents.llm`) are checked by `validate_translation`:

- numbers must match the source;
- the length ratio must be in bounds;
- banned claims ("confirmed", "survivor found" ...) are rejected.

Translations that pass are stored as `pending_approval`. `BroadcastLog.add` raises if LLM text is marked scheduled without `approved_by`. `Speaker.ingest_operator` plays only released items.

## 4. Contract text to append to `contracts/INTERFACES.md` (additive, next to the other v1.3 sections)

```markdown
## Speaker (v1.3, additive): loudspeaker broadcasts, call-out responsiveness, operator API

Heavy-lift drones carry a loudspeaker. Every played string is a vetted template (sim/guidance.py) or text an
operator typed; machine translations are `pending_approval` until an operator approves them. Never "confirmed".

state.json additions (sim/callout.Speaker.state; written by sim/grid_mission.py):

"broadcasts": [ { "id": "b23", "t": 600.1, "drone_id": "d3",
    "kind": "callout|reassure|guidance|operator", "lang": "zh-TW|nan-TW|en|id|vi",
    "text": "...", "text_en": "...", "audio_url": "acoustic/data/speaker/<key>.<lang>.wav" | null,
    "audio_status": "rendered|unvetted (...)|missing: record a native speaker|no audio yet ...",
    "target": { "type": "sector|ping|sectors|track|zone|drone", "id": "A3-R1", "xy": [x, y]? },
    "heading_deg": 271.6,            // guidance only: first leg, CCW from east (contract convention)
    "source": "template|operator|llm_translation", "template": "callout|reassure|guidance" | null,
    "status": "played|pending_approval|rejected", "approved_by": "op1" | null,
    "duration_s": 4.7, "review": "pending native-speaker review" | null, "group": "callout-2" } ],
    // scheduled items appear as "played" once t <= state.t; pending / rejected operator items are listed for the UI
"callouts": [ { "id": "C2", "t_start": 466.0, "listen_ends": 523.0, "t_end": 644.5, "speaker_drone": "d2",
    "target": { "type": "ping", "id": "A3-R1", "xy": [x, y] }, "langs": [...],
    "steps": [ { "step": "silence|message|listen", "lang"?, "t0", "t1" } ],
    "sectors": { "A3": <responsiveness> }, "pings": { "A3-R1": <responsiveness + ping_id, note> },
    "tracks": [ { "id": "T1", "xy": [x, y], "source": "callout_voice_open_ground", "evidence": [t...], "why": "..." } ],
    "guided": [ { "track": "2-T1", "xy", "source", "why", "text_en", "path": [[x, y]...], "legs": [{bearing_deg, dist_m, compass}] } ],
    "reassured": ["A3-R1"], "n_events": 41, "synthetic": true } ],   // results only after listen_ends; before: status "in_progress"
"responsiveness": { "A3": <responsiveness> },   // per pile: the call-out heard from closest (then latest)
<responsiveness> = { "status": "responded|possible_response|nonhuman_only|no_response", "kind": "knock_x3|knock_fast|knock|voice|shout|crying|..." | null,
    "avpu": "V|unknown|not V (A/V not shown)", "confidence": 0-1, "text": "...", "callout_t": 466.0,
    "range_m": 11.4, "message_reached": true, "events": [...]? , "lang_hint": "zh-TW"? }
piles[].responsiveness, radar_candidates[].responsiveness (same object); radar_candidates[].triage_fused
    { triage_hint, condition, avpu, reason, changed_from, rank } when the candidate has vitals
    (hints / ranks from sim/vitals.TRIAGE_RANK; no response raises, never lowers).
drones[].speaker = { kind, lang, id } while that drone is playing.

Rules: "no_response" never clears a pile and its text says it does not rule out an unconscious person; a
"responded" sector is a candidate, not a confirmed person; guidance routes use the LiDAR heightmap (rubble
> 0.4 m above local ground, +2 m margin) and tasked zones, never truth.

Operator API (scripts/serve.py, queue file dashboard/data/broadcast_queue.json, read by the mission each step):
GET  /api/broadcast                 -> { schema: "larpverse.broadcast_queue/v1", revision, items: [broadcast...] }
POST /api/broadcast                 { "drone_id": "d2" | "zone": "Z1", "kind": "operator|callout|reassure",
                                      "text": "...", "src_lang": "en", "langs": [...], "translate": true,
                                      "operator": "name" }
     -> { items, translation_problems }; operator text: scheduled; translations: pending_approval;
        callout / reassure: the vetted templates (text ignored)
POST /api/broadcast/approve         { "id": "op203011-2", "operator": "name", "approve": true|false }
```

## 5. Operator API patch for `scripts/serve.py` (orchestrator applies)

This patch was built and compiled on a scratch copy. Its handlers were exercised without a socket, since no second server may be started: POST returned 200, a request without an operator name returned 400, and a GET returned the queue. The handler imports the stdlib-only parts of `sim.guidance`; numpy is optional there, so the system `python3` runs it.

```diff
--- a/scripts/serve.py
+++ b/scripts/serve.py
@@ -5,17 +5,25 @@
 GET  /...               static files from the repo root (no-cache, so polling always sees fresh state)
 GET  /api/tasking       current operator tasking (dashboard/data/tasking.json) or {"zones": []}
 POST /api/tasking       replace the tasking with a JSON body {"zones": [...], ...}; validated, written atomically
+GET  /api/broadcast     operator loudspeaker queue (dashboard/data/broadcast_queue.json; contract v1.3)
+POST /api/broadcast     {drone_id | zone, kind: operator|callout|reassure, text, src_lang, langs, translate, operator}
+                        -> operator text queued; machine translations (Gemini, optional) stay pending_approval
+POST /api/broadcast/approve  {id, operator, approve: true|false} -> release or reject one pending item
 Stdlib only. Binds to 127.0.0.1 by default (LAN demo: --host 0.0.0.0 at your own risk).
 """
 import argparse
 import json
 import os
+import sys
 import time
 from http import HTTPStatus
 from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
 
 ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
 TASKING = os.path.join(ROOT, "dashboard", "data", "tasking.json")
+BROADCAST_Q = os.path.join(ROOT, "dashboard", "data", "broadcast_queue.json")
+if ROOT not in sys.path:                     # sim.guidance (stdlib-only parts) for the broadcast API
+    sys.path.insert(0, ROOT)
 MAX_BODY = 512 * 1024
 MAX_ZONES, MAX_VERTS = 64, 400
 
@@ -62,6 +70,9 @@
         self.wfile.write(body)
 
     def do_GET(self):
+        if self.path.split("?")[0] == "/api/broadcast":
+            from sim.guidance import queue_load          # stdlib-only part of sim.guidance (numpy optional)
+            return self._json(HTTPStatus.OK, queue_load(BROADCAST_Q))
         if self.path.split("?")[0] == "/api/tasking":
             try:
                 with open(TASKING) as f:
@@ -71,7 +82,10 @@
         return super().do_GET()
 
     def do_POST(self):
-        if self.path.split("?")[0] != "/api/tasking":
+        route = self.path.split("?")[0]
+        if route in ("/api/broadcast", "/api/broadcast/approve"):
+            return self._broadcast(route)
+        if route != "/api/tasking":
             return self._json(HTTPStatus.NOT_FOUND, {"error": "unknown endpoint"})
         n = int(self.headers.get("Content-Length") or 0)
         if n <= 0 or n > MAX_BODY:
@@ -87,6 +101,24 @@
         os.replace(tmp, TASKING)
         return self._json(HTTPStatus.OK, doc)
 
+    def _broadcast(self, route):
+        """Operator loudspeaker API (contract v1.3). Nothing machine-translated is ever playable before an
+        operator approves it; the mission (sim/callout.Speaker.ingest_operator) plays only released items."""
+        from sim.guidance import BroadcastError, queue_decide, queue_post
+        n = int(self.headers.get("Content-Length") or 0)
+        if n <= 0 or n > 16 * 1024:
+            return self._json(HTTPStatus.BAD_REQUEST, {"error": "empty or too large body"})
+        try:
+            body = json.loads(self.rfile.read(n))
+            if route == "/api/broadcast":
+                out = queue_post(BROADCAST_Q, body)        # may call Gemini (agents.llm: .env key, cache, backoff)
+            else:
+                out = queue_decide(BROADCAST_Q, str(body.get("id") or ""), str(body.get("operator") or "")[:40],
+                                   approve=bool(body.get("approve", True)))
+        except (BroadcastError, ValueError, json.JSONDecodeError) as e:
+            return self._json(HTTPStatus.BAD_REQUEST, {"error": str(e)})
+        return self._json(HTTPStatus.OK, out)
+
     def log_message(self, fmt, *args):  # quiet: only API calls and errors
         if "/api/" in (self.path or "") or (args and str(args[1]).startswith(("4", "5"))):
             super().log_message(fmt, *args)
@@ -97,5 +129,5 @@
     ap.add_argument("--host", default="127.0.0.1")
     ap.add_argument("--port", type=int, default=8765)
     a = ap.parse_args()
-    print(f"Larpverse ops server: http://{a.host}:{a.port}/dashboard/  (tasking API: /api/tasking)")
+    print(f"Larpverse ops server: http://{a.host}:{a.port}/dashboard/  (APIs: /api/tasking, /api/broadcast)")
     ThreadingHTTPServer((a.host, a.port), Handler).serve_forever()
```

## 6. Mission patch for `sim/grid_mission.py` (mission builder applies)

This patch is a 3-way merge onto the working tree as it stood at the last check, including the uncommitted `launch_pile` / `site` / `platform` / `delivery` / vitals edits. Re-merge it if the file has moved since. `patch --dry-run` is clean. It adds:

- `speaker=True` / `--no-speaker` and `callout_fn=`;
- `_begin_callout`, which plans the call-out when the listen starts and adds the landed seconds to `listen_end`;
- `_apply_callout`, which runs before the dispatch gate. It sets pile and ping responsiveness, boosts clips, raises a responding pile's priority (but never changes a ping), and attaches `triage_fused` when a candidate has `vitals`;
- operator queue ingest each step;
- `drones[].speaker`;
- the v1.3 state fields.

The mission still never reads truth: the scenario goes only to the call-out generator, as it already does for the acoustic pipeline.

```diff
--- a/sim/grid_mission.py
+++ b/sim/grid_mission.py
@@ -19,6 +19,9 @@
                ``.sample_pointcloud(n)``, ``.pile_xy``) or feed LiDAR with ``ingest_pointcloud``
   * sensors  : pass ``sensor_factory=`` (pile_world, seed) -> object with ``radar_scan(...)``
   * acoustic : pass ``acoustic_fn=`` (scenario, drone_positions, exclude_piles, seed) -> acoustic/v1 dict
+  * speaker  : ``speaker=True`` (default): loudspeaker call-out after each passive listen, reassurance after
+               a ping / response, route guidance to people on open ground (sim/callout.py, contract v1.3);
+               pass ``callout_fn=`` (scenario, plan, seed) -> {"detections": [...]} to replace the generator
 """
 from __future__ import annotations
 
@@ -95,7 +98,7 @@
                  slam=False, slam_scans=SLAM_SCANS, slam_kw=None, slam_cache=SLAM_CACHE,
                  autonomous=True, mission_scans=None, dispatch_policy="acoustic", reserve=RESERVE, swap_s=SWAP_S,
                  listen_s=None, revisit_max=2, penetration=None, launch_pile=None, site="grid", platform=None,
-                 delivery=None):
+                 delivery=None, speaker=True, callout_fn=None):
         """``autonomous`` (default): keep searching until the mission scan budget (``mission_scans``, default
         24), the time limit or the open leads run out; battery swaps at the pads; open leads without a firm
         ping are revisited once. ``pile_budget`` caps pile visits (None = no cap; legacy mode uses 3).
@@ -157,6 +160,12 @@
         self.slam_metrics = {}         # pile -> metrics line (ATE, map error) for the log / state
         self._terrain = self.grid.z.copy()   # sim-diagnostic reference for the SLAM map-error line only
         self.decisions = []
+        # loudspeaker (contract v1.3): call-out + listen per listen phase, reassurance, guidance; never reads truth
+        # itself: the scenario goes only to the call-out response generator, as with the acoustic pipeline
+        self.speaker, self.callout_fn = None, callout_fn
+        if speaker and self.listen_s > 0:
+            from .callout import Speaker
+            self.speaker = Speaker(self.world.pile_xy, self.grid, BASE_XY, seed=seed)
         # --- mission state (never touches truth)
         self.t, self.k = 0.0, 0
         self.stage = "pile"            # pile | listen | return | done
@@ -385,6 +394,10 @@
 
     # ------------------------------------------------------------------ main loop
     def step(self):
+        if self.speaker is not None and self.out_dir:     # operator broadcasts approved on the dashboard (v1.3)
+            n = self.speaker.ingest_operator(os.path.join(self.out_dir, "broadcast_queue.json"), self.t, self._drone_xy())
+            if n:
+                self._say(f"Operator broadcast: {n} approved message(s) queued on the loudspeaker")
         if self.stage == "pile":
             self.pm.step()
             self.t, self.k = self.pm.t, self.k + 1
@@ -526,6 +539,8 @@
             self.listen_end = self.t + self.listen_s
             self._say(f"Radar phase at {name} over; landed drones listen to the other piles for {self.listen_s:.0f} s "
                       "and stream clips to the acoustic agent")
+            if self.speaker is not None:
+                self.listen_end += self._begin_callout(name, firm, pm)
         else:
             self._gate(None)
 
@@ -668,8 +683,81 @@
                     st = "no_signal_not_cleared"
                     why = f"Heard a non-human-like sound once (needs 2 listens to dismiss). {why}"
             p.update(priority=new, status=st, why=why)
+        self._apply_callout()
         self._gate(res)
 
+    # ------------------------------------------------------------------ loudspeaker (v1.3)
+    def _begin_callout(self, name, firm, pm):
+        """Plan the call-out after the passive listen. Returns the extra landed seconds: message + listen window
+        (rotors off) + the reassurance / guidance broadcasts that follow (played while still landed nearby)."""
+        t_call = self.t + self.listen_s
+        if firm:
+            target = {"type": "ping", "id": firm[0]["id"], "xy": firm[0]["xy"]}
+        else:
+            target = {"type": "sector", "id": name, "xy": self.world.pile_xy[name]}
+        pings = [{"id": c["id"], "xy": c["xy"], "pile_id": c["pile_id"]} for c in self.done_cands
+                 if c["status"] == "candidate"]
+        moving = [{"id": f"{name}-{c['id']}", "xy": c["xy"]} for c in pm.cands
+                  if c.get("kind") == "moving" and c.get("status") == "candidate"]
+        extra, rec = self.speaker.begin_listen(self.scenario, t_call, self._drone_xy(), target, pings, moving,
+                                               callout_fn=self.callout_fn)
+        self._say(f"Call-out {rec['id']} planned: after the passive listen {rec['speaker_drone']} plays the rescue "
+                  f"call-out ({', '.join(rec['langs'])}) towards {target['id']}, then all drones listen "
+                  f"{rec['listen_ends'] - rec['steps'][-1]['t0']:.0f} s for knocks or voices "
+                  f"(+{extra:.0f} s landed incl. reassurance / guidance)")
+        return extra
+
+    def _apply_callout(self):
+        """Fold the last call-out into pile / ping state: responsiveness, priority for responding piles,
+        call-and-response boost of this listen's clips, and log lines. Measurements only (no truth)."""
+        from .acoustic_grid import clip_priority
+        from .callout import association_boost, fuse_triage, response_priority
+        if self.speaker is None or not self.speaker.callouts or self.speaker.callouts[-1].get("applied"):
+            return
+        rec = self.speaker.callouts[-1]
+        rec["applied"] = True
+        quiet = []
+        for sid, r in rec["sectors"].items():
+            p = self.piles.get(sid)
+            if p is None:
+                continue
+            if r["status"] == "no_response":
+                quiet.append(sid)
+                continue
+            self._say(f"Call-out {rec['id']} (listen ended t {rec['listen_ends']:.0f} s) [{sid}]: {r['text']}")
+            if r["status"] != "responded":
+                continue
+            for c in self.clips:
+                if c["pile_id"] == sid and c["listen"] == self.n_listen and c["scores"].get("reliability") is not None:
+                    rel, note = association_boost(c["scores"]["reliability"], r)
+                    c["scores"]["reliability"] = rel
+                    c["priority"] = clip_priority({k: c["scores"][k] or 0.0 for k in ("human", "urgency", "reliability")})
+                    c.setdefault("notes", []).append(note)
+            pr = response_priority(r)
+            if p["status"] != "ping" and pr is not None and pr > (p["priority"] or 0):
+                p.update(priority=pr, status="lead",
+                         why=f"{r['text']} {p['why']}" if p["why"] else r["text"])
+        for sid, agg in self.speaker.state(self.t)["responsiveness"].items():   # strongest evidence + history
+            if sid in self.piles:
+                self.piles[sid]["responsiveness"] = agg
+        if quiet:
+            self._say(f"Call-out {rec['id']}: no response from {', '.join(quiet)} (does not rule out an unconscious "
+                      "person; not cleared)")
+        for c in self.done_cands:
+            pr = rec["pings"].get(c["id"])
+            old = c.get("responsiveness")
+            rank = {"responded": 3, "possible_response": 2, "nonhuman_only": 1, "no_response": 0}
+            if pr and (old is None or rank[pr["status"]] >= rank[old["status"]]):    # strongest evidence wins
+                c["responsiveness"] = {k: pr[k] for k in ("status", "kind", "avpu", "confidence", "text", "callout_t")}
+                if c.get("vitals"):
+                    c["triage_fused"] = fuse_triage(c["vitals"], pr)
+        if rec["reassured"]:
+            self._say(f"Reassurance played from t {rec['listen_ends']:.0f} s ({', '.join(rec['langs'])}) towards {', '.join(rec['reassured'])}: "
+                      "\"Help is coming. Stay still, save your energy, tap if you can hear me.\"")
+        for g in rec["guided"]:
+            self._say(f"Guidance played to {g['track']} at ({g['xy'][0]:.0f}, {g['xy'][1]:.0f}) ({g['source']}): "
+                      f"\"{g['text_en']}\" Route {g['route']['length_m']:.0f} m clear of rubble and tasked zones")
+
     @staticmethod
     def _clip_bearings(c, drones):
         """Per-drone bearings of a clip: [{drone_id, origin, bearing_deg, width_deg, snr_db}] (as reported)."""
@@ -928,6 +1016,7 @@
 
     def _drones_out(self):
         out = []
+        playing = self.speaker.active(self.t) if self.speaker is not None else {}
         for d in self.flight.poses():
             d = dict(d)
             tr = self._tracks.setdefault(d["id"], [])
@@ -937,6 +1026,8 @@
             d["track"] = list(tr)
             if self.stage == "listen" and d["state"] in ("landed", "scanning"):
                 d["state"] = "listening"
+            if self.speaker is not None and d["id"] in playing:
+                d["speaker"] = playing[d["id"]]
             out.append(d)
         return out
 
@@ -1003,6 +1094,9 @@
             "dispatch": self.dispatch,
             "decisions": self.decisions,
             "slam": list(self.slam_metrics.values()) if self.slam else None,
+            # ---- v1.3 speaker (broadcasts, callouts, responsiveness per sector)
+            **(self.speaker.state(self.t) if self.speaker is not None else
+               {"broadcasts": [], "callouts": [], "responsiveness": {}}),
         }
 
     def build_state(self):
@@ -1074,6 +1168,8 @@
     ap.add_argument("--slam", action="store_true",
                     help="build each pile's map with LiDAR SLAM (mapping.slam, KISS-ICP); cached in dashboard/data/slam_cache")
     ap.add_argument("--slam-scans", type=int, default=SLAM_SCANS)
+    ap.add_argument("--no-speaker", dest="speaker", action="store_false",
+                    help="disable the loudspeaker call-out / reassurance / guidance (contract v1.3)")
     ap.add_argument("--quiet", action="store_true")
     a = ap.parse_args(argv)
     t0 = time.time()
@@ -1088,7 +1184,7 @@
                     pile_budget=a.piles, priority_min=a.priority_min, acoustic=a.acoustic, decider=decider,
                     slam=a.slam, slam_scans=a.slam_scans, autonomous=a.autonomous, mission_scans=a.scans,
                     dispatch_policy=a.dispatch, penetration=a.penetration, site=a.site,
-                    platform=a.platform, delivery=a.delivery)
+                    platform=a.platform, delivery=a.delivery, speaker=a.speaker)
     m.run()
     s = m.summary()
     print(f"done in {time.time() - t0:.1f} s wall, {m.t:.0f} s sim; visited {' -> '.join(s['visited'])}; "
```

**Scratch run.** `sim/` was copied to the scratch directory, the patch applied, and the run made with `scripts/heavy.sh .venv/bin/python -m sim.grid_mission --seed 7 --acoustic stub`.

**(a) Committed baseline (before 6adeb2a), speaker on:**

- Visits A2 → A3 → B2 and pings A2-R1, A3-R1, B2-R1: the same as `--no-speaker`.
- Sim time 968 s against 583 s.
- 35 broadcasts: 15 call-out, 15 reassure, 5 guidance.
- A3 (calm) **responded** (voice 1.6 s after the zh-TW message, 3 drones).
- B1 (apnoeic): no response.
- The hidden bystander at (−31.2, −12.8) was triangulated to (−31, −12) and guided "Walk south about 15 metres, then south-east about 40 metres, to the rescue tent", heading 271.6°.

**(b) Latest working tree (21:xx), including other agents' uncommitted platform / delivery / vitals edits and the realistic-penetration default.** That tree gives no pings and revisits A3 with or without this patch.

- Speaker on: 1068 s against 746 s, the same visits.
- 4 call-outs, 30 broadcasts.
- A3 (calm) and A2 (distressed) responded.
- B1 (apnoeic) and B2 (now `unconscious_slow`): no response, every time.
- No bystander track this run (recall is conservative).

**Tests.** `tests/test_callout.py`: 17 passed. `tests/test_grid_mission.py` on the patched latest tree has 3 failures (`test_visits_multiple_piles_in_acoustic_priority_order`, `test_seed7_dsp_default_pings_every_voice_pile`, `test_step_api_and_listening_state`). **The same 3 fail without the patch**: they come from the new penetration default and the working-tree edits, not from the speaker. On the committed baseline all 18 passed with the patch.

## 7. Requests

1. Apply the mission patch (§6) and the serve.py patch (§5). Append §4 to INTERFACES.md.
2. **Dashboard: a "Speaker" panel.**
   - Compose box: drone or zone picker, kind, source language, target languages, a translate toggle, and the operator's name.
   - Pending translations shown side by side with the English, each with Approve / Reject buttons (`/api/broadcast/approve`). The UI never auto-plays them.
   - A broadcast log, newest first, showing language, kind, target, `audio_status` and a ▶ play button for `audio_url`.
   - Responsiveness chips per pile and ping: responded (V) / possible / no response, the last with the tooltip "does not rule out an unconscious person".
   - A speaker icon on drones with `drones[].speaker`.
   - Guidance paths (`callouts[].guided[].path`) drawn as dashed arrows to the tent.
3. Scenario (`sim/scenario.py` owner): physio state and sound role are drawn independently. In seed 7, B1 is `irregular_apneic` but is also the "hello... water..." talker, and B2 is `unconscious_slow` but is also the knocker. The call-out generator mutes them, but the passive acoustic scene does not. Suggestion: give unconscious or apnoeic victims only silent roles, or mute them in `acoustic/scene.py` and the stub.
4. Taiwanese Hokkien: a native-speaker recording of both lines, plus a native check of all the zh-TW, id and vi strings (`REVIEW`).
5. Mission time grows by about 100–180 s per pile. If that is too much, play only zh-TW and nan-TW plus English (`Speaker(langs=...)`), or shorten `LISTEN_WINDOW_S`. That is a product decision.

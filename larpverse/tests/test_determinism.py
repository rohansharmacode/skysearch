"""Same seed + same flags -> same story, independent of PYTHONHASHSEED (set / dict ordering) and wall clock."""
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SNIPPET = r"""
import hashlib, json, sys
sys.path.insert(0, %r); sys.path.insert(0, %r)
from test_demo_story import replay_fn
from sim.grid_mission import GridMission
fn, _ = replay_fn()
m = GridMission(seed=7, verbose=False, acoustic_fn=fn, tasking="default", pile_budget=3).run()
log = json.dumps([e["msg"] for e in m.log])
print("@@" + json.dumps({"pings": [[c["id"], c["status"], c["t_found"]] for c in m.done_cands],
                         "visited": m.visited, "t": m.t, "log": hashlib.sha1(log.encode()).hexdigest()}))
"""


def _run(hashseed):
    env = dict(os.environ, PYTHONHASHSEED=str(hashseed))
    p = subprocess.run([sys.executable, "-c", SNIPPET % (ROOT, os.path.join(ROOT, "tests"))], cwd=ROOT, env=env,
                       capture_output=True, text=True, timeout=900)
    assert p.returncode == 0, p.stderr[-2000:]
    return json.loads(p.stdout.rsplit("@@", 1)[1])


def test_seed7_story_is_deterministic_across_hash_seeds():
    a, b = _run(0), _run(4242)
    assert a == b

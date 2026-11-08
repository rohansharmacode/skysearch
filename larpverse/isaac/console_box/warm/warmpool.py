"""Warm pool for the operator console: keep >= 1 held, unused Isaac instance per site so a site switch / Reset is
instant (serve.py swaps the dashboard/data symlink + the viewer port to a warm slot).

Slots 1..MAX: data dir dashboard/slotN, Kit signal port 49100+N, stream port 47998+N, viewer (vite) port 5173+N.
State: dashboard/pool.json {"slots": {"N": {site, seed, look, state: loading|warm|active|retire, pgid, t}}}
serve.py marks a slot "active" (and the previous active one "retire"); this loop kills retired slots and launches a
replacement warm instance for any site that has none. One launch at a time (boots contend for CPU).
"""
import json
import os
import signal
import subprocess
import time

L = "/home/ubuntu/lv/larpverse"
POOL = f"{L}/dashboard/pool.json"
SITES = [("grid", 7, "day"), ("weiguan", 7, "day"), ("collapse", 7, "day")]
MAX = int(os.environ.get("POOL_MAX", "5"))
LOG = "/home/ubuntu/warm/warmpool.log"


def log(m):
    with open(LOG, "a") as f:
        f.write(time.strftime("%FT%TZ ", time.gmtime()) + m + "\n")


def load():
    try:
        return json.load(open(POOL))
    except Exception:  # noqa: BLE001
        return {"slots": {}}


def save(d):
    tmp = POOL + ".tmp"
    json.dump(d, open(tmp, "w"), indent=1)
    os.replace(tmp, POOL)


def alive(pgid):
    try:
        os.killpg(int(pgid), 0)
        return True
    except Exception:  # noqa: BLE001
        return False


def console_status(n):
    try:
        return json.load(open(f"{L}/dashboard/slot{n}/console.json")).get("status")
    except Exception:  # noqa: BLE001
        return None


def launch(n, site, seed, look):
    p = subprocess.Popen(["bash", "/home/ubuntu/warm/launch_slot.sh"], cwd="/home/ubuntu/warm",
                         env=dict(os.environ, SLOT=str(n), SITE=site, SEED=str(seed), LOOK=look),
                         stdout=open(f"/home/ubuntu/warm/slot{n}.log", "w"), stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL, start_new_session=True)
    log(f"launch slot {n}: {site} {seed} {look} (pgid {p.pid})")
    return p.pid


def main():
    log("warmpool started")
    import fcntl
    while True:
        lk = open(POOL + ".lock", "w"); fcntl.flock(lk, fcntl.LOCK_EX)
        d = load()
        slots = d.setdefault("slots", {})
        changed = False
        for n, s in list(slots.items()):
            if s.get("state") == "retire":
                if s.get("pgid") and int(s["pgid"]) != os.getpgid(0) and alive(s["pgid"]):   # never our own group
                    try:
                        os.killpg(int(s["pgid"]), signal.SIGTERM)
                    except Exception:  # noqa: BLE001
                        pass
                log(f"retired slot {n} ({s.get('site')})")
                del slots[n]; changed = True
            elif s.get("state") in ("loading", "warm") and s.get("pgid") and not alive(s["pgid"]):
                log(f"slot {n} died ({s.get('site')}, {s.get('state')})"); del slots[n]; changed = True
            elif s.get("state") == "loading" and console_status(n) == "waiting_for_goto":
                s["state"], s["t_warm"] = "warm", time.time(); changed = True
                log(f"slot {n} warm: {s['site']} in {time.time() - s['t']:.0f} s")
        loading = [s for s in slots.values() if s.get("state") == "loading"]
        if not loading:
            act = [s.get("site") for s in slots.values() if s.get("state") == "active"]
            for site, seed, look in sorted(SITES, key=lambda t: t[0] not in act):   # on-screen site first (Reset)
                if any(s.get("site") == site and s.get("state") in ("warm", "loading") for s in slots.values()):
                    continue
                free = [n for n in range(1, MAX + 1) if str(n) not in slots]
                if not free:
                    break
                n = free[0]
                slots[str(n)] = {"site": site, "seed": seed, "look": look, "state": "loading", "t": time.time(),
                                 "pgid": launch(n, site, seed, look)}
                changed = True
                break                                   # one boot at a time
        if changed:
            save(d)
        lk.close()
        time.sleep(3)


if __name__ == "__main__":
    main()

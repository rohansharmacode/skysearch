"""Start / stop / auto-stop the Brev GPU instance so we only pay while we use it.

Needs the Brev CLI logged in (`brev login`). On Windows the CLI usually runs in WSL; this
script calls `wsl brev` / `wsl ssh` automatically when plain `brev` isn't on PATH.

    python tools/brev_ctl.py status
    python tools/brev_ctl.py up          # start + wait until running + refresh SSH config
    python tools/brev_ctl.py down        # stop (disk is kept)
    python tools/brev_ctl.py watchdog --idle-min 30   # stop automatically after N idle minutes
    python tools/brev_ctl.py run "nvidia-smi"         # run a command on the instance

"Idle" = GPU utilisation under 5% AND no Isaac process running, checked every 2 minutes.
"""
import argparse, os, re, shutil, subprocess, sys, time

INSTANCE = os.environ.get("BREV_INSTANCE", "isaac-launchable-e2055c")


def brev_cmd():
    if shutil.which("brev"):
        return ["brev"]
    if os.name == "nt" and shutil.which("wsl"):
        return ["wsl", "-e", "bash", "-lc", "brev \"$@\"", "brev"]
    sys.exit("brev CLI not found. Install: https://docs.nvidia.com/brev/latest/brev-cli.html")


def ssh_cmd():
    if shutil.which("brev") or os.name != "nt":
        return ["ssh", "-T", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=20"]
    return ["wsl", "-e", "ssh", "-T", "-o", "StrictHostKeyChecking=no", "-o", "ConnectTimeout=20"]


def sh(args, check=False, capture=True):
    r = subprocess.run(args, capture_output=capture, text=True)
    if check and r.returncode != 0:
        sys.exit(f"failed: {' '.join(args)}\n{r.stdout}\n{r.stderr}")
    return r


def status():
    out = sh(brev_cmd() + ["ls"]).stdout
    for line in out.splitlines():
        if INSTANCE in line:
            m = re.search(r"\b(RUNNING|STOPPED|STOPPING|STARTING|DEPLOYING|CREATING|FAILURE|PROVISIONING)\b", line, re.I)
            return (m.group(1).upper() if m else "UNKNOWN"), line.strip()
    return "NOT FOUND", out


def up():
    st, _ = status()
    if st != "RUNNING":
        print(f"{INSTANCE} is {st}: starting")
        sh(brev_cmd() + ["start", INSTANCE])
    for _ in range(90):
        st, line = status()
        if st == "RUNNING":
            break
        print("  waiting...", st); time.sleep(10)
    sh(brev_cmd() + ["refresh"])
    print("running:", line)


def down():
    print(f"stopping {INSTANCE}")
    print(sh(brev_cmd() + ["stop", INSTANCE]).stdout)


def run(cmd):
    r = sh(ssh_cmd() + [INSTANCE, cmd])
    return r.returncode, r.stdout + r.stderr


def busy():
    code, out = run("nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits; "
                    # bracketed first letters stop pgrep -f matching this very ssh command line
                    "pgrep -f '[i]saac_bridge|[i]saaclab.sh|[k]it/kit|[e]arthquake_scene' >/dev/null && echo ISAAC_RUNNING || true")
    if code != 0:
        return True, f"ssh failed ({out.strip()[:80]}); assuming busy"
    nums = [int(x) for x in re.findall(r"^\s*(\d+)\s*$", out, re.M)]
    util = max(nums) if nums else 0
    isaac = "ISAAC_RUNNING" in out
    return (util >= 5 or isaac), f"gpu {util}% isaac={'yes' if isaac else 'no'}"


def watchdog(idle_min):
    idle_since = None
    print(f"watchdog: will stop {INSTANCE} after {idle_min} idle minutes (Ctrl+C to quit)")
    while True:
        st, _ = status()
        if st != "RUNNING":
            idle_since = None
            print(time.strftime("%H:%M"), st); time.sleep(120); continue
        b, why = busy()
        now = time.time()
        if b:
            idle_since = None
        elif idle_since is None:
            idle_since = now
        idle = 0 if idle_since is None else (now - idle_since) / 60
        print(time.strftime("%H:%M"), why, f"idle {idle:.0f} min")
        if idle >= idle_min:
            down(); idle_since = None
        time.sleep(120)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["status", "up", "down", "watchdog", "run"])
    ap.add_argument("cmd", nargs="?", default="")
    ap.add_argument("--idle-min", type=float, default=30)
    a = ap.parse_args()
    if a.action == "status":
        print(*status())
    elif a.action == "up":
        up()
    elif a.action == "down":
        down()
    elif a.action == "watchdog":
        watchdog(a.idle_min)
    elif a.action == "run":
        print(run(a.cmd)[1])

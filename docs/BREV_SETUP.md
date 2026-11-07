# Isaac Sim on NVIDIA Brev: first-hour checklist

Get the stream working before anything else; the slides ask for it in the first hour. The swarm itself already
runs on CPU (`python -m skysearch.live`), so this hour is only about Isaac Sim.

## 1. Deploy the Isaac launchable (Rohan, about 10–20 min including first build)
1. Open https://brev.nvidia.com/launchable/deploy/now?launchableID=env-35JP2ywERLgqtD0b0MIeK1HnF46 and sign in.
2. Check the GPU and hourly price shown, then click **Deploy Launchable**.
3. Wait until the instance is running, built and setup has finished. The first launch is slow.
4. On the instance page, open **Using Secure Links**, click the arrow next to the Shareable URL, and log in with Brev. This opens **VS Code in the browser**.

Brev bills by the hour, so **stop the instance whenever we're not using it**.

## 2. Upload our code
- In VS Code's file explorer, drag in `skysearch.zip` (or `git clone` our repo once we have one).
- In the VS Code terminal:
```bash
cd ~ && unzip -o skysearch.zip   # or the folder path you uploaded to
cd /workspace/isaaclab
./isaaclab.sh -p -m pip install scipy matplotlib
```

## 3. Run the swarm in Isaac Sim, streamed
```bash
cd /workspace/isaaclab
./isaaclab.sh -p ~/skysearch/isaac/isaac_bridge.py --livestream 2 --viz kit --enable_cameras
```
- Wait for `Simulation App Startup Complete`.
- **Viewport:** open a new browser tab with the VS Code URL, changing the end to `/viewer`. You can also use Ctrl+Shift+P, then "Simple Browser: Show", then `/viewer/`. Keep only one viewer tab open.
- **Mission control dashboard (port 8090):** `<VS Code URL>/proxy/8090/`. Port 8080 is taken by the launchable's code-server.
- **If the dashboard won't load:** run the CPU dashboard on a laptop for the side panel. Or ask Brev to expose port 8080 (the custom-launchable settings allow extra port rules).

## 4. Record the 60-second video
Put the `/viewer` tab and the dashboard side by side and screen-record one continuous take. A teammate clicks one unscripted event mid-take.

## If it isn't streaming by hour one
Stop fighting it. Record the CPU dashboard, which has the same brain and the same buttons, and keep debugging Isaac in parallel.

## Isaac Sim 6.0.1 port
The launchable ships Isaac Sim 6.0.1 + Isaac Lab 3.0, not 5.x. `isaac_bridge.py` has been ported; see `docs/ISAAC6_API.md` for what works, what's broken, and the open black-viewport issue.

## Original risk notes (Isaac Sim 5.x)
- **Import paths:** expect to fix one or two (`isaacsim.core.api`, `isaacsim.sensors.camera`, semantics helpers). Each is used in one place.
- **Camera orientation:** `[0, 90, 0]` should point straight down. Check one camera frame.
- **Performance:** if it's slow, raise `--tree_stride` (default 2 keeps 1,250 trees).
- **Thermal camera:** this is a proxy built from semantic segmentation of warm bodies. Say so in the technical summary.

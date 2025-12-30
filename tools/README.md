# BGR Simulator launcher (tools/)

This folder contains the team-friendly launcher scripts to run the simulator with **Docker** (recommended) or **Native** (optional).

## Quick start (daily use)

From the repo root:

```bash
chmod +x tools/run_gui_.sh
./tools/run_gui_.sh
```

Choose:
- **1) Start ALL (Docker)** – builds image/workspace if needed, then launches Gazebo + RViz.

### If Docker GUI doesn't open
On the host (once per session):

```bash
xhost +local:docker
echo $DISPLAY
```

Make sure `DISPLAY` is not empty.

## First time setup (new machine)

1) Install Docker Engine
2) Clone repo
3) Run:

```bash
chmod +x tools/run_gui.sh
./tools/run_gui.sh
```

Then choose:
- **3) Check / Build Docker image**
- **4) Build workspace (Docker)**

After that, daily use is just **Start ALL (Docker)**.

## CLI mode (for future GUI / automation)

Examples:

```bash
./tools/run_gui.sh docker all
./tools/run_gui.sh docker gazebo
./tools/run_gui.sh docker rviz
./tools/run_gui.sh docker build-ws
```

## Notes

- The build outputs (`build/ install/ log/`) are created on the host repo (because the repo is mounted into the container).
- The launcher does a simple “needs rebuild” check: if any file in `src/` is newer than `install/setup.bash`, it rebuilds.

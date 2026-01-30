# cdpr_ros2_ws

Comprehensive research workspace for Cable-Driven Parallel Robot (CDPR) control and simulation built on ROS 2 Jazzy and Gazebo Harmonic.

This repository contains ROS 2 packages, simulation worlds, launch files, visualization configs, and CI configuration to build and test the workspace automatically. It is organized to support reproducible research, collaborative development, and continuous integration.

---

Table of contents
- Overview
- Repository layout
- Requirements (software + OS)
- Quickstart (clone, install deps, build, run)
- How to run (detailed: helper script, launch_all.sh, manual launch)
- Detailed setup (WSL notes, ROS/Gazebo install)
- Common workflows (edit → commit → push; branching & PRs)
- Running simulations (headless vs GUI)
- Testing and CI (GitHub Actions)
- Reproducibility & environment capture
- Large files, assets and Git LFS
- Devcontainers / Codespaces
- Troubleshooting
- Contributing
- License & citation
- Contact

---

## Overview

This workspace contains ROS 2 packages for CDPR control and simulation:
- cdpr_control — controllers, nodes, launch files, visualization scripts
- cdpr_description — URDF/SDF and robot description assets
- cdpr_gz_plugins — Gazebo/GZ plugins for cables, sensors, etc.
- other supporting packages and scripts

Primary goals:
- Provide a reproducible environment for simulation experiments
- Be easy to build locally (WSL or native Ubuntu) and in CI (GitHub Actions)
- Keep large generated artifacts out of git (build/, install/, logs/, virtualenvs)
- Provide automation for builds/tests and clear developer workflows

---

## Repository layout

Example (top-level of repo):

```
cdpr_ros2_ws/
├── .github/
│   └── workflows/
│       └── ros2-ci.yml          # GitHub Actions CI workflow
├── src/
│   ├── cdpr_control/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── src/
│   │   ├── include/
│   │   ├── launch/
│   │   ├── rviz/
│   │   ├── worlds/
│   │   ├── scripts/
│   │   └── config/
│   ├── cdpr_description/
│   └── cdpr_gz_plugins/
├── worlds/
├── scripts/
├── README.md
└── .gitignore
```

Recommended files:
- `README.md` — this file (usage, build, CI, reproducibility)
- `.gitignore` — ignore build artifacts, venvs and logs
- `.github/workflows/ros2-ci.yml` — CI pipeline (build + tests)
- `src/` — ROS 2 packages

---

## Requirements

Minimum recommended environment for development (local):

- Host OS:
  - Ubuntu (native) or Windows 11 with WSL2 + Ubuntu distribution (recommended: Ubuntu 22.04 or distro that supports ROS 2 Jazzy; check ROS 2 Jazzy compatibility)
- ROS 2: Jazzy (installed per official instructions)
  - docs: https://docs.ros.org
- Gazebo: Harmonic (Gazebo / GZ package) — install official OSRF packages
  - docs: https://gazebosim.org
- Utilities:
  - git, curl, wget
  - python3, python3-pip, python3-venv (optional but recommended)
  - colcon: `python3 -m pip install -U colcon-common-extensions`
  - rosdep, vcstool
- Optional:
  - Visual Studio Code + Remote - WSL extension
  - Docker (for building devcontainers or self-hosted CI runners)
  - Git LFS if you track large binary assets

Notes:
- Replace references to `jazzy`/`harmonic` with actual distro names if there are changes.
- For GPU-accelerated or GUI simulations, run Gazebo locally rather than in CI.

---

## Quickstart (clone, install deps, build, run)

Run these commands in WSL/Ubuntu (adjust when using native Ubuntu):

1. Clone
```bash
git clone https://github.com/Tachia/cdpr_ros2_ws.git
cd cdpr_ros2_ws
```

2. Initialize rosdep (if not done before)
```bash
sudo apt update
sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update
```

3. Install system dependencies declared in package.xml
```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Source ROS 2 and build
```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install -U colcon-common-extensions
colcon build --event-handlers console_direct+
```

5. Source the workspace
```bash
source install/setup.bash
```

6. Launch an example simulation
```bash
# Example: adjust package/launch names to actual files in this repo
ros2 launch cdpr_control cdpr_simulation.launch.py
```

---

## How to run (recommended, reproducible, and exact commands)

This section shows the most reliable ways to run the simulation locally and how to prepare shells so `ros2` commands can find your packages. Use one of the two main options below: (A) use the provided launcher (launch_all.sh), or (B) use manual/ros2 launch with an explicit build + source step.

Important: every new terminal session must have ROS 2 sourced and the workspace overlay sourced before ROS 2 tools see your packages.

A — Recommended quick-run (single-command launcher)
- If `launch_all.sh` is present and working, it already builds, sources and starts all components (Gazebo, ROS nodes, RViz, python visualizer). From the repo root:
```bash
cd ~/cdpr_ros2_ws
./launch_all.sh
# To stop: Ctrl+C in the terminal or follow instructions printed by launch_all.sh
```
- `launch_all.sh` is convenient for experiments because it encapsulates build + launch steps.

B — Manual (build once, iterate, and launch with ros2)
1. Open a terminal and source the ROS 2 distro:
```bash
source /opt/ros/jazzy/setup.bash
```

2. Build the whole workspace (or only the package you changed):
```bash
# full build
cd ~/cdpr_ros2_ws
colcon build --event-handlers console_direct+

# or build only cdpr_control (faster during development)
colcon build --packages-select cdpr_control --event-handlers console_direct+
```

3. Source the workspace overlay (this makes ros2 aware of local packages):
```bash
source ~/cdpr_ros2_ws/install/setup.bash
```

4. Verify the package is visible:
```bash
ros2 pkg list | grep cdpr_control
# Should print 'cdpr_control' if the package is visible
```

5. Launch the simulation using ros2:
```bash
ros2 launch cdpr_control cdpr_simulation.launch.py
```

6. Useful debug commands (while things run):
```bash
# list topics
ros2 topic list

# echo a topic
ros2 topic echo /platform/pose

# check node graph
ros2 node list

# check where the package is located
ros2 pkg prefix cdpr_control
```

C — Create and use a small helper script to source environment (recommended)
Create `setup_ws.sh` at the workspace root and source it in any new terminal to quickly prepare the shell:

```bash
cat > ~/cdpr_ros2_ws/setup_ws.sh <<'EOF'
#!/usr/bin/env bash
# setup_ws.sh - source ROS distro and this workspace overlay
ROS_DISTRO=jazzy
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
else
  echo "Warning: /opt/ros/${ROS_DISTRO}/setup.bash not found"
fi
if [ -f "$(pwd)/install/setup.bash" ]; then
  source "$(pwd)/install/setup.bash"
else
  echo "Workspace overlay not found. Run: colcon build"
fi
EOF
chmod +x ~/cdpr_ros2_ws/setup_ws.sh
# Use it like:
cd ~/cdpr_ros2_ws
source ./setup_ws.sh
```

D — Headless Gazebo run (example)
If you want to run Gazebo without GUI (useful for CI or headless machines):
```bash
# start Gazebo server (headless)
gzserver worlds/cdpr_world.sdf &

# then launch ROS nodes
ros2 launch cdpr_control cdpr_simulation.launch.py
```

E — Stopping processes
- If launch scripts spawn multiple processes, stop using Ctrl+C (if in foreground).
- Or kill by name if needed:
```bash
pkill -f gzserver
pkill -f rviz2
pkill -f cdpr_control   # careful: only kill your expected processes
```

F — Notes for WSL + GUI (Windows)
- For Gazebo GUI run in WSL, install an X server on Windows (VcXsrv, X410) and set DISPLAY before launching GUI processes:
```bash
# on Windows start VcXsrv; in WSL:
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=1
ros2 launch cdpr_control cdpr_simulation.launch.py   # will try to open Gazebo/RViz windows
```
- An alternative is to run Gazebo/RViz natively on a Linux VM or native Ubuntu for better performance.

---

## Detailed setup notes

### WSL 2 on Windows 11
- Use WSL2 with Ubuntu (recommended 22.04). Install WSL per Microsoft docs.
- Install ROS 2 Jazzy in your WSL Ubuntu environment (follow ROS docs for Debian packages or binaries).
- For Gazebo GUI on Windows: display via X server (VcXsrv / X410) or run GUI natively.

### Python virtualenv (optional, recommended for isolated Python deps)
```bash
python3 -m venv cdpr_venv
source cdpr_venv/bin/activate
pip install -U pip
pip install -r src/cdpr_control/requirements.txt  # if provided
```

### .gitignore (important)
Ensure `build/`, `install/`, `log/`, and virtualenv directories are ignored. Example:
```
build/
install/
log/
cdpr_venv/
*.pyc
__pycache__/
.vscode/
```

---

## Common development workflow

1. Configure git (one-time):
```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

2. Create a branch for a feature/experiment:
```bash
git checkout -b feature/experiment-A
```

3. Make edits, test locally, then stage and commit:
```bash
git add path/to/file
git commit -m "Add controller skeleton for experiment A"
```

4. Push branch and open a PR:
```bash
git push -u origin feature/experiment-A
# Then open a PR on GitHub to merge into main
```

5. Merge when CI passes and reviews are complete.

---

## Running simulations: headless vs GUI

- Headless (suitable for CI and automation):
  - Launch `gzserver` (or equivalent) and run nodes without GUI.
  - Use headless mode in CI — GUI cannot run on GitHub-hosted runners.

- GUI:
  - To view Gazebo UI, run `gzclient` (or `gz`/`ign`) locally or use an X server/remote display for WSL.
  - For best experience run Gazebo natively on Ubuntu with GPU drivers if needed.

---

## Testing and CI (GitHub Actions)

This repository includes `.github/workflows/ros2-ci.yml` which:
- Installs ROS 2 Jazzy via the `ros-tooling/setup-ros` action
- Adds OSRF Gazebo apt key so `rosdep` can install gazebo-related deps
- Runs `rosdep install`, `colcon build`, `colcon test`, and uploads logs on failure

Notes about CI:
- Hosted runners are headless — GUI interactions are not possible
- If a package requires additional apt packages, add them in the workflow or list them in package.xml
- Consider caching colcon artifacts (advanced) to speed up repeated builds

---

## Reproducibility & environment capture

To make experiments reproducible:
- Record exact ROS/Gazebo versions:
  ```bash
  ros2 --version
  gz --version   # or gazebo --version depending on your installation
  lsb_release -a
  uname -a
  ```
- Capture Python packages:
  ```bash
  pip freeze > requirements-pip.txt
  ```
- Capture OS package list (optional):
  ```bash
  apt list --installed > apt-installed.txt
  ```

---

## Large assets & Git LFS

For large binary assets (slam maps, recorded bags, models):
- Install Git LFS: `sudo apt install git-lfs && git lfs install`
- Track patterns:
  ```bash
  git lfs track "*.bag"
  git lfs track "models/*"
  git add .gitattributes
  git commit -m "Track large assets with Git LFS"
  ```

---

## Devcontainers & Codespaces

A `.devcontainer/` is recommended for contributors to get a ready dev environment:
- Devcontainer can install ROS 2 packages and colcon
- Note: display forwarding for Gazebo GUI in Codespaces is typically not available; use devcontainers for building & headless testing only. For GUI, prefer local development or self-hosted runners.

---

## Troubleshooting (common issues)

- "Package 'cdpr_control' not found"
  - You must source the workspace overlay after building:
    ```bash
    source /opt/ros/jazzy/setup.bash
    colcon build
    source install/setup.bash
    ```
  - Or run the included `launch_all.sh` which builds and sources automatically.

- "Please tell me who you are" on commit:
  ```bash
  git config --global user.name "Your Name"
  git config --global user.email "you@example.com"
  ```

- Large accidentally committed build/venv files:
  ```bash
  git rm -r --cached build install log cdpr_venv
  git add .gitignore
  git commit -m "Remove build/install/log and add .gitignore"
  git push
  ```

- rosdep missing keys (e.g., OSRF Gazebo apt key):
  - Add the OSRF key:
    ```bash
    curl -fsSL https://packages.osrfoundation.org/gazebo.key | sudo gpg --dearmor -o /usr/share/keyrings/osrf-archive-keyring.gpg
    ```

- SSH push failing with `Permission denied (publickey)`:
  - Add your SSH public key to GitHub (Settings → SSH and GPG keys)
  - Ensure `ssh-agent` has your key: `eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519`

---

## How to update this repository files (via terminal)

Examples (from repo root):

1. Stage and commit changes:
```bash
git add README.md src/cdpr_control/...
git commit -m "Describe changes"
git push
```

2. Create feature branch:
```bash
git checkout -b feature/new-launch
# edit files
git add .
git commit -m "Add new launch to test scenario"
git push -u origin feature/new-launch
```

3. Update workflow or README locally and push:
```bash
git add .github/workflows/ros2-ci.yml README.md
git commit -m "Update CI workflow and README"
git push
```

4. Create a release (tag and push tag):
```bash
git tag -a v0.1.0 -m "Initial release for reproducible experiments"
git push origin v0.1.0
```

---

## Contributing

We welcome contributions. Recommended flow:
1. Fork the repository (if external)
2. Create a branch for your feature/fix
3. Make changes and add tests if appropriate
4. Push your branch and open a Pull Request
5. Add a clear description of the change, test results, and any instructions to reproduce

---

## License & citation

Choose and include an OSI-approved license in `LICENSE` (e.g., MIT, Apache-2.0). If this repository is part of published research, include a CITATION file or a citation block in README explaining how to cite the work (BibTeX entry).

Example citation block to add to README:
```text
If you use this repository in your research, please cite:
Tachia M. J., Maloletov A. V., Applications of Dynamic Models for Cable-Driven Parallel Robots: A Comprehensive Review, Rus. J. Nonlin. Dyn., 2025, https://doi.org/10.20537nd251101
```
```text
Also cite this:
M. J. Tachia and A. Maloletov, “Reinforcement Learning-Based Control for Cable Sag Compensation in Cable-Driven Parallel Robots Using Soft Actor-Critic Algorithm,” in 2025 VI International Conference on Control in Technical Systems (CTS), Saint Petersburg, Russian Federation: IEEE, Sept. 2025, pp. 171–176. doi: 10.1109/CTS67336.2025.1119669
```

---

## Contact & acknowledgements

- Maintainer: Mfeuter Joseph Tachia — tachiajoseph3@gmail.com
- Acknowledgements: List funding sources, collaborators, libraries and toolkits used (ROS 2, Gazebo, etc.)

---

Thank you for using this repository for your research. If you want, I can:
- generate a CITATION.bib entry,
- produce a template `experiments/README.md` describing experiment protocols, or
- create a devcontainer/Dockerfile that installs ROS 2 Jazzy + Gazebo Harmonic for reproducible CI/dev.

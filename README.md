# cdpr_ros2_ws

Comprehensive research workspace for Cable-Driven Parallel Robot (CDPR) control and simulation built on ROS 2 Jazzy and Gazebo Harmonic.

This repository contains ROS 2 packages, simulation worlds, launch files, visualization configs, and CI configuration to build and test the workspace automatically. It is organized to support reproducible research, collaborative development, and continuous integration.

---

Table of contents
- Overview
- Repository layout
- Requirements (software + OS)
- Quickstart (clone, install deps, build, run)
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

## Detailed setup notes

### WSL 2 on Windows 11
- Use WSL2 with Ubuntu (recommended 22.04). Install WSL per Microsoft docs.
- Install ROS 2 Jazzy in your WSL Ubuntu environment (follow ROS docs for Debian packages or binaries).
- For Gazebo GUI on Windows: you can display GUI via an X server (VcXsrv or X410) or use a remote desktop. Many developers run Gazebo natively on Windows or run headless simulation in WSL and view results separately.

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
  - Example:
    ```
    # Run Gazebo server (headless) and a ROS 2 launch
    gzserver worlds/cdpr_world.sdf &   # or appropriate gz/gazebo cmd
    ros2 launch cdpr_control cdpr_simulation.launch.py
    ```
  - Use headless mode in CI — GUI cannot run on a GitHub-hosted runner.

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
- Create a small environment manifest (e.g., `env/ros_jazzy_harmonic.md`) describing OS, ROS, Gazebo, and key packages used.

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
- Alternatively, host very large datasets externally (S3, Zenodo, institutional storage) and link from README.

---

## Devcontainers & Codespaces

A `.devcontainer/` is recommended for contributors to get a ready dev environment:
- Devcontainer can install ROS 2 packages and colcon
- Note: display forwarding for Gazebo GUI in Codespaces is typically not available; use devcontainers for building & headless testing only. For GUI, prefer local development or self-hosted runners.

---

## Troubleshooting (common issues)

- "Please tell me who you are" on commit:
  ```bash
  git config --global user.name "Your Name"
  git config --global user.email "you@example.com"
  ```

- Large accidentally committed build/venv files:
  ```bash
  # add .gitignore then:
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
  - Ensure your apt source list references `signed-by=/usr/share/keyrings/osrf-archive-keyring.gpg`

- CI failure due to missing system libs:
  - Add apt install lines to the CI workflow or add the package names to package.xml so `rosdep` can resolve them.

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
# Make changes to .github/workflows/ros2-ci.yml or README.md
git add .github/workflows/ros2-ci.yml README.md
git commit -m "Update CI workflow and README"
git push
```

4. Create a release (tag and push tag):
```bash
git tag -a v0.1.0 -m "Initial release for reproducible experiments"
git push origin v0.1.0
# Then create a GitHub Release from the tag via the UI or gh CLI
```

---

## Contributing

We welcome contributions. Recommended flow:
1. Fork the repository (if external)
2. Create a branch for your feature/fix
3. Make changes and add tests if appropriate
4. Push your branch and open a Pull Request
5. Add a clear description of the change, test results, and any instructions to reproduce

Additions to documentation (README, examples, experiment descriptions) are highly valued for research reproducibility.

---

## License & citation

Choose and include an OSI-approved license in `LICENSE` (e.g., MIT, Apache-2.0). If this repository is part of published research, include a CITATION file or a citation block in README explaining how to cite the work (BibTeX entry).

Example citation block to add to README:
```text
If you use this repository in your research, please cite:
Author(s), "Title", Conference/Journal, Year. DOI:xxxxx
```

---

## Contact & acknowledgements

- Maintainer: Your Name — your.email@example.com
- Acknowledgements: List funding sources, collaborators, libraries and toolkits used (ROS 2, Gazebo, etc.)

---

Thank you for using this repository for your research. If you want, I can:
- generate a CITATION.bib entry, or
- produce a template `experiments/README.md` describing experiment protocols (input parameters, seeds, how to capture metrics), or
- create a devcontainer/Dockerfile that installs ROS 2 Jazzy + Gazebo Harmonic for reproducible CI/dev.

```

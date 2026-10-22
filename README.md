# Mobile Manipulator Drawing in MuJoCo

A Husky-style mobile base and UR5e arm reproduce strokes drawn in a desktop panel. OCS2 coordinates base and arm motion; MuJoCo renders the robot and its drawing. The panel accepts multiple drawings in one session. After each drawing, the robot retreats to the right of the board and waits for the next command.

## Requirements

- Ubuntu 22.04, ROS 2 Humble, Python 3.10, and `python3-tk`
- NVIDIA driver and `glxinfo` (`mesa-utils`) for the MuJoCo viewer
- Pinocchio, Coal, and EigenPy for OCS2; run `./scripts/setup_local_pinocchio.sh` if they are not installed in ROS

## Setup and run

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-mujoco.txt
```

In VS Code, run the **Build OCS2 mobile manipulator** and **Build Husky UR5e description** tasks from [.vscode/tasks.json](.vscode/tasks.json). Then start the drawing panel:

```bash
./scripts/run_ocs2_vr.sh
```

Hold the left mouse button to draw a stroke, then click **Gửi cho robot** (Send to robot). Submit more drawings while the robot is working; they are queued. Click **Kết thúc / Thoát MuJoCo** (Exit) to end the session. Drawings and telemetry are saved locally in `runs/`.

| Command | Purpose |
| --- | --- |
| `./scripts/run_ocs2_vr.sh --demo-vr` | Draw the built-in “VR” trajectory |
| `./scripts/run_ocs2_vr.sh --headless --demo-vr` | Run without the viewer |
| `./scripts/run_ocs2_vr.sh --strokes-json runs/drawing.json` | Replay a saved drawing |
| `DRAW_VIEWER_FPS=15 ./scripts/run_ocs2_vr.sh` | Reduce viewer rendering load |

The viewer launch script selects the NVIDIA GeForce RTX 4050 through PRIME and stops if another OpenGL renderer is active. Headless mode skips this GPU check.

## Project layout

- [Robot description and MuJoCo scene](src/Clearpath_mobile_manipulator/)
- [OCS2 source and assets](src/ocs2_ros2/)
- [Drawing panel and simulator bridge](scripts/)

After changing the robot Xacro, rebuild the description package and run `.venv/bin/python scripts/prepare_mujoco.py` to regenerate the MuJoCo scene and OCS2 URDF. Local environments, build outputs, logs, and drawings are excluded from Git.

## Simulation scope

The pen remains about 5 cm from the board; the red trace is a projection, with no contact-force control. Lateral travel uses an idealized omni base; a physical Husky cannot strafe with its stock wheels. On a saved “SIEU” drawing, the simulated pen achieved approximately **3.25 mm RMS** tracking error.

Model and controller sources: [Husky ROS](https://github.com/husky/husky), [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description), and [OCS2 ROS 2](https://github.com/legubiao/ocs2_ros2). Upstream licenses are retained with the source.

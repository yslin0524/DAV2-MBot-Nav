# DAV2-MBot-Nav

![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.10+-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red)

**Closed-loop, single-camera obstacle avoidance for a Raspberry Pi 5 MBot, using Depth Anything V2 as the only depth sensor — no LiDAR, no GPU.**

DAV2-MBot-Nav turns a pretrained monocular depth foundation model into a working navigation stack. A single RGB camera feeds Depth Anything V2, a lightweight three-region cost-map state machine turns each depth frame into a steering decision, and a decoupled controller drives the motors — the entire pipeline runs on an ARM CPU at ~1.1 FPS.

---

## Demo



https://github.com/user-attachments/assets/964ff780-5e82-4704-879c-0903f14f712f





---

## Overview

Depth perception gates almost every part of mobile-robot navigation, but conventional depth sensors such as LiDAR and RGB-D cameras add non-trivial cost, power draw, and form-factor constraints that are awkward on small, low-cost platforms. This project asks a concrete operational question: *can a robot use only RGB and a pretrained depth foundation model, on CPU-only edge hardware, to perform closed-loop autonomous obstacle avoidance?*

Our answer pairs the released Depth Anything V2 metric checkpoint with a three-region cost-map policy and deploys the full pipeline as a three-node ROS 2 graph on a Raspberry Pi 5 MBot Classic with a single rolling-shutter RGB camera. Rather than stopping at pixel-level depth benchmarks — which do not separate safe from unsafe predictions — we evaluate the system at the decision level and with human-in-the-loop walkthroughs, and show that monocular depth can drive practical obstacle avoidance on hardware with no GPU or accelerator.

---

## System Architecture

The system takes a single RGB stream as input and produces motor commands as output. It is partitioned into four single-responsibility ROS 2 nodes connected by best-effort or reliable QoS profiles:

```
  ┌──────────────┐   /image_raw    ┌──────────────┐   /depth/image   ┌──────────────┐      /dir       ┌──────────────────┐   /cmd_vel
  │  camera_ros  │ ──────────────► │  dav2_node   │ ───────────────► │   nav_node   │ ──────────────► │  controller_node │ ──────────►  motors
  │ (libcamera)  │  640×480 BGR888 │  Depth        │   32FC1 depth    │  cost map +   │  discrete       │  symbolic → Twist │   20 Hz
  │  180° rot.   │  best-effort    │  Anything V2  │   (metres)       │  L/C/R policy │  decision       │  stale-stop guard │
  └──────────────┘                 └──────────────┘                  └──────────────┘  reliable QoS    └──────────────────┘
                                     ~1.1 FPS (CPU)                    2.0 s window
                                                                       0.5 s republish
```

Stage by stage:

- **`camera_ros`** — driver over `libcamera`; forces a 640×480 BGR888 stream and applies a 180° orientation rotation to match the model's expected input format.
- **`dav2_node`** — subscribes to `/image_raw` (best-effort QoS), runs the DAV2 metric ViT-S checkpoint at input size 518, and republishes a single-channel float32 depth image in metres (`32FC1`) on `/depth/image`. This is the dominant latency in the stack.
- **`nav_node`** — sanitises the depth frame, builds a normalised obstacle cost map, smooths it with a 31×31 elliptical kernel, discards ceiling rows above the 70th-percentile ROI, splits the field of view into left / centre / right thirds, and emits a discrete steering decision (`FORWARD` / `LEFT` / `RIGHT` / `STOP`) on `/dir`. Costs are accumulated over a 2.0 s decision window; the last decision is republished every 0.5 s so downstream always has a fresh command.
- **`controller_node`** — subscribes to `/dir`, converts the symbolic decision into `/cmd_vel` `Twist` messages at 20 Hz, and forces `STOP` if no fresh decision has arrived within the stale-message timeout.

### Steering policy

Each depth frame is turned into a decision in four steps. The per-pixel obstacle cost is

```
c(u,v) = clip( (d_safe − d(u,v)) / (d_safe − d_close), 0, 1 )
```

with `c(u,v) = 1` forced wherever `d(u,v) < d_close`, so close pixels saturate as hard obstacles (`d_close = 0.6 m`, `d_safe = 2.0 m` on the robot). After smoothing and ROI cropping, the three per-region mean costs `{c_L, c_C, c_R}` drive a state machine: `STOP` if the minimum region cost exceeds `τ_nogo = 0.6`, otherwise steer toward the lowest-cost region.

---

## Key Features

- **LiDAR-free, GPU-free navigation.** A single passive RGB camera and a pretrained depth model replace dedicated depth hardware, running end-to-end on a Raspberry Pi 5 ARM CPU with no discrete GPU or accelerator.
- **Decoupled perception and control.** Because depth inference runs at only ~1.1 FPS, the controller runs on its own 20 Hz loop and executes the most recent cached decision. A republisher re-emits the latest symbolic command every 0.5 s, so slow inference never freezes or jerks the robot.
- **Three-region cost-map state machine.** A continuous, distance-based cost map plus left/centre/right region averaging absorbs per-pixel depth noise over thousands of pixels and a 2.0 s window — decision-level robustness rather than pixel-level precision.
- **Layered, redundant safety.** Three independent fail-safes force zero velocity: stale-perception timeout, shutdown zero-Twist publish, and an all-regions-blocked `STOP`. Any single failure lands the robot in a safe state rather than the last-commanded one.
- **Validated beyond pixel metrics.** Evaluated at pixel level (NYU-Depth V2), decision level (free-space IoU + obstacle precision/recall), and with human-in-the-loop walkthroughs against a LiDAR reference — confirming that pixel-level depth accuracy is *not* a sufficient proxy for navigation safety.

---

## Hardware & Software Requirements

**Robot platform**

| Component | Requirement |
|---|---|
| Robot | MBot Classic, differential drive |
| Compute | Raspberry Pi 5 (ARM CPU, no GPU required) |
| Camera | Raspberry Pi Camera Module v1.3 (OV5647 rolling-shutter sensor) |
| Camera driver | `camera_ros` over `libcamera` |
| Stream | 640×480, BGR888, 180° orientation |

**Software**

| Component | Requirement |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy (`rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `cv_bridge`) |
| Python | 3.10+ |
| DL stack | PyTorch, TorchVision, OpenCV, NumPy |
| Model | Depth Anything V2 metric ViT-S, fine-tuned on Hypersim (`max_depth = 20 m`) |

> A GPU is **not** required to run the robot — the whole pipeline is CPU-only by design. A CUDA GPU is helpful only for the off-robot NYU-Depth V2 evaluation scripts.

---

## Installation

The ROS 2 package expects the Depth Anything V2 code and checkpoint to live in a sibling repo at `~/DA-V2-for-RobotNav` (this is where `dav2_node` inserts its import path and resolves the checkpoint).

```bash
# 1. Clone both repositories side by side in your home directory.
#    DA-V2-for-RobotNav is our team fork of Depth-Anything-V2; the evaluation
#    scripts used for the results below live on the yslin/evaluate-metrics branch.
cd ~
git clone https://github.com/sy3da/DA-V2-for-RobotNav.git
git clone https://github.com/yslin0524/DAV2-MBot-Nav.git

# 2. Install the depth model's Python dependencies
cd ~/DA-V2-for-RobotNav
pip install -r requirements.txt          # torch, torchvision, opencv-python, matplotlib, ...

# 3. Download the metric ViT-S checkpoint into checkpoints/
#    (Depth-Anything-V2-Small, metric / Hypersim)
mkdir -p checkpoints
# place depth_anything_v2_metric_hypersim_vits.pth in ~/DA-V2-for-RobotNav/checkpoints/

# 4. Build the ROS 2 workspace
cd ~/DAV2-MBot-Nav/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # cv_bridge, camera_ros, etc.
colcon build --symlink-install
source install/setup.bash
```

Make sure `camera_ros` (the `libcamera`-based ROS 2 camera driver) is installed and that the Raspberry Pi camera is enabled before launching.

---

## Usage

Launch the full four-node pipeline (camera → depth → policy → controller) with a single command:

```bash
cd ~/DAV2-MBot-Nav/ros2_ws
source install/setup.bash
ros2 launch dav2_nav pipeline.launch.py
```

All tunable parameters live in `ros2_ws/src/dav2_nav/config/nav_params.yaml` — depth `max_depth` and `input_size`, cost-map thresholds (`close_thresh_m`, `safe_dist_m`, `nogo_thresh`), ROI crop, decision window, republish period, and controller speeds / stale timeout. Override at launch, for example:

```bash
ros2 launch dav2_nav pipeline.launch.py   # then adjust nav_params.yaml and rebuild, or:
ros2 run dav2_nav nav_node --ros-args -p nogo_thresh:=0.5 -p safe_dist_m:=2.5
```

Inspect the pipeline live:

```bash
ros2 topic echo /dir           # symbolic steering decisions
ros2 topic echo /cmd_vel       # motor commands
ros2 topic hz /depth/image     # confirm ~1.1 FPS depth throughput
```

**Off-robot depth / decision evaluation** (no robot required). Run the navigation policy over a folder of images:

```bash
cd ~/DA-V2-for-RobotNav
python nav.py --img-path <folder> --encoder vits --nav --outdir vis_depth
```

To reproduce the pixel-level and decision-level metrics reported below, check out the evaluation branch of the team fork, which adds `prepare_eval_data.py`, `evaluate_metrics.py`, and the Colab notebook `evaluate_metrics_colab.ipynb`:

```bash
cd ~/DA-V2-for-RobotNav
git checkout yslin/evaluate-metrics
python prepare_eval_data.py      # convert NYU-D / KITTI GT depth to the eval format
python evaluate_metrics.py       # free-space IoU + obstacle precision/recall
```

---

## Results

All numbers are from the accompanying ROB 599 study.

**Decision-level metrics** — NYU-Depth V2 held-out validation, *n = 100*, threshold τ = 0.4 on normalised inverse depth:

| Metric | Value |
|---|---|
| Obstacle Recall ↑ (safety-critical) | **0.831** |
| Obstacle Precision ↑ | **0.889** |
| Free-Space IoU ↑ | **0.566** |

**Human-in-the-loop walkthroughs** — 5 routes (~30 s each), *n = 148* decision frames:

| Metric | Value |
|---|---|
| Steering Accuracy vs. Human ↑ | **78.4 %** |
| Collision-Rate Proxy (DAV2 vs. LiDAR) ↓ | **1.4 %** |
| Route Success Rate ↑ | **4 / 5 (80 %)** |

**On-robot throughput** — DAV2 ViT-S inference dominates latency; the policy and controller are sub-millisecond:

| Component | Value |
|---|---|
| DAV2 inference rate | ~1.1 FPS (CPU only) |
| Decision window | 2.0 s |
| Republish period | 0.5 s |
| Control loop | 20 Hz |
| Stale-decision timeout | 5.0 s |

**Key findings.** A free-space IoU of 0.566 is a poor *pixel-level* result, yet the same model reaches 78.4 % agreement with human steering and only 1.4 % disagreement with a LiDAR-driven policy — decision-level averaging absorbs per-pixel error, confirming that pixel accuracy alone does not predict navigation safety. The precision/recall profile is conservative in the right direction (rarely calls an obstacle "free"). The dominant residual failure mode is a **white-wall bias**: on high-albedo, low-texture surfaces the model underestimates distance and the policy stops in front of unobstructed walls — a structural bias that spatial smoothing cannot fix, and the main driver of the missing 20 % route completion.

---

## Citation

If you use this work, please cite:

```bibtex
@inproceedings{reza2026dav2mbot,
  title     = {Can a Single Camera Replace LiDAR? Depth Anything V2 for
               Closed-Loop Robot Navigation on CPU-Only Edge Hardware},
  author    = {Reza, Syeda and Lin, Ethan and Cleeman, Darren},
  booktitle = {ROB 599 Final Project, University of Michigan},
  year      = {2026}
}
```

This project builds on Depth Anything V2:

```bibtex
@article{yang2024depthv2,
  title   = {Depth Anything V2},
  author  = {Yang, Lihe and Kang, Bingyi and Huang, Zilong and Zhao, Zhen
             and Xu, Xiaogang and Feng, Jiashi and Zhao, Hengshuang},
  journal = {arXiv:2406.09414},
  year    = {2024}
}
```

---

## Acknowledgments

The authors thank the ROB 599 teaching staff at the University of Michigan for project guidance, and the Depth Anything V2 authors for releasing the pretrained checkpoints used in this work.

## License

MIT — see the `dav2_nav` package manifest. Depth Anything V2 and its checkpoints are subject to their own upstream licenses.

# Motion Planning for Active Viewpoint Selection in Reconstruction-Based Grasping

A robot-manipulator pipeline that builds a **3D model of an object** by moving a wrist-mounted RGBD camera along a smooth, constrained path around the object, capturing views from multiple angles, and stitching them into a single point cloud. Better 3D models give better grasps — so the focus is on planning a **continuous, collision-aware path that keeps the camera pointed at the object** throughout the motion.

Final project for **RBE 550: Motion Planning, WPI**, in collaboration with the [ELPIS Lab](https://elpislab.org/). Team project.

---

## Overview

To reconstruct an object, the manipulator follows a **semicircular workspace path** around it while a gripper-mounted **Intel RealSense D435** captures RGB-D images from many viewpoints. The end-effector is constrained to stay oriented toward the object so every view is usable. The captured point clouds are then stitched with **Iterative Closest Point (ICP)** into one 3D model.

The core challenge is **planning a smooth path**: a kinematically redundant arm maps each workspace point to many joint configurations, and a naive inverse-kinematics approach produces discontinuous, jumpy joint motions. The project explores two ways to get a continuous path, and uses the resulting motion to drive 3D reconstruction.

## Approach

Three parts, built by the team:

1. **MoveIt motion planning (ROS2)** — a UR10 arm is driven through the semicircular capture trajectory using MoveIt's `MoveGroupInterface` and `computeCartesianPath`, which interpolates Cartesian waypoints along the path and solves IK for each while bounding the joint "jump" between consecutive points. *(see `ur_ws/` — the `hello_moveit` and `ur10_moveit_config` packages.)*
2. **Expansion-GRR path planning** — Global Redundancy Resolution (Expansion-GRR) is used to generate **smooth, continuous, injective** configuration-space paths for the UR10, demonstrated in PyBullet and on the real robot. Because the task is 6-DOF (non-redundant), the workspace path is shifted around the object until continuous solutions are found.
3. **Perception & 3D reconstruction** — RGB-D images captured along the path are converted to point clouds and stitched into a 3D object model using **ICP** (with Open3D / PCL). Keeping the camera pointed at the object the whole time yields denser, more precise reconstructions.

## Results

- The UR10 followed the semicircular capture path and the end-effector stayed correctly oriented toward the object throughout, verified by checking joint continuity between successive steps.
- **Expansion-GRR** produced smooth paths in both task and configuration space and was demonstrated **on the real robot** as well as in simulation.
- The captured views were stitched into a combined 3D point-cloud model of the object (e.g. the YCB banana), suitable for downstream grasp analysis.

## Repository structure

| Path | Description |
|------|-------------|
| `ur_ws/` | **ROS2 / MoveIt** workspace — UR10 description, `ur10_moveit_config`, and the `hello_moveit` package (Cartesian-path planning for the capture trajectory). |
| `Expansion-GRR/` | Expansion-GRR planner and PyBullet demos used to generate smooth configuration-space paths. |
| `main.py`, `bullet_camera.py` | Pipeline entry point and the simulated/real RGBD camera capture. |
| `stitcher.py` | Point-cloud stitching (ICP) into a single 3D model. |
| `011_banana/` | Example YCB object model and reconstructed point clouds. |
| `*traj*.txt` | Generated workspace / configuration-space trajectory data. |
| `environment.yml` | Conda environment. |

## Running it

```bash
# 1) create the environment
conda env create -f environment.yml

# 2) MoveIt side (ROS2): build and launch the UR10 MoveIt workspace
cd ur_ws && colcon build && source install/setup.bash
ros2 launch ur10_moveit_config demo.launch.py
# then run the hello_moveit node to execute the capture trajectory

# 3) Expansion-GRR / perception: run the PyBullet demo and the reconstruction pipeline
python main.py
```
*(See `Expansion-GRR/README.md` for that module's own setup.)*

## My contributions

This was a team project (5 members). My main contribution was the **MoveIt implementation (ROS2)** — setting up the UR10 MoveIt configuration and using `computeCartesianPath` to generate and execute the semicircular capture trajectory, including tuning the end-effector step size and jump threshold and analyzing where this Cartesian approach produced discontinuous trajectories (which motivated the team's move to Expansion-GRR). I also worked on understanding and integrating **Expansion-GRR** for smooth path generation.

## Acknowledgements

- [Expansion-GRR](https://arxiv.org/abs/2405.13770) — Zhong, Li & Chamzas, *"Expansion-GRR: Efficient Generation of Smooth Global Redundancy Resolution Roadmaps,"* IROS 2024.
- [YCB Object and Model Set](https://www.ycbbenchmarks.com/) — Calli et al.
- Built in collaboration with the [ELPIS Lab](https://elpislab.org/), WPI.

---

**Laasya Kodandaramu** · [GitHub](https://github.com/laasyak02) · [LinkedIn](https://www.linkedin.com/in/laasya-kodandaramu-085a13234/)

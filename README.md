[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/01QzAdHm)
# MoveIt 2 Planning — Ex10–Ex11

**Target robot:** OpenMANIPULATOR‑X (OMX)  
**Pre-req:** ROS 2 workspace builds, Ex1–9 complete, basic ROS 2/MoveIt familiarity.

---

## Quick Legend & Assumptions

- **Frames:** `base_link` is the world origin for the arm.  
- **Groups:** `arm` (joints 1–4), `gripper` (finger joint).  
- **EE link:** `gripper_link` (or your end-effector link name).  
- **Named poses (SRDF):** `home`, `ready`, `pick`, `place`.  
- **Safety:** Keep velocity/accel scaling ≤ **0.5** in all demos.

> If your group or link names differ, **search & replace** in the code and YAML before building.

---

# Exercise 10 — MoveIt 2 Quickstart (Plan & Execute to Named Poses)

### Objective
Bring up MoveIt 2 for OMX and **plan+execute** to SRDF **named poses** with collision checking.

### What you’ll build
- A `openmanipulator_x_moveit_config/` package (URDF→SRDF via Setup Assistant).
- A bring-up launch that starts **RViz + move_group + controllers**.
- A tiny demo node to move to `home → pre_grasp/ready → pick → place`.

### Do it (checklist)
1. **Setup Assistant**  
   - Load OMX **URDF/Xacro**; create SRDF groups and **named poses**.  
   - Export **kinematics.yaml**, **joint_limits.yaml**, **ompl_planning.yaml**.
2. **Controllers**  
   - Map joints correctly in `controllers.yaml` (trajectory controller for `arm`; position controller for `gripper`).  
   - Verify `ros2_control` hardware/sim adapter loads without warnings.
3. **Bring-up**  
   - Launch `demo.launch.py` (or your `bringup_moveit.launch.py`) and open the **MotionPlanning** panel.  
   - Confirm TF tree, robot state, and planning scene render properly.
4. **Plan & Execute**  
   - In RViz: select **Planning** group `arm`, set **Goal State** to a **named pose**, click **Plan** then **Execute**.  
   - Repeat for `home`, `ready`, `pick`, `place`.

### You’re done when… (acceptance tests)
- From small random joint perturbations (±0.05 rad) you **plan+execute** to all four named poses **reliably**.  
- No self‑collision or world collision (add a thin **table** as a fixed collision object if needed).  
- Trajectories respect joint **velocity/accel limits** (see RViz trajectory display).

### Common pitfalls → fixes
- **Group mismatch**: `MoveGroupInterface("arm")` must match SRDF group.  
- **Controller won’t follow**: joint order in `controllers.yaml` must match URDF.  
- **IK failures**: increase planning time (e.g., 5s), verify kinematics solver entry.

### Submission
- `openmanipulator_x_moveit_config/` + a one‑command bring‑up, and a short **demo** that moves through **two** named poses programmatically.

---

# Exercise 11 — PlanningScene + Attach/Detach (Table & Cube)

### Objective
Use **PlanningScene** to add a **table** and a **cube**; plan collision‑free motions; **attach** the cube during grasp so it travels with the gripper; **detach** at place.

### What you’ll build
- A Python package `omx_moveit_demos/` with one node: `ex11_planning_scene_attach`.

### Do it (sequence)
1. **Seed world**  
   - Add a **table** (box) under `base_link`.  
   - Add a **cube** at the intended **pick** pose.
2. **Approach**  
   - Move to `home → ready → pick` using SRDF **named poses**.
3. **Grasp & attach**  
   - Close gripper (or just simulate), then **attach** the cube to `ee_link`.  
   - Allow collisions only between **gripper↔cube** (touch links), not the arm.
4. **Transfer & place**  
   - Plan to `place` and **execute** (the attached cube should follow).  
   - **Detach** at place (leave cube in scene at the placed pose).  
   - Return to `home`.

### You’re done when… (acceptance tests)
- The cube **moves with the gripper** while attached and returns to world on detach.  
- RViz shows **no collisions** with the table during approach/retreat.  
- Named‑pose motions succeed from consistent initial states.

### Troubleshooting
- **Object not added**: check `header.frame_id` and wait for scene sync.  
- **Attach fails**: wrong object id or link name; ensure the cube exists first.  
- **Planner refuses path**: inflate table size slightly; raise approach height.

### Submission
- `omx_moveit_demos/` with launch to include your Ex10 bring‑up + the **Ex11** node and a short GIF/PNG of RViz showing attach/detach in action.

---


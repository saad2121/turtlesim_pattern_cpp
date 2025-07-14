# ROS 2 Project: turtlesim_pattern_cpp

This ROS 2 package uses turtlesim to:
- Draw a square with turtle1
- Make turtle2 follow turtle1

## How to build
```bash
cd ~/ros2_ws
colcon build --packages-select turtlesim_pattern_cpp
source install/setup.bash
```
## How to Launch
```bash
ros2 launch turtlesim_pattern_cpp draw_follow_launch.xml
```
## 📌 PID Controller for Trajectory Tracking

**Given:**
- Desired trajectory pose: (x_d, y_d, θ_d)
- Current pose: (x, y, θ)
- Goal: compute:
  - `v`: linear velocity (robot forward x-direction)
  - `ω`: angular velocity around z-axis

---

### ✅ Step 1: Kinematic model (unicycle / differential drive)

x_dot = v * cos(θ)

y_dot = v * sin(θ)

θ_dot = ω

---

### ⚙ Step 2: Compute tracking errors

**In global frame:**
e_x = x_d - x

e_y = y_d - y

e_theta = θ_d - θ

**Transform to robot frame:**
e_xR = cos(θ) * e_x + sin(θ) * e_y

e_yR = -sin(θ) * e_x + cos(θ) * e_y

---

### 🎛 Step 3: PID controllers

v = Kp_v * e_xR + Ki_v * integral(e_xR) + Kd_v * derivative(e_xR)

ω = Kp_ω * e_theta + Ki_ω * integral(e_theta) + Kd_ω * derivative(e_theta) + Kp_lat * e_yR

---

### ✏ Step 4: Practical loop at each control step

1. Measure current pose `(x, y, θ)`
2. Get desired pose `(x_d, y_d, θ_d)`
3. Compute errors:
e_x = x_d - x

e_y = y_d - y

e_theta = θ_d - θ

4. Transform to robot frame:

e_xR = cos(θ) * e_x + sin(θ) * e_y

e_yR = -sin(θ) * e_x + cos(θ) * e_y

5. Compute control inputs:

v = Kp_v * e_xR + Ki_v * integral(e_xR) + Kd_v * derivative(e_xR)

ω = Kp_ω * e_theta + Ki_ω * integral(e_theta) + Kd_ω * derivative(e_theta) + Kp_lat * e_yR

---

## ✅ Final summary

e_x = x_d - x

e_y = y_d - y

e_theta = θ_d - θ

e_xR = cos(θ) * e_x + sin(θ) * e_y

e_yR = -sin(θ) * e_x + cos(θ) * e_y

v = Kp_v * e_xR + Ki_v * integral(e_xR) + Kd_v * derivative(e_xR)

ω = Kp_ω * e_theta + Ki_ω * integral(e_theta) + Kd_ω * derivative(e_theta) + Kp_lat * e_yR

---

## ✒ Notes
- Tune gains `Kp`, `Ki`, `Kd` experimentally.
- `v` handles forward (longitudinal) error; `ω` corrects heading and lateral deviation.


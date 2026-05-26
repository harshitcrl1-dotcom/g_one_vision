# G1 Vision - Sim-to-Sim Object Detection & Avoidance

Bridges Unitree G1 humanoid robot MuJoCo simulation to RViz2 for real-time visualization, obstacle detection and avoidance.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-3.x-green)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange)

---

## What This Does

- Streams G1 joint states from MuJoCo physics sim → RViz2 in real time
- Publishes full base TF (position + orientation) using IMU + odometry
- Renders complete office environment as RViz markers
- Detects obstacles via geometric proximity (CLEAR → WARN → STOP)

---

## System Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- MuJoCo 3.x
- CMake 3.8+

---

## Step 1 — Install ROS2 Humble

If not already installed:
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher \
  ros-humble-tf2-tools ros-humble-xacro -y
```

---

## Step 2 — Clone This Repo

```bash
cd ~
git clone https://github.com/harshitcrl1-dotcom/g_one_vision.git
cd g_one_vision
```

---

## Step 3 — Install Dependencies

### 3a. Unitree SDK2
```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake ..
sudo make install
```

### 3b. unitree_ros2
```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-ignore \
  b2_description b1_description b2w_description \
  a1_description aliengo_description aliengoZ1_description \
  go1_description go2_description go2w_description \
  laikago_description h1_description
source install/setup.bash
```

### 3c. unitree_mujoco
```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4
```

### 3d. unitree_rl_mjlab (for robot movement)
```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git
cd unitree_rl_mjlab
pip install -r requirements.txt --break-system-packages
```

---

## Step 4 — Setup This Package

```bash
cd ~/g_one_vision

# Copy custom office scene to mujoco
cp mujoco_scenes/scene_29dof.xml ~/unitree_mujoco/unitree_robots/g1/

# Copy fixed URDF (absolute mesh paths for RViz)
cp urdf/g1_29dof_fixed.urdf \
   ~/unitree_ros2/src/unitree_ros/robots/g1_description/

# Copy and build custom ROS2 package
cp -r src/g1_joint_state_bridge ~/unitree_ros2/src/

source /opt/ros/humble/setup.bash
cd ~/unitree_ros2
colcon build --packages-select g1_joint_state_bridge
source install/setup.bash
```

---

## Step 5 — Add Aliases

Add to `~/.bashrc` for easy launching:
```bash
cat >> ~/.bashrc << 'ALIASES'
# G1 Vision aliases
alias g1_source='source /opt/ros/humble/setup.bash && source ~/unitree_ros2/install/setup.bash'
alias g1_mujoco='cd ~/unitree_mujoco/simulate/build && ./unitree_mujoco'
alias g1_urdf='ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/unitree_ros2/src/unitree_ros/robots/g1_description/g1_29dof_fixed.urdf)"'
alias g1_bridge='~/unitree_ros2/install/g1_joint_state_bridge/lib/g1_joint_state_bridge/g1_joint_state_bridge'
alias g1_scene='~/unitree_ros2/install/g1_joint_state_bridge/lib/g1_joint_state_bridge/g1_scene_publisher'
alias g1_detector='~/unitree_ros2/install/g1_joint_state_bridge/lib/g1_joint_state_bridge/g1_obstacle_detector'
alias g1_alerts='ros2 topic echo /obstacle_alert'
ALIASES
source ~/.bashrc
```

---

## Step 6 — Launch Everything

Open 6 terminals, run in order:

| Terminal | Command | What it does |
|----------|---------|--------------|
| T1 | `g1_mujoco` | Starts MuJoCo physics simulation |
| T2 | `g1_source && g1_urdf` | Publishes robot URDF description |
| T3 | `g1_source && g1_bridge` | Bridges joint states + TF to ROS2 |
| T4 | `g1_source && g1_scene` | Publishes office scene markers |
| T5 | `g1_source && g1_detector` | Starts obstacle proximity detection |
| T6 | `rviz2` | Opens RViz visualizer |

### For robot movement (optional):
```bash
# T7 - after T1 is running
cd ~/unitree_rl_mjlab
python3 deploy/deploy_mujoco.py configs/g1.yaml
```

---

## Step 7 — RViz Configuration

1. Set **Fixed Frame** → `odom`
2. Add → **RobotModel** → Description Topic: `/robot_description`
3. Add → **TF**
4. Add → **MarkerArray** → Topic: `/scene_markers`
5. Add → **MarkerArray** → Topic: `/obstacle_markers`

---

## Step 8 — Monitor Obstacle Alerts

```bash
ros2 topic echo /obstacle_alert
```

Output:
data: CLEAR           # robot far from all obstacles
data: WARN: conf_table dist=1.2m   # getting close
data: STOP: conf_table dist=0.5m   # too close

---

## How It Works

### Data Flow
MuJoCo Physics Simulation
│
├──► /lowstate (joints + IMU) ──────────────────────────────────┐
└──► /sportmodestate (x,y,z position) ──────────────────────┐  │
│  │
g1_joint_state_bridge
│
┌────────────────────────────────┘
├──► /joint_states (29 joint angles)
└──► /tf (odom → pelvis)
│
robot_state_publisher
│
/tf (all 40+ links)
│
┌─────────────────────┤
│                     │
g1_obstacle_detector     g1_scene_publisher
│                     │
/obstacle_alert          /scene_markers
/obstacle_markers
│
RViz2

### Nodes Explained
| Node | Subscribes | Publishes | Purpose |
|------|-----------|-----------|---------|
| `g1_joint_state_bridge` | `/lowstate`, `/sportmodestate` | `/joint_states`, `/tf` | Translates Unitree DDS → ROS2 |
| `g1_scene_publisher` | — | `/scene_markers` | Renders office environment |
| `g1_obstacle_detector` | `/tf` | `/obstacle_alert`, `/obstacle_markers` | Proximity detection |

---

## Roadmap

- [x] Plan B: Geometric proximity detection
- [ ] Plan C: Simulated LiDAR raycasting (mj_ray)
- [ ] Plan A: Depth camera pointcloud (D435)

---

## Troubleshooting

**Robot not showing in RViz:**
```bash
ros2 topic echo /robot_description --once | head -3
# Should output URDF xml
```

**Joint states not publishing:**
```bash
ros2 topic hz /joint_states
# Should show ~900 Hz
```

**TF broken:**
```bash
ros2 run tf2_tools view_frames
```

**MuJoCo not connecting to ROS2:**
- Check domain ID matches: `config.yaml` should have `domain_id: 0`
- Check interface: `interface: "lo"`

---

## License
MIT

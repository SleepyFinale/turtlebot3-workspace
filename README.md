# TurtleBot3 Workspace - Autonomous Exploration Setup

This workspace contains editable TurtleBot3 packages for ROS 2 Humble, configured for autonomous exploration and mapping with Nav2, SLAM Toolbox, and Explore Lite.

## Quick Start

**IMPORTANT**: Make sure `ROS_DOMAIN_ID` is set (see Prerequisites above). If not set, run:
```bash
export ROS_DOMAIN_ID=30
```

**On Robot:**
```bash
export ROS_DOMAIN_ID=30  # If not in ~/.bashrc
ros2 launch turtlebot3_bringup robot.launch.py
```

**On Computer (4 terminals, start in order, wait between steps):**

**Setup (run in each terminal):**
```bash
export ROS_DOMAIN_ID=30  # If not in ~/.bashrc
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

1. **SLAM Toolbox** (wait 20-30s):
   ```bash
   ros2 launch slam_toolbox online_async_launch.py
   ```

2. **Nav2** (wait 20-30s after SLAM):
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py
   ```

3. **Explorer**:
   ```bash
   cd ~/turtlebot3_ws
   source install/setup.bash
   ./start_explorer_simple.sh
   ```

**Total startup time**: ~60-90 seconds from start to exploration.

**Having issues?** See [Troubleshooting](#troubleshooting) section below.

## Prerequisites

Before proceeding, ensure you have the following installed:

```bash
sudo apt update
sudo apt install git
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

### Important: Set ROS_DOMAIN_ID

**CRITICAL**: ROS 2 uses `ROS_DOMAIN_ID` to separate different robot networks. Both your robot and computer must use the **same** `ROS_DOMAIN_ID` value, otherwise they won't see each other's topics.

**Set it once (recommended - add to `~/.bashrc`):**

**On Robot:**
```bash
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc
```

**On Computer:**
```bash
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc
```

**Or set it manually in each terminal session:**
```bash
export ROS_DOMAIN_ID=30
```

**Verify it's set:**
```bash
echo $ROS_DOMAIN_ID
```
Should show `30` (or your chosen value).

**Note**: If `ROS_DOMAIN_ID` is not set, it defaults to 0. Make sure both robot and computer use the same value!

## Setup Instructions

### 1. Clone the TurtleBot3 Packages

Navigate to the src directory and clone the required packages:

```bash
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
```

### 2. Remove Git Directories from Cloned Packages

To make these packages part of your single repository (instead of submodules), remove their .git directories:

```bash
cd ~/turtlebot3_ws/src
rm -rf DynamixelSDK/.git
rm -rf turtlebot3_msgs/.git
rm -rf turtlebot3/.git
```

### 3. Build the Workspace

Install colcon build tools if needed:
```bash
sudo apt install python3-colcon-common-extensions
```

Build the workspace:
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Build Explore Lite

If you want to use Explore Lite for autonomous exploration, build it:

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select explore_lite --packages-ignore nav2_msgs nav2_voxel_grid nav2_util nav2_lifecycle_manager nav2_map_server nav2_costmap_2d nav2_ros_common navigation2
source install/setup.bash
```

Or use the provided build script:
```bash
./BUILD_EXPLORE_LITE.sh
```

---

# Autonomous Exploration Guide

**After your robot is running** (`ros2 launch turtlebot3_bringup robot.launch.py` on the robot), run these commands on your **central computer** in **separate terminals**:

## Setup (Run once per terminal session)

**IMPORTANT**: Set `ROS_DOMAIN_ID` first (must match robot's value):

```bash
export ROS_DOMAIN_ID=30  # Use same value as robot!
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

**To make permanent** (so you don't need to set it each time):
```bash
echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
source ~/.bashrc
```

**Verify it's set:**
```bash
echo $ROS_DOMAIN_ID
```
Should show `30` (or your chosen value).

---

## Terminal 1: SLAM Toolbox (Mapping)

**Purpose**: Creates the map as the robot explores

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch slam_toolbox online_async_launch.py
```

**Alternative**: If you prefer Cartographer (your old setup):
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py configuration_basename:=turtlebot3_lds_2d_improved.lua
```

---

## Terminal 2: Nav2 (Navigation)

**Purpose**: Provides navigation and path planning capabilities

**Option A: TurtleBot3 Nav2 (Recommended - includes RViz)**
```bash
# First install nav2_bringup if not already installed:
sudo apt install ros-humble-nav2-bringup

# Then launch:
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```
*Note: This launches both Nav2 and RViz. If you use this, you can skip Terminal 4.*

**Option B: Standard Nav2 (if nav2_bringup is installed)**
```bash
sudo apt install ros-humble-nav2-bringup  # Install first if needed
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup navigation_launch.py
```

---

## Terminal 3: Explorer (Autonomous Exploration)

**Purpose**: Decides where to explore next and sends goals to Nav2

**IMPORTANT**: Start this AFTER Nav2 is running and initialized (wait 20-30 seconds after starting Nav2).

**Option A: Explore Lite with Nav2 Costmap (Recommended)**
```bash
cd ~/turtlebot3_ws
./start_explorer_simple.sh
```

This uses Nav2's costmap which includes inflation for better obstacle avoidance.

**Option B: Explore Lite with SLAM Map (Alternative)**
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run explore_lite explore --ros-args \
    --params-file src/m-explore-ros2/explore/config/params.yaml
```

This uses the SLAM map directly (may initialize faster if Nav2 is slow).

**Option C: Your Custom Explorer**
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run custom_explorer explorer
```

**Note**: The explorer will wait for the costmap automatically. You'll see:
```
[INFO] [explore_node]: Waiting for costmap to become available...
```
Once the costmap is ready, it will automatically start exploring.

---

## Terminal 4: RViz (Visualization - Optional but Recommended)

**Purpose**: Visualize the map, robot position, and exploration progress

**Option A: Using TurtleBot3 Navigation2 (Recommended)**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```
*Note: This launches both Nav2 and RViz together*

**Option B: Launch RViz directly**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run rviz2 rviz2
```
*Then manually add displays: Map, TF, LaserScan, RobotModel*

**Option C: Install nav2_bringup (if you want the standard Nav2 RViz)**
```bash
sudo apt install ros-humble-nav2-bringup
```
*Then you can use: `ros2 launch nav2_bringup rviz_launch.py`*

---

## Quick Reference: Complete Startup Sequence

**CRITICAL**: Start in this exact order and wait between steps. Timing is important!

### Step 1: Start Robot (On Robot)

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Verify it's working:**
- Check topics: `ros2 topic list | grep -E "(scan|odom|joint_states)"`
- Check scan: `ros2 topic echo /scan --once` (should show laser data)
- Check odometry: `ros2 topic echo /odom --once` (may need robot to move first)

**Note**: If `/odom` isn't publishing, try moving the robot slightly:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

### Step 2: Start SLAM Toolbox (On Computer - Terminal 1)

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch slam_toolbox online_async_launch.py
```

You should see:
```
[INFO] [slam_toolbox]: Node using stack size 40000000
[INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[INFO] [slam_toolbox]: Registering sensor: [Custom Described Lidar]
```

**Wait 20-30 seconds** for SLAM to initialize and start publishing the map.

**What's happening:**
- SLAM Toolbox is receiving scan data and processing it
- It needs several scan messages before it can create and publish a map
- The `/map` topic won't appear until SLAM has processed enough data

**Verify it's working:**
- Check scan is being received: `ros2 topic echo /scan --once` (should show data)
- Wait 20-30 seconds, then check `/map` topic: `ros2 topic list | grep "^/map$"`
- Once `/map` appears, check it's publishing: `ros2 topic echo /map --once` (may need to wait a few more seconds)
- Check TF: `ros2 run tf2_ros tf2_echo map odom` (should show transform after map is published)

**If map isn't publishing after 30 seconds:**
- Make sure robot is providing scan data: `ros2 topic hz /scan` (should show ~10 Hz)
- Try moving robot slightly to help SLAM initialize:
  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
  ```
- Wait longer (SLAM needs several scan messages to start)
- Check SLAM node is running: `ros2 node list | grep slam`

### Step 3: Start Nav2 (On Computer - Terminal 2)

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

**Wait 20-30 seconds** for Nav2 to fully initialize.

**Verify it's working:**
- Check Nav2 nodes: `ros2 node list | grep nav2`
- Check costmap: `ros2 topic list | grep costmap`
- Check TF tree: `ros2 run tf2_ros tf2_echo map base_link` (should work without errors)

**If you get TF errors:**
- Make sure SLAM Toolbox is running and has initialized (Step 2)
- Wait longer - Nav2 needs the map and TF tree to be ready
- Check: `ros2 run tf2_ros tf2_monitor` (should show map, odom, base_link frames)

### Step 4: Start Explorer (On Computer - Terminal 3)

```bash
cd ~/turtlebot3_ws
./start_explorer_simple.sh
```

Or manually:
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run explore_lite explore --ros-args \
    --params-file src/m-explore-ros2/explore/config/params_costmap.yaml
```

**The explorer will wait for the costmap automatically.** You'll see:
```
[INFO] [explore_node]: Waiting for costmap to become available, topic: /global_costmap/costmap
```

Once Nav2's costmap is ready (usually 20-40 seconds after Nav2 starts), the explorer will automatically connect and start exploring.

**Verify it's working:**
- Wait for message: `[INFO] [explore_node]: Exploration started`
- Robot should start moving autonomously
- Check goals being sent: `ros2 topic echo /goal_pose`

### Step 5: RViz (Optional - Already Included)

If you used `turtlebot3_navigation2 navigation2.launch.py` in Step 3, RViz is already running. Otherwise, launch it separately.

---

## Expected Timeline

| Step | Component | Time to Start | Time to Ready |
|------|-----------|--------------|---------------|
| 1 | Robot | Immediate | Immediate |
| 2 | SLAM Toolbox | Immediate | 20-30 seconds |
| 3 | Nav2 | Immediate | 20-30 seconds (after SLAM ready) |
| 4 | Explorer | Immediate | Waits for Nav2 costmap (20-40s after Nav2) |

**Total time from start to exploration**: ~60-90 seconds

---

## What Each Terminal Does

| Terminal | Component | Purpose |
|----------|-----------|---------|
| Robot | Robot Launch | Publishes sensor data (`/scan`), odometry, TF |
| Terminal 1 | SLAM Toolbox | Builds the map from sensor data, publishes `/map` |
| Terminal 2 | Nav2 | Plans paths, avoids obstacles, executes navigation |
| Terminal 3 | Explorer | Detects frontiers, sends exploration goals to Nav2 |
| Terminal 4 | RViz | Visualizes everything in real-time |

---

## Verification Checklist

After starting all terminals, verify each step:

### After Step 1 (Robot):
- [ ] `/scan` topic exists and publishing: `ros2 topic echo /scan --once`
- [ ] `/odom` topic exists: `ros2 topic list | grep odom`
- [ ] Robot responds to commands: `ros2 topic pub --once /cmd_vel ...`

### After Step 2 (SLAM Toolbox - wait 20-30s):
- [ ] `/map` topic exists: `ros2 topic list | grep "^/map$"`
- [ ] Map is publishing: `ros2 topic echo /map --once` (wait a few seconds)
- [ ] TF frame `map` exists: `ros2 run tf2_ros tf2_monitor | grep map`

### After Step 3 (Nav2 - wait 20-30s):
- [ ] Nav2 nodes running: `ros2 node list | grep nav2`
- [ ] Costmap topics exist: `ros2 topic list | grep costmap`
- [ ] TF tree complete: `ros2 run tf2_ros tf2_echo map base_link` (no errors)
- [ ] RViz shows map and robot (if using turtlebot3_navigation2)

### After Step 4 (Explorer):
- [ ] Explorer node running: `ros2 node list | grep explore`
- [ ] Explorer started: Look for `[INFO] [explore_node]: Exploration started`
- [ ] Goals being sent: `ros2 topic echo /goal_pose` (should see goals)
- [ ] Robot is moving autonomously

### Overall System Check:
- [ ] Map is growing in RViz
- [ ] Robot is exploring autonomously
- [ ] All topics active: `ros2 topic list | grep -E "(map|scan|cmd_vel|goal_pose)"`
- [ ] TF is working: `ros2 run tf2_ros tf2_echo map base_link` (shows transform)

---

## Troubleshooting

### Common Issues and Solutions

#### 1. TF Errors: "Invalid frame ID 'odom' passed to canTransform"

**Cause**: Nav2 started before SLAM Toolbox initialized, or odometry isn't publishing.

**Solution**:
1. Stop Nav2 (Ctrl+C)
2. Verify SLAM Toolbox is running: `ros2 node list | grep slam`
3. Wait 20-30 seconds for SLAM to initialize
4. Check TF tree: `ros2 run tf2_ros tf2_echo map odom` (should work)
5. Restart Nav2

**Prevention**: Always wait 20-30 seconds after starting SLAM before starting Nav2.

#### 2. Odometry Not Publishing (`/odom` topic exists but no data)

**Cause**: Odometry needs robot movement to initialize, or parameters not loaded.

**Solution**:
1. Move robot slightly:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
   ```
2. Check odometry: `ros2 topic echo /odom --once`
3. If still not working, restart robot launch on the robot

**Check**: `ros2 topic list | grep odom` - topic should exist and be publishing data.

#### 3. Explorer Waiting for Costmap

**Cause**: Nav2 costmap hasn't initialized yet (normal - takes 20-40 seconds).

**Solution**: 
- **This is normal!** The explorer is designed to wait. Just be patient.
- The explorer will automatically connect once Nav2's costmap is ready.
- You'll see: `[INFO] [explore_node]: Exploration started` when ready.

**Alternative**: Use SLAM map directly (faster initialization):
```bash
ros2 run explore_lite explore --ros-args \
    --params-file src/m-explore-ros2/explore/config/params.yaml
```

#### 4. No Map Appearing in SLAM (`/map` topic doesn't exist or not publishing)

**Cause**: SLAM Toolbox not receiving scan data, not initialized yet, or needs more time.

**Symptoms**:
- `/map` topic doesn't appear in `ros2 topic list`
- `/map` topic exists but `ros2 topic echo /map --once` shows "does not appear to be published yet"

**Solution**:
1. **Check scan data is available**: 
   ```bash
   ros2 topic echo /scan --once
   ```
   Should show laser scan data. If not, check robot connection.

2. **Check scan frequency**:
   ```bash
   ros2 topic hz /scan
   ```
   Should show ~10 Hz (depends on lidar). If 0 Hz, scan isn't publishing.

3. **Move robot slightly** to help SLAM initialize:
   ```bash
   ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
   ```

4. **Wait longer**: SLAM needs 20-30 seconds AND several scan messages before publishing map
   - The `/map` topic won't appear until SLAM has processed enough scans
   - This is normal - be patient!

5. **Verify SLAM is running**:
   ```bash
   ros2 node list | grep slam
   ```
   Should show `async_slam_toolbox_node`

6. **Check SLAM logs** for errors in the terminal where you launched it

**Note**: It's normal for `/map` to not appear immediately. SLAM Toolbox needs to:
- Receive scan data
- Process several scans
- Build initial map
- Then start publishing `/map` topic

This typically takes 20-40 seconds from when SLAM starts.

#### 5. Robot Not Moving / Explorer Not Finding Frontiers

**Cause**: System still initializing, or map too small.

**Solution**:
1. Wait 60-90 seconds total from startup
2. Check explorer status: Should see `[INFO] [explore_node]: Exploration started`
3. Check goals: `ros2 topic echo /goal_pose` (should see goals being sent)
4. Check Nav2: `ros2 service list | grep lifecycle` (should see Nav2 services)
5. Wait for map to build - explorer needs some map data before finding frontiers

#### 6. ROS_DOMAIN_ID Mismatch (Topics Not Visible)

**Cause**: Robot and computer using different ROS_DOMAIN_ID values, or ROS_DOMAIN_ID not set.

**Symptoms**:
- Topics from robot not visible on computer (or vice versa)
- `ros2 topic list` shows different topics on robot vs computer
- Nodes can't see each other

**Solution**:
1. **Check current value on robot:**
   ```bash
   echo $ROS_DOMAIN_ID
   ```
   If empty, it defaults to 0.

2. **Check current value on computer:**
   ```bash
   echo $ROS_DOMAIN_ID
   ```

3. **Set same value on both** (use 30 if that's what you're using):
   ```bash
   export ROS_DOMAIN_ID=30
   ```

4. **Make it permanent** (so you don't need to set it each terminal):
   ```bash
   echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc
   source ~/.bashrc
   ```
   Do this on **both robot and computer**.

5. **Restart terminals** or source bashrc:
   ```bash
   source ~/.bashrc
   ```

6. **Verify both match:**
   ```bash
   # On robot
   echo $ROS_DOMAIN_ID
   
   # On computer  
   echo $ROS_DOMAIN_ID
   ```
   Both should show the same value (e.g., `30`).

**Note**: If `ROS_DOMAIN_ID` is not set, ROS 2 defaults to 0. Make sure both robot and computer explicitly set the same value!

### Diagnostic Commands

**Check what's running:**
```bash
ros2 node list                    # List all nodes
ros2 topic list                   # List all topics
ros2 topic hz /scan              # Check scan frequency
ros2 topic hz /map               # Check map publishing rate
```

**Check TF tree:**
```bash
ros2 run tf2_ros tf2_monitor      # Show all frames
ros2 run tf2_ros tf2_echo map base_link  # Check specific transform
```

**Check if topics are publishing:**
```bash
ros2 topic echo /scan --once      # Should show laser data
ros2 topic echo /odom --once      # Should show odometry
ros2 topic echo /map --once       # Should show map (wait a few seconds)
```

**Check Nav2 status:**
```bash
ros2 service list | grep lifecycle
ros2 topic list | grep costmap
```

### Quick Fix Checklist

If nothing is working, go through this checklist:

- [ ] Robot launch is running on robot
- [ ] `/scan` topic is publishing: `ros2 topic echo /scan --once`
- [ ] SLAM Toolbox is running: `ros2 node list | grep slam`
- [ ] Waited 20-30 seconds after starting SLAM
- [ ] `/map` topic exists: `ros2 topic list | grep "^/map$"`
- [ ] Nav2 is running: `ros2 node list | grep nav2`
- [ ] Waited 20-30 seconds after starting Nav2
- [ ] TF tree is complete: `ros2 run tf2_ros tf2_echo map base_link` (no errors)
- [ ] Explorer is running and waiting/started
- [ ] ROS_DOMAIN_ID matches on robot and computer: `echo $ROS_DOMAIN_ID` (should show same value on both)

**Remember**: The system needs 60-90 seconds total to fully initialize. Be patient!

---

## Workspace Structure

This workspace includes:
- **TurtleBot3 packages**: Core robot packages from ROBOTIS
- **Navigation2**: Navigation stack (can use system packages or workspace version)
- **SLAM Toolbox**: For mapping (installed via apt)
- **Explore Lite**: Autonomous exploration package (in `src/m-explore-ros2/`)
- **Custom Explorer**: Alternative exploration node (in `src/Autonomous-Explorer-and-Mapper-ros2-nav2/`)

---

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://www.turtlebot.com/)

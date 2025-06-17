# TurtleBot3 SLAM Mapping Quick Start Guide

## Complete Step-by-Step Process

### Prerequisites
- TurtleBot3 packages installed
- Gazebo simulation working
- Navigation2 and SLAM packages installed

### Step 1: Launch Gazebo Simulation

```bash
# Terminal 1: Start Gazebo simulation
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait for Gazebo to fully load with the TurtleBot3 robot visible.

### Step 2: Launch SLAM Mapping

```bash
# Terminal 2: Start SLAM mapping with Cartographer (includes RViz automatically)
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

You should see output indicating that Cartographer is running and processing laser scans. RViz will also open automatically, showing:
- Robot model
- Laser scan data (red dots)
- Gradually building map (gray/black/white areas)

### Step 3: Drive Robot to Create Map

Choose one of these teleop methods:

#### Option A: Custom Python Teleop (Recommended)
```bash
# Terminal 3: Use the custom teleop script
cd ~/ros2_ws
python3 better_teleop.py
```

#### Option B: C++ Smart Teleop
```bash
# Terminal 3: Use C++ teleop with obstacle avoidance
cd ~/ros2_ws
source install/setup.bash
ros2 run matrix_publisher teleop_control_node
```

#### Option C: Standard TurtleBot3 Teleop
```bash
# Terminal 3: Use standard teleop
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

### Step 4: Mapping Strategy

**Drive the robot systematically:**
1. Move slowly (avoid rapid movements)
2. Cover all rooms and areas
3. Drive close to walls and obstacles
4. Visit each area from multiple angles
5. Make smooth turns, avoid spinning in place
6. Watch the map build in RViz

**Good mapping pattern:**
- Start with outer perimeter
- Then explore inner areas
- Return to areas that look incomplete
- Ensure all doorways and passages are mapped

### Step 5: Save the Map

When satisfied with the map quality:

```bash
# Terminal 4: Save the map (keep SLAM running!)
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/my_turtlebot_map
```

**This creates two files:**
- `~/my_turtlebot_map.pgm` - The map image
- `~/my_turtlebot_map.yaml` - Map configuration

### Step 6: Verify Map Files

```bash
# Check that map files were created
ls -la ~/my_turtlebot_map.*

# View map metadata
cat ~/my_turtlebot_map.yaml
```

Expected output:
```yaml
image: /home/jihwanoh/my_turtlebot_map.pgm
mode: trinary
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### Step 7: Test Navigation with Saved Map

```bash
# Terminal 1: Close Gazebo and restart
# Ctrl+C in Terminal 1, then:
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch navigation with your map
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/jihwanoh/my_turtlebot_map.yaml

# Terminal 3: Launch RViz for navigation
ros2 launch turtlebot3_navigation2 rviz_launch.py
```

### Step 8: Set Initial Pose and Navigate

In RViz:
1. Click "2D Pose Estimate" button
2. Click and drag on map where robot is located
3. Click "2D Goal Pose" button
4. Click where you want robot to go
5. Watch autonomous navigation!

## Alternative: SLAM Toolbox Method

If you prefer SLAM Toolbox over Cartographer:

```bash
# Terminal 2: Use SLAM Toolbox instead (also includes RViz)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Save map with SLAM Toolbox service call
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: data: 'my_slam_map'"
```

## Troubleshooting

### Map not appearing in RViz:
```bash
# Check if map topic is being published
ros2 topic list | grep map
ros2 topic echo /map | head -5
```

### Poor map quality:
- Drive slower
- Cover areas multiple times
- Ensure robot sees walls from different angles
- Check laser scan: `ros2 topic echo /scan`

### Save command fails:
- Ensure SLAM node is still running
- Check directory permissions
- Try saving to a different location

### Navigation issues:
- Verify map file paths in launch command
- Check robot localization (green arrows in RViz)
- Ensure initial pose is set correctly

## Map Management

```bash
# Create maps directory
mkdir -p ~/maps

# Copy map to organized location
cp ~/my_turtlebot_map.* ~/maps/

# Use specific map for navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/jihwanoh/maps/my_turtlebot_map.yaml
```

## Success Indicators

✅ **Good map has:**
- Clear wall boundaries
- Proper room separation
- Minimal noise/artifacts
- Complete coverage of desired areas

❌ **Poor map shows:**
- Blurry or thick walls
- Missing areas
- Excessive noise
- Disconnected rooms

The key to good mapping is **slow, systematic exploration** while watching the map build in RViz! 
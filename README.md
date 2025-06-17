# Matrix Publisher ROS2 Package

A comprehensive ROS2 package that demonstrates matrix publishing/subscribing, service handling, and robot control using C++ and Python. This package includes TurtleBot3 simulation examples and serves as a learning resource for ROS2 development.

## Project Structure

```
matrix_publisher/
├── src/
│   ├── matrix_publisher_node.cpp    # Publisher node for matrix data
│   ├── matrix_subscriber_node.cpp   # Subscriber node for matrix data
│   ├── multiply_server_node.cpp     # Service server node for multiplication
│   ├── bridge_node.cpp             # Topic-to-service bridge node
│   ├── teleop_control_node.cpp     # C++ keyboard teleop with obstacle avoidance
│   ├── obstacle_avoider_node.cpp   # Autonomous obstacle avoidance node
│   ├── wall_follower_node.cpp      # Advanced wall following with dual modes
│   ├── pid_wall_follower_node.cpp  # PID-based wall following with advanced control
│   └── cmd_vel_publisher.cpp       # Simple velocity publisher node
├── include/
│   └── pid_controller.hpp          # Robust PID controller implementation
├── srv/
│   └── MultiplyTwoFloats.srv       # Service definition
├── better_teleop.py               # Python keyboard teleop script
├── pid_monitor_working.py         # Real-time PID performance monitor
├── CMakeLists.txt
├── package.xml
└── README.md
```

# TurtleBot3 Setup and Control

This section provides complete instructions for setting up TurtleBot3 simulation and keyboard control in ROS2 Humble.

## TurtleBot3 Prerequisites

- Ubuntu 22.04 (Jammy)
- ROS2 Humble installed
- Working ROS2 workspace (e.g., `~/ros2_ws`)

## TurtleBot3 Installation

### Step 1: Install Required Dependencies

First, fix any missing dependencies and install required packages:

```bash
# Fix any missing dependencies
sudo apt --fix-broken install

# Install TurtleBot3 packages
sudo apt update
sudo apt install ros-humble-turtlebot3* -y

# Install simulation packages
sudo apt install ros-humble-turtlebot3-simulations -y

# Install additional required packages
sudo apt install ros-humble-example-interfaces -y
sudo apt install ros-humble-rosidl-default-generators -y
```

### Step 2: Set Environment Variables

Add TurtleBot3 model to your shell configuration:

```bash
# Add to your ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
source ~/.bashrc

# Or set for current session
export TURTLEBOT3_MODEL=waffle
```

## Running TurtleBot3 Simulation

### Launch Gazebo Simulation

Open a terminal and run:

```bash
# Navigate to your workspace
cd ~/ros2_ws

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Launch Gazebo simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This will open Gazebo with a TurtleBot3 robot in a simulated world.

## TurtleBot3 Keyboard Control

### Method 1: Custom Teleop Scripts

We've created both Python and C++ keyboard control options with improved responsiveness:

#### Option A: Python Teleop (Simple & Reliable)

#### Step 1: Use the provided control script

The custom keyboard control script `better_teleop.py` is already provided in this workspace.

**Controls:**
- **W/w** - Move Forward
- **S/s** - Move Backward 
- **A/a** - Turn Left
- **D/d** - Turn Right
- **X/SPACE/0** - Stop Robot
- **Q/q** - Quit Program

#### Step 2: Run the teleop script

```bash
# Make it executable and run (make sure Gazebo is running first)
chmod +x better_teleop.py
source /opt/ros/humble/setup.bash
python3 better_teleop.py
```

#### Option B: C++ Smart Teleop (With Obstacle Avoidance)

Advanced C++ keyboard control with built-in safety features:

**Features:**
- Obstacle detection using laser scan
- Safety override prevents crashes
- RViz visualization markers
- Real-time distance feedback

**Controls:**
- `w` - Move forward (if safe)
- `a` - Turn left
- `d` - Turn right
- `s` - Stop
- `q` - Quit

**Usage:**
```bash
# Make sure Gazebo is running, then:
source install/setup.bash
ros2 run matrix_publisher teleop_control_node
```

### Method 2: Standard TurtleBot3 Teleop

If you prefer the standard teleop:

```bash
# In a new terminal (while Gazebo is running)
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard
```

**Note**: The standard teleop may have keyboard input issues in some terminal environments.

### Method 3: RQT Robot Steering (GUI)

For a graphical control interface:

```bash
# Start RQT
source /opt/ros/humble/setup.bash
rqt

# In RQT: Plugins → Robot Tools → Robot Steering
```

## Testing Manual Commands

You can also test robot movement with manual commands:

```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## TurtleBot3 Troubleshooting

### Common Issues

1. **"Package 'turtlebot3_gazebo' not found"**
   ```bash
   sudo apt install ros-humble-turtlebot3-simulations
   ```

2. **Robot doesn't move with keyboard**
   - Ensure the teleop terminal window has focus
   - Try the custom Python script (Method 1 above)
   - Test with manual commands first

3. **Gazebo doesn't start**
   - Check if Gazebo is already running: `ps aux | grep gazebo`
   - Kill existing instances: `pkill -f gazebo`
   - Restart with the launch command

4. **Missing dependencies**
   ```bash
   sudo apt install ros-humble-example-interfaces
   sudo apt install ros-humble-rosidl-default-generators
   ```

### Verification Commands

Check if everything is working:

```bash
# List available topics
ros2 topic list

# Check robot's velocity topic
ros2 topic echo /cmd_vel

# Monitor robot's odometry
ros2 topic echo /odom

# List available services
ros2 service list
```

## TurtleBot3 Navigation (Advanced)

For autonomous navigation capabilities:

```bash
# Install navigation packages
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Launch navigation (in separate terminal after Gazebo is running)
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

This enables SLAM (Simultaneous Localization and Mapping) and autonomous navigation features.

# C++ Robot Control Nodes

This package includes several C++ nodes for robot control and obstacle avoidance. These demonstrate different approaches to robot control in ROS2, including advanced PID (Proportional-Integral-Derivative) control systems for precise and smooth robot navigation.

## C++ Control Nodes Overview

### 1. Smart Keyboard Teleop (`teleop_control_node.cpp`)

Advanced keyboard control with integrated laser scan obstacle detection and visualization markers.

**Features:**
- Real-time keyboard input for robot control
- Laser scan integration for obstacle detection
- Safety override - prevents forward movement when obstacles detected
- RViz visualization markers for obstacle positions
- Separate controls for forward/backward movement and turning

**Usage:**
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run smart teleop
source install/setup.bash
ros2 run matrix_publisher teleop_control_node

# Terminal 3: Launch RViz to see markers (optional)
rviz2
```

**Controls:**
- `w` - Move forward (if path is clear)
- `a` - Turn left
- `d` - Turn right  
- `s` - Stop/brake
- `q` - Quit

**Safety Features:**
- Automatically stops if obstacle within 0.3 meters ahead
- Allows turning even when obstacle detected
- Visual markers in RViz show obstacle positions
- Real-time distance feedback in terminal

### 2. Autonomous Obstacle Avoidance (`obstacle_avoider_node.cpp`)

Fully autonomous robot that moves forward and avoids obstacles automatically.

**Features:**
- Continuous forward movement
- Automatic obstacle detection using laser scan
- Turn-in-place behavior when obstacle detected
- No user input required

**Usage:**
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run autonomous avoidance
source install/setup.bash
ros2 run matrix_publisher obstacle_avoider_node
```

**Behavior:**
- Moves forward at 0.2 m/s when path is clear
- Stops and turns right when obstacle within 0.3 meters
- Monitors 30-degree cone in front of robot
- Provides real-time feedback in terminal

### 3. Advanced Wall Follower (`wall_follower_node.cpp`)

Intelligent wall following robot with dual manual/autonomous modes and persistent navigation behavior.

**Features:**
- Dual mode operation: Manual teleop + Autonomous wall following
- Persistent wall following with memory (prevents oscillation)
- Smart obstacle avoidance with committed turning
- Adaptive wall detection and following
- Counter-clockwise preference (left wall following)
- Anti-oscillation algorithms for smooth navigation

**Usage:**
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run wall follower
source install/setup.bash
ros2 run matrix_publisher wall_follower_node
```

**Controls:**
- `m` - Toggle between Manual and Autonomous modes
- **Manual Mode:**
  - `w` - Move forward
  - `a` - Turn left
  - `d` - Turn right
  - `x` - Stop
- **Autonomous Mode:** Robot automatically follows walls

**Autonomous Behavior:**
- **Wall Detection:** Detects walls within 1.0m using laser scan
- **Wall Following:** Maintains 0.4m distance from walls
- **Persistence:** Remembers which wall to follow (left/right preference)
- **Smart Switching:** Only switches walls after 2+ seconds of wall loss
- **Obstacle Avoidance:** Aggressive turning with 3-second commitment periods
- **Search Mode:** Explores environment when no walls detected

**Advanced Algorithms:**
- **Committed Turning:** Prevents oscillation by committing to turn direction for 3 seconds
- **Wall Preference Memory:** Maintains consistent following direction (counter-clockwise preferred)
- **Proportional Control:** Smooth wall following with distance-based corrections
- **Backup Capability:** Reverses when critically close to obstacles (<0.3m)

### 4. Simple Velocity Publisher (`cmd_vel_publisher.cpp`)

Basic demonstration of publishing velocity commands to move the robot.

**Features:**
- Continuous forward movement
- Simple timer-based publishing
- Good for testing basic robot movement

**Usage:**
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run velocity publisher
source install/setup.bash
ros2 run matrix_publisher cmd_vel_publisher
```

**Behavior:**
- Publishes forward velocity (0.2 m/s) every 500ms
- Robot moves straight forward continuously
- Simple demonstration of `/cmd_vel` publishing

## PID Control Nodes (Advanced)

### 5. PID Wall Follower (`pid_wall_follower_node.cpp`)

Advanced wall following robot with sophisticated PID control systems for smooth and precise navigation.

**Features:**
- **Multiple PID Controllers:** Separate controllers for wall distance, obstacle avoidance, and speed control
- **Intelligent Mode Switching:** Automatic switching between wall following, obstacle avoidance, and exploration
- **Smooth Control:** 20Hz control loop with proper time step management
- **Real-time Tuning:** Runtime PID gain adjustment with 'p' key
- **Advanced Sensor Processing:** Sector-based laser scan averaging for stable readings
- **Comprehensive Visualization:** RViz markers and debug data publishing

**Usage:**
```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Run PID wall follower
source install/setup.bash
ros2 run matrix_publisher pid_wall_follower_node

# Terminal 3: Monitor PID performance
python3 src/cpp/pid_monitor_working.py
```

**Controls:**
- `m` - Toggle between Manual and Autonomous modes
- `p` - Cycle through PID gain presets (Conservative/Balanced/Aggressive)
- **Manual Mode:** `w/a/d/x` for movement control
- **Autonomous Mode:** Fully automatic wall following

**PID Controllers:**
- **Wall Distance PID:** P=2.0, I=0.1, D=0.5 (maintains 0.5m from walls)
- **Obstacle Avoidance PID:** P=3.0, I=0.0, D=0.8 (emergency obstacle avoidance)
- **Speed Control PID:** P=1.5, I=0.05, D=0.3 (smooth speed transitions)

**Advanced Features:**
- **Anti-windup Protection:** Prevents integral term from growing excessively
- **Derivative Kick Prevention:** Smooth derivative calculations
- **Output Saturation:** Bounded control outputs for safety
- **Mode Persistence:** Intelligent wall preference memory
- **Real-time Debug Data:** Published to `/pid_debug` topic



## PID Controller Implementation

### Core PID Class (`include/pid_controller.hpp`)

A robust, reusable PID controller implementation with advanced features:

**Key Features:**
- **Anti-windup Protection:** Prevents integral term saturation
- **Derivative Kick Prevention:** Smooth derivative calculations
- **Output Saturation:** Configurable min/max output limits
- **Runtime Tuning:** Dynamic gain adjustment
- **Component Tracking:** Individual P, I, D term monitoring

**Constructor Parameters:**
```cpp
PIDController(double kp, double ki, double kd, 
              double output_min = -1.0, double output_max = 1.0,
              double integral_max = 1.0)
```

**Usage Example:**
```cpp
// Create PID controller
PIDController pid(2.0, 0.1, 0.5, -2.0, 2.0);

// In control loop
double output = pid.compute(setpoint, measurement, dt);

// Get individual components for debugging
auto components = pid.getLastComponents();
RCLCPP_INFO(logger, "P=%.2f I=%.2f D=%.2f", 
           components.proportional, 
           components.integral, 
           components.derivative);
```

### Angular PID Controller

Specialized PID controller for angular control with:
- **Angle Wrapping:** Proper handling of circular angles
- **Deadband:** Reduces oscillation near setpoint
- **Feedforward:** Optional velocity feedforward term

## PID Performance Monitoring

### Real-time Visualization (`pid_monitor_working.py`)

Python script for reliable real-time PID performance monitoring with live plots:

**Features:**
- **Live Plotting:** Real-time distance and PID error visualization
- **Dual Plots:** Distance measurements and PID error tracking
- **Auto-scaling:** Dynamic axis adjustment
- **Reliable Updates:** Updates every 2 seconds with guaranteed visibility
- **Simple Architecture:** Uses `spin_once()` for maximum reliability

**Usage:**
```bash
# Monitor PID performance (while PID node is running)
python3 src/cpp/pid_monitor_working.py
```

**Requirements:**
```bash
# Install matplotlib if needed
sudo apt install python3-matplotlib python3-tk
```

**Monitored Data:**
- Front/Left/Right distances
- Target distance
- PID error signal
- Real-time timestamps

## PID Tuning Guide

### Understanding PID Gains

**Proportional (P) Gain:**
- Controls response strength to current error
- Higher P = faster response, but may cause overshoot
- Too high = oscillation, too low = slow response

**Integral (I) Gain:**
- Eliminates steady-state error
- Accumulates error over time
- Too high = windup and instability

**Derivative (D) Gain:**
- Predicts future error based on rate of change
- Reduces overshoot and improves stability
- Too high = noise amplification

### Tuning Process

1. **Start with P-only:** Set I=0, D=0, increase P until oscillation
2. **Add D term:** Increase D to reduce oscillation
3. **Add I term:** Small I to eliminate steady-state error
4. **Fine-tune:** Adjust all gains for optimal performance

### Preset Gain Sets

**Conservative (Stable):**
- Wall Following: P=1.5, I=0.05, D=0.3
- Good for: Learning, testing, stable environments

**Balanced (Default):**
- Wall Following: P=2.0, I=0.1, D=0.5
- Good for: General use, mixed environments

**Aggressive (Fast):**
- Wall Following: P=3.0, I=0.2, D=0.8
- Good for: Fast navigation, open spaces

## Building the C++ Nodes

Make sure to install the required dependencies:

```bash
# Install sensor and visualization message packages
sudo apt install ros-humble-sensor-msgs ros-humble-visualization-msgs

# Build the package
colcon build --packages-select matrix_publisher

# Source the workspace
source install/setup.bash
```

## Node Comparison

| Node | Control Type | Obstacle Avoidance | User Input | PID Control | Best For |
|------|-------------|-------------------|------------|-------------|----------|
| `teleop_control_node` | Manual + Safety | ✅ Override | Keyboard | ❌ | Learning/Testing |
| `obstacle_avoider_node` | Autonomous | ✅ Active | None | ❌ | Basic Autonomous Demo |
| `wall_follower_node` | Dual Mode | ✅ Advanced | Mode Toggle | ❌ | Wall Navigation |
| `cmd_vel_publisher` | Programmed | ❌ None | None | ❌ | Basic Movement |
| `pid_wall_follower_node` | Dual Mode | ✅ PID-based | Mode Toggle | ✅ Multi-PID | Advanced Wall Following |

## Advanced Features

### Laser Scan Integration
The smart teleop and obstacle avoider nodes use the `/scan` topic to receive laser range data:
- 360-degree laser range measurements
- Obstacle detection within configurable distance thresholds
- Real-time safety decisions based on sensor data

### RViz Visualization
The smart teleop node publishes visualization markers:
- Red spheres indicate detected obstacles
- Markers appear at obstacle positions relative to robot
- View in RViz by adding "Marker" display type

### Safety Systems
Both obstacle-aware nodes implement safety features:
- Distance thresholds (0.3 meters default)
- Immediate stop commands when obstacles detected
- Graceful recovery when path clears

### Laser Scan Indexing (Critical Implementation Detail)
All laser-based nodes use the correct ROS2 laser scan convention:
- **Index 0**: Front (0°)
- **Index total_ranges/4**: Left side (90° counter-clockwise)
- **Index total_ranges/2**: Back (180°)
- **Index 3*total_ranges/4**: Right side (270° counter-clockwise)

**Important**: ROS2 laser scans rotate **counter-clockwise**, not clockwise. This was a critical bug fix that prevented left/right confusion in wall following and obstacle detection.

---

## Prerequisites

- ROS2 Humble installed
- Workspace set up (e.g., `~/ros2_ws`)
- Basic understanding of C++ and ROS2 concepts

## Building the Package

### Step 1: Navigate to workspace
```bash
cd ~/ros2_ws
```

### Step 2: Build the package
```bash
colcon build --packages-select matrix_publisher
```

### Step 3: Source the environment
```bash
source install/setup.bash
```



## Running the Nodes

### Matrix Publisher/Subscriber Demo

#### Terminal 1 - Run Publisher
```bash
# Make sure you're in the workspace directory
cd ~/ros2_ws

# Source the environment
source install/setup.bash

# Run the publisher node
ros2 run matrix_publisher matrix_publisher_node
```

**Expected output**:
```
[INFO] [timestamp] [matrix_publisher]: Publishing matrix with 6 elements
[INFO] [timestamp] [matrix_publisher]: Publishing matrix with 6 elements
...
```

#### Terminal 2 - Run Subscriber
```bash
# Open a new terminal and navigate to workspace
cd ~/ros2_ws

# Source the environment
source install/setup.bash

# Run the subscriber node
ros2 run matrix_publisher matrix_subscriber_node
```

**Expected output**:
```
Reshaped matrix:
1 2 3
4 5 6
Reshaped matrix:
1 2 3
4 5 6
...
```

### Addition Service Demo

#### Terminal 1 - Run Server
```bash
# Make sure you're in the workspace directory
cd ~/ros2_ws

# Source the environment
source install/setup.bash

# Run the server node
ros2 run matrix_publisher multiply_server_node
```

#### Terminal 2 - Run Bridge
```bash
# Open a new terminal and navigate to workspace
cd ~/ros2_ws

# Source the environment
source install/setup.bash

# Run the bridge node
ros2 run matrix_publisher bridge_node
```

#### Terminal 3 - Publish to Input Topic
```bash
# Open a new terminal and navigate to workspace
cd ~/ros2_ws

# Source the environment
source install/setup.bash

# Publish test data
ros2 topic pub /multiply_input example_interfaces/msg/Int64MultiArray "{data: [5, 3]}" --once
```

#### Terminal 4 - Monitor Results
```bash
# Open a new terminal and navigate to workspace
cd ~/ros2_ws

# Source the environment
source install/setup.bash

# Monitor the result topic
ros2 topic echo /multiply_result
```

**Expected output**:
```
[INFO] [timestamp] [topic_service_bridge]: Bridge ready
[INFO] [timestamp] [topic_service_bridge]: Sending request: 5.00 * 3.00
[INFO] [timestamp] [topic_service_bridge]: Result: 15.00
[INFO] [timestamp] [topic_service_bridge]: Published result: 15.00
```

## What the Code Does

### Publisher (`matrix_publisher_node.cpp`)
- Creates a 2x3 matrix: `[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]`
- Flattens it to a 1D array: `[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]`
- Publishes this as `Float64MultiArray` on topic `/matrix` every 1 second

### Subscriber (`matrix_subscriber_node.cpp`)
- Subscribes to the `/matrix` topic
- Receives `Float64MultiArray` messages
- Reshapes the flat array back into a 2x3 matrix and displays it

### Service Server (`multiply_server_node.cpp`)
- Creates a service server listening on `/multiply`
- Uses `MultiplyTwoFloats` service interface (takes two floats, returns their product)
- Performs multiplication: `a * b = product`
- Logs each request with the calculation

### Bridge Node (`bridge_node.cpp`)
- Subscribes to `/multiply_input` topic (Int64MultiArray)
- Extracts two numbers from the array
- Calls the `/multiply` service with these numbers
- Publishes the result to `/multiply_result` topic (Float64)

## C++ and ROS2 Learning Points

### C++ Concepts Demonstrated
1. **Classes and Inheritance**
   - All nodes inherit from `rclcpp::Node`
   - Custom classes for different node types
   - Public/private member organization

2. **Smart Pointers**
   - `std::shared_ptr` for node management
   - `std::make_shared` for object creation

3. **Templates**
   - Matrix class template for different data types
   - ROS2 message type templates

4. **Modern C++ Features**
   - Lambda functions
   - Auto type deduction
   - Range-based for loops
   - Chrono literals

### ROS2 Concepts Demonstrated
1. **Node Lifecycle**
   - Node initialization
   - Spinning and shutdown
   - Service availability checking

2. **Communication Patterns**
   - Publisher/Subscriber (pub/sub)
   - Service/Client (request/response)
   - Topic and service naming
   - Bridge pattern (connecting topics to services)

3. **Message Types**
   - `Float64MultiArray` for matrix data
   - `MultiplyTwoFloats` service interface

4. **ROS2 Best Practices**
   - Proper node naming
   - Logging levels (INFO, WARN, ERROR)
   - Error handling
   - Service timeout handling

## Available Services and Topics

### Topics
- `/matrix` - `std_msgs/msg/Float64MultiArray` - Matrix data
- `/multiply_input` - `example_interfaces/msg/Int64MultiArray` - Input numbers for multiplication
- `/multiply_result` - `std_msgs/msg/Float64` - Result of multiplication

### Services
- `/multiply` - `matrix_publisher/srv/MultiplyTwoFloats` - Multiplication service

## Troubleshooting

### Build Issues
If you encounter compiler errors:
1. Clean the build: `rm -rf build/matrix_publisher install/matrix_publisher log/matrix_publisher`
2. Use the full build command with system compilers (see Step 2 above)

### Runtime Issues
- Make sure to source `install/setup.bash` in each terminal
- Check that both nodes are running in different terminals
- Verify the workspace path is correct

### Common Warnings (Safe to Ignore)
- Missing `local_setup.sh` files - these are optional
- Deprecation warnings about callback signatures - non-critical

## Additional ROS2 Commands

You can inspect topics and services using standard ROS2 commands:

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /matrix

# Echo topic messages
ros2 topic echo /matrix

# List all services
ros2 service list

# Show service info
ros2 service type /multiply

# Check service interface
ros2 interface show matrix_publisher/srv/MultiplyTwoFloats
```

## Learning Resources

### ROS2 Documentation
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [ROS2 C++ Client Library](https://docs.ros.org/en/rolling/Concepts/Basic/About-ROS-2-Client-Libraries.html)
- [ROS2 Service Tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)

### C++ Resources
- [C++ Reference](https://en.cppreference.com/)
- [Modern C++ Features](https://github.com/AnthonyCalandra/modern-cpp-features)
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) 
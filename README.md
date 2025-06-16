# Matrix Publisher ROS2 Package

A ROS2 package that demonstrates matrix publishing/subscribing and service handling using `Float64MultiArray` messages and `MultiplyTwoFloats` service. This package serves as a learning resource for C++ and ROS2 development.

## Project Structure

```
matrix_publisher/
├── src/
│   ├── matrix_publisher_node.cpp    # Publisher node
│   ├── matrix_subscriber_node.cpp   # Subscriber node
│   ├── multiply_server_node.cpp     # Service server node
│   ├── bridge_node.cpp             # Topic-to-service bridge node
├── srv/
│   └── MultiplyTwoFloats.srv       # Service definition
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

### Method 1: Custom Python Teleop (Recommended)

We've created a custom keyboard control script with improved responsiveness:

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

---

## Prerequisites

- ROS2 installed (conda environment with robostack)
- Workspace set up in `/Users/jihwan/ros2_ws`
- Basic understanding of C++ and ROS2 concepts

## Building the Package

**Important**: This package requires using system compilers due to conda environment issues with clang.

### Step 1: Navigate to workspace
```bash
cd /Users/jihwan/ros2_ws
```

### Step 2: Build the package
```bash
colcon build --packages-select matrix_publisher --cmake-args -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++
```

**Note**: The specific compiler flags are required to bypass broken conda clang compilers.

### Step 3: Source the environment
```bash
source install/setup.bash
```

**Expected warnings** (safe to ignore):
```
/Users/jihwan/miniforge3/envs/ros2/local_setup.bash:.:11: no such file or directory: /Users/jihwan/ros2_ws/local_setup.sh
not found: "/Users/jihwan/ros2_ws/local_setup.bash"
```

## Running the Nodes

### Matrix Publisher/Subscriber Demo

#### Terminal 1 - Run Publisher
```bash
# Make sure you're in the workspace directory
cd /Users/jihwan/ros2_ws

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
cd /Users/jihwan/ros2_ws

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
cd /Users/jihwan/ros2_ws

# Source the environment
source install/setup.bash

# Run the server node
ros2 run matrix_publisher multiply_server_node
```

#### Terminal 2 - Run Bridge
```bash
# Open a new terminal and navigate to workspace
cd /Users/jihwan/ros2_ws

# Source the environment
source install/setup.bash

# Run the bridge node
ros2 run matrix_publisher bridge_node
```

#### Terminal 3 - Publish to Input Topic
```bash
# Open a new terminal and navigate to workspace
cd /Users/jihwan/ros2_ws

# Source the environment
source install/setup.bash

# Publish test data
ros2 topic pub /multiply_input example_interfaces/msg/Int64MultiArray "{data: [5, 3]}" --once
```

#### Terminal 4 - Monitor Results
```bash
# Open a new terminal and navigate to workspace
cd /Users/jihwan/ros2_ws

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
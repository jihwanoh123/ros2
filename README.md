# Matrix Publisher ROS2 Package

A ROS2 package that demonstrates matrix publishing/subscribing and service handling using `Float64MultiArray` messages and `AddTwoInts` service. This package serves as a learning resource for C++ and ROS2 development.

## Project Structure

```
matrix_publisher/
├── src/
│   ├── matrix_publisher_node.cpp    # Publisher node
│   ├── matrix_subscriber_node.cpp   # Subscriber node
│   ├── add_server_node.cpp         # Service server node
│   ├── adder_client_node.cpp       # Command-line client node
│   └── interactive_client_node.cpp # Interactive client node
├── CMakeLists.txt
├── package.xml
└── README.md
```

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
ros2 run matrix_publisher add_server_node
```

#### Terminal 2 - Run Command-line Client
```bash
# Open a new terminal and navigate to workspace
cd /Users/jihwan/ros2_ws

# Source the environment
source install/setup.bash

# Run the command-line client with two numbers
ros2 run matrix_publisher adder_client_node 5 3
```

**Expected output**:
```
[INFO] [timestamp] [adder_client]: Result: 8
```

#### Terminal 2 (Alternative) - Run Interactive Client
```bash
# Open a new terminal and navigate to workspace
cd /Users/jihwan/ros2_ws

# Source the environment
source install/setup.bash

# Run the interactive client
ros2 run matrix_publisher interactive_client_node
```

**Expected output**:
```
[INFO] [timestamp] [interactive_client]: Service available. Ready for input.
Enter two integers (a b), or Ctrl+C to exit: 5 3
[INFO] [timestamp] [interactive_client]: Result: 8
Enter two integers (a b), or Ctrl+C to exit: 
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

### Service Server (`add_server_node.cpp`)
- Creates a service server listening on `/add`
- Uses `AddTwoInts` service interface (takes two integers, returns their sum)
- Performs addition: `a + b = sum`
- Logs each request with the calculation

### Command-line Client (`adder_client_node.cpp`)
- Takes two integers as command-line arguments
- Creates a service client for the `/add` service
- Sends a single request and displays the result
- Automatically shuts down after receiving the response

### Interactive Client (`interactive_client_node.cpp`)
- Creates a service client for the `/add` service
- Waits for the service to become available
- Continuously prompts for two integers
- Sends requests and displays results until Ctrl+C is pressed

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

3. **Message Types**
   - `Float64MultiArray` for matrix data
   - `AddTwoInts` service interface

4. **ROS2 Best Practices**
   - Proper node naming
   - Logging levels (INFO, WARN, ERROR)
   - Error handling
   - Service timeout handling

## Available Services and Topics

### Topics
- `/matrix` - `std_msgs/msg/Float64MultiArray` - Matrix data

### Services
- `/add` - `example_interfaces/srv/AddTwoInts` - Addition service

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
ros2 service type /add

# Check service interface
ros2 interface show example_interfaces/srv/AddTwoInts
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
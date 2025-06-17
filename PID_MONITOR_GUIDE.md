# PID Monitor Guide

This guide explains the three different PID monitoring tools available and how to use them effectively.

## Available Monitors

### 1. Simple Text Monitor (`pid_monitor_simple.py`) ⭐ **RECOMMENDED**

**Best for:** Debugging, reliable monitoring, terminal environments

**Features:**
- ✅ Always works (no GUI dependencies)
- ✅ Real-time numerical data display
- ✅ Formatted table output
- ✅ No threading issues
- ✅ Works over SSH

**Usage:**
```bash
python3 src/cpp/pid_monitor_simple.py
```

**Sample Output:**
```
🎯 Simple PID Monitor - Real-time data display
==========================================
Time: 12.34s | Front: 0.335m | Left: 0.779m | Right: 0.769m | Target: 3.000m
PID: Error=0.000 | Integral=0.000 | Angular=0.050 rad/s
```

### 2. Live Graphical Monitor (`pid_monitor_live.py`) 📊 **BEST VISUALIZATION**

**Best for:** Real-time visualization, analysis, demonstrations

**Features:**
- ✅ Real-time updating plots
- ✅ Distance and PID component graphs
- ✅ Auto-scaling axes
- ✅ Smooth updates (10Hz)
- ⚠️ Requires GUI display

**Usage:**
```bash
python3 src/cpp/pid_monitor_live.py
```

**Requirements:**
```bash
sudo apt install python3-matplotlib python3-tk
```

### 3. Animation Monitor (`pid_monitor.py`) 🎬 **EXPERIMENTAL**

**Best for:** Advanced features, research

**Features:**
- ✅ Matplotlib FuncAnimation
- ✅ Advanced plotting features
- ⚠️ May have threading issues
- ⚠️ Complex animation system

**Usage:**
```bash
python3 src/cpp/pid_monitor.py
```

## Quick Start Guide

### Step 1: Launch Simulation
```bash
# Terminal 1: Start Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Start PID Node
```bash
# Terminal 2: Run PID controller
source install/setup.bash
ros2 run matrix_publisher pid_wall_follower_node
# OR
ros2 run matrix_publisher pid_obstacle_avoider_node
```

### Step 3: Choose Monitor
```bash
# Terminal 3: Pick your monitor

# Option A: Simple and reliable
python3 src/cpp/pid_monitor_simple.py

# Option B: Beautiful plots
python3 src/cpp/pid_monitor_live.py

# Option C: Experimental
python3 src/cpp/pid_monitor.py
```

## Data Format

All monitors subscribe to `/pid_debug` topic with this data format:

| Index | Data | Description |
|-------|------|-------------|
| 0 | Front Distance | Distance to front obstacle (m) |
| 1 | Left Distance | Distance to left obstacle (m) |
| 2 | Right Distance | Distance to right obstacle (m) |
| 3 | Target Distance | PID setpoint (m) |
| 4 | PID Error | Current error signal |
| 5 | PID Integral | Accumulated integral term |
| 6 | Angular Velocity | Control output (rad/s) |
| 7 | Timestamp | ROS time (seconds) |

## Troubleshooting

### "No data received"
- Check if PID node is running: `ros2 node list | grep pid`
- Verify topic exists: `ros2 topic list | grep pid_debug`
- Test data flow: `ros2 topic echo /pid_debug --once`

### "Display issues" (Live Monitor)
- Install GUI packages: `sudo apt install python3-tk`
- Try X11 forwarding: `ssh -X user@host`
- Use simple monitor instead: `python3 src/cpp/pid_monitor_simple.py`

### "matplotlib errors"
```bash
# Install required packages
sudo apt install python3-matplotlib python3-tk
pip3 install matplotlib
```

### "Threading issues" (Animation Monitor)
- Use live monitor instead: `python3 src/cpp/pid_monitor_live.py`
- Or use simple monitor: `python3 src/cpp/pid_monitor_simple.py`

## Monitor Comparison

| Feature | Simple | Live | Animation |
|---------|--------|------|-----------|
| **Reliability** | ⭐⭐⭐ | ⭐⭐ | ⭐ |
| **Visualization** | ⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| **Setup Complexity** | ⭐⭐⭐ | ⭐⭐ | ⭐ |
| **Resource Usage** | ⭐⭐⭐ | ⭐⭐ | ⭐ |
| **SSH Compatible** | ✅ | ⚠️ | ⚠️ |
| **Real-time Updates** | ✅ | ✅ | ⚠️ |

## Best Practices

1. **Start with Simple Monitor** - Always test with the simple monitor first
2. **Use Live Monitor for Analysis** - Switch to live monitor for detailed analysis
3. **Monitor Data Quality** - Check for reasonable values (distances 0-5m, etc.)
4. **Adjust PID Gains** - Use 'p' key in PID nodes to tune gains while monitoring
5. **Save Data** - Consider logging data for offline analysis

## Example Workflow

```bash
# 1. Quick check - is data flowing?
python3 src/cpp/pid_monitor_simple.py

# 2. Detailed analysis - see the graphs
python3 src/cpp/pid_monitor_live.py

# 3. Tune PID gains while watching performance
# In PID node terminal, press 'p' to cycle through gain presets
# Watch the monitor to see the effect
```

This multi-monitor approach gives you flexibility to choose the right tool for your specific needs! 
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import time
from collections import deque

class SimpleLivePIDMonitor(Node):
    def __init__(self):
        super().__init__('simple_live_pid_monitor')
        
        # Subscribe to PID debug data
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/pid_debug',
            self.debug_callback,
            10)
        
        # Data storage (last 50 points)
        self.max_points = 50
        self.time_data = deque(maxlen=self.max_points)
        self.front_distance = deque(maxlen=self.max_points)
        self.left_distance = deque(maxlen=self.max_points)
        self.right_distance = deque(maxlen=self.max_points)
        self.target_distance = deque(maxlen=self.max_points)
        self.pid_error = deque(maxlen=self.max_points)
        
        self.start_time = time.time()
        self.message_count = 0
        
        print("🎯 Simple Live PID Monitor started!")
        print("📡 Waiting for PID data...")
    
    def debug_callback(self, msg):
        if len(msg.data) >= 6:
            self.message_count += 1
            current_time = time.time() - self.start_time
            
            # Store data
            self.time_data.append(current_time)
            self.front_distance.append(msg.data[0])
            self.left_distance.append(msg.data[1])
            self.right_distance.append(msg.data[2])
            self.target_distance.append(msg.data[3])
            self.pid_error.append(msg.data[4])
            
            # Print status every 10 messages
            if self.message_count % 10 == 0:
                print(f"📊 Received {self.message_count} messages, Front: {msg.data[0]:.2f}m")

def main():
    print("🚀 Starting Simple Live PID Monitor...")
    
    rclpy.init()
    monitor = SimpleLivePIDMonitor()
    
    # Set up matplotlib
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Simple Live PID Monitor', fontsize=14)
    
    # Configure plots
    ax1.set_title('Distance Measurements')
    ax1.set_ylabel('Distance (m)')
    ax1.grid(True)
    ax1.legend(['Front', 'Left', 'Right', 'Target'])
    
    ax2.set_title('PID Error')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error')
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)
    
    print("🎨 Plot window should be visible now!")
    print("📈 Plots will update every 2 seconds...")
    
    try:
        last_update = time.time()
        while rclpy.ok():
            # Process ROS messages
            rclpy.spin_once(monitor, timeout_sec=0.1)
            
            # Update plots every 2 seconds
            current_time = time.time()
            if current_time - last_update > 2.0 and len(monitor.time_data) > 1:
                # Convert to lists
                times = list(monitor.time_data)
                front = list(monitor.front_distance)
                left = list(monitor.left_distance)
                right = list(monitor.right_distance)
                target = list(monitor.target_distance)
                error = list(monitor.pid_error)
                
                # Clear and update plots
                ax1.clear()
                ax2.clear()
                
                # Plot distance data
                ax1.plot(times, front, 'r-', label='Front', linewidth=2)
                ax1.plot(times, left, 'g-', label='Left', alpha=0.7)
                ax1.plot(times, right, 'b-', label='Right', alpha=0.7)
                ax1.plot(times, target, 'y--', label='Target', linewidth=2)
                ax1.set_title('Distance Measurements')
                ax1.set_ylabel('Distance (m)')
                ax1.grid(True)
                ax1.legend()
                ax1.set_ylim(0, 4)
                
                # Plot PID error
                ax2.plot(times, error, 'orange', label='Error', linewidth=2)
                ax2.set_title('PID Error')
                ax2.set_xlabel('Time (s)')
                ax2.set_ylabel('Error')
                ax2.grid(True)
                ax2.legend()
                
                # Update display
                plt.tight_layout()
                fig.canvas.draw()
                fig.canvas.flush_events()
                
                last_update = current_time
                print(f"📊 Updated plots - {len(times)} data points, Latest front: {front[-1]:.2f}m")
            
            time.sleep(0.05)  # Small sleep to prevent busy waiting
            
    except KeyboardInterrupt:
        print("\n🛑 Monitor shutting down...")
    finally:
        rclpy.shutdown()
        plt.close('all')
        print("✅ Shutdown complete")

if __name__ == '__main__':
    main() 
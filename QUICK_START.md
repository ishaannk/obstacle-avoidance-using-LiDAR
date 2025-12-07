# Quick Start Guide - RPLIDAR Obstacle Avoidance

## 🚀 Quick Setup

### 1. Build the Package
```bash
cd ~/rplidar_ws
colcon build --packages-select rplidar_obstacle_avoidance
source install/setup.bash
```

### 2. Run the System
```bash
ros2 launch rplidar_obstacle_avoidance obstacle_avoidance_launch.py
```

### 3. Monitor Output
```bash
# In another terminal, check obstacle detection
ros2 topic echo /obstacle_detector
```

---

## 📊 System Overview

```
┌──────────────┐
│ RPLIDAR C1   │  Hardware Sensor
└──────┬───────┘
       │ USB/Serial
       ▼
┌──────────────────┐
│  rplidar_node    │  Publishes /scan
└──────┬───────────┘
       │ LaserScan message
       │ (360 distance measurements)
       ▼
┌──────────────────────────┐
│ obstacle_avoidance_node  │  Processes & Filters
│                          │  Publishes /obstacle_detector
└──────┬───────────────────┘
       │ Bool (True/False)
       ▼
┌──────────────────┐
│ Robot Controller │  Your Application
└──────────────────┘
```

---

## ⚙️ Configuration

Edit: `src/rplidar_obstacle_avoidance/config/obstacle_avoidance.yaml`

### Key Settings:

```yaml
min_distance_threshold: 0.5    # Distance in meters
fov_start_angle: 0.0          # Start angle (degrees)
fov_end_angle: 120.0          # End angle (degrees)
min_obstacle_points: 5        # Points needed to confirm
consecutive_detections: 3      # Scans that must agree
```

---

## 🔧 Troubleshooting

### Problem: Always detecting obstacles
**Solution**: Increase filtering
```yaml
min_obstacle_points: 10
consecutive_detections: 5
min_distance_threshold: 0.8
```

### Problem: Missing real obstacles
**Solution**: Decrease filtering
```yaml
min_obstacle_points: 3
consecutive_detections: 2
min_distance_threshold: 0.3
```

### Problem: Sensor mount causing false alarms
**Solution**: Exclude that angle range
```yaml
exclude_angle_ranges: [25.0, 35.0]  # Exclude 25-35 degrees
```

---

## 📝 ROS2 Commands Cheat Sheet

```bash
# List all topics
ros2 topic list

# View topic data
ros2 topic echo /scan
ros2 topic echo /obstacle_detector

# Check node info
ros2 node list
ros2 node info /obstacle_avoidance_node

# View parameters
ros2 param list /obstacle_avoidance_node
ros2 param get /obstacle_avoidance_node min_distance_threshold

# Set parameter at runtime
ros2 param set /obstacle_avoidance_node min_distance_threshold 0.8
```

---

## 🎯 Integration Example

### Python Robot Controller:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/obstacle_detector',
            self.obstacle_callback,
            10
        )
        # Your motor control code here
    
    def obstacle_callback(self, msg):
        if msg.data:  # True = obstacle detected
            self.get_logger().warn('OBSTACLE! Stopping...')
            # Stop motors
            # Turn or reverse
        else:  # False = clear
            # Continue normal operation
            pass

def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 📚 Learn More

See `WORKSPACE_EXPLANATION.md` for detailed concepts and architecture.

---

**Happy Robot Building! 🤖**


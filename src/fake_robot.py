#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Header, Int32, String
from cv_bridge import CvBridge
import struct
import argparse
import json
import time
import uuid

class FakeRobot:
    def __init__(self, log_level=rospy.INFO):
        rospy.init_node('fake_robot_node', anonymous=True, log_level=log_level)
        
        # Publisher for camera images
        self.image_pub = rospy.Publisher('/cameraF/camera/color/image_raw', Image, queue_size=1)
        
        # Publisher for point cloud data
        self.pointcloud_pub = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=1)
        
        # Publisher for goal reached status
        self.goal_reached_pub = rospy.Publisher('/goal_reached', Bool, queue_size=1)
        
        # Publisher for operation status (1Hz)
        self.status_pub = rospy.Publisher('/operation_status', Int32, queue_size=10)
        
        # Publisher for robot information (1Hz)
        self.robot_info_pub = rospy.Publisher('/robot_info', String, queue_size=10)
        
        # CV Bridge for converting between OpenCV and ROS Image messages
        self.bridge = CvBridge()
        
        # Timer for goal reached callback
        self.goal_timer = None
        
        # Robot operation status (default: 0 = idle)
        self.operation_status = 0
        
        # Robot information parameters
        self.accid = "PF_TRON1A_042"
        self.sw_version = "robot-tron1-2.0.10.20241111103012"
        self.battery_level = 50  # Start with 95% battery
        
        # Image parameters
        self.width = 1920  # 1080P width
        self.height = 1080  # 1080P height
        self.frame_counter = 0
        
        # Set up timer for 30Hz publishing (images)
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.publish_image)
        
        # Set up timer for 10Hz publishing (point clouds)
        self.pc_timer = rospy.Timer(rospy.Duration(1.0/10.0), self.publish_pointcloud)
        
        # Set up timer for 1Hz publishing (operation status)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_operation_status)
        
        # Set up timer for 1Hz publishing (robot information)
        self.robot_info_timer = rospy.Timer(rospy.Duration(1.0), self.publish_robot_info)
        
        # Subscribers for navigation topics
        self.nav_goal_sub = rospy.Subscriber('/navigation_goal', PoseStamped, self.navigation_goal_callback)
        self.cancel_nav_sub = rospy.Subscriber('/cancel_nav', Bool, self.cancel_navigation_callback)
        
        # Subscriber for operation status change
        self.status_change_sub = rospy.Subscriber('/operation_status_change', Int32, self.status_change_callback)
        
        # Subscriber for manual control commands
        self.cmd_vel_1_sub = rospy.Subscriber('/cmd_vel_1', Twist, self.cmd_vel_1_callback)
        
        rospy.loginfo("Fake robot node started. Publishing 1080P images at 30Hz on /cameraF/camera/color/image_raw")
        rospy.loginfo("Publishing point clouds at 10Hz on /cloud_registered")
        rospy.loginfo("Publishing goal reached status on /goal_reached topic")
        rospy.loginfo("Publishing operation status at 1Hz on /operation_status topic")
        rospy.loginfo("Publishing robot information at 1Hz on /robot_info topic")
        rospy.loginfo("Subscribed to /navigation_goal and /cancel_nav topics")
        rospy.loginfo("Subscribed to /operation_status_change topic")
        rospy.loginfo("Subscribed to /cmd_vel_1 topic for manual control commands")

    def navigation_goal_callback(self, msg):
        """Callback for navigation goal messages"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("NAVIGATION GOAL RECEIVED:")
        rospy.loginfo(f"Frame ID: {msg.header.frame_id}")
        rospy.loginfo(f"Timestamp: {msg.header.stamp}")
        rospy.loginfo(f"Position:")
        rospy.loginfo(f"  X: {msg.pose.position.x:.3f}")
        rospy.loginfo(f"  Y: {msg.pose.position.y:.3f}")
        rospy.loginfo(f"  Z: {msg.pose.position.z:.3f}")
        rospy.loginfo(f"Orientation:")
        rospy.loginfo(f"  X: {msg.pose.orientation.x:.3f}")
        rospy.loginfo(f"  Y: {msg.pose.orientation.y:.3f}")
        rospy.loginfo(f"  Z: {msg.pose.orientation.z:.3f}")
        rospy.loginfo(f"  W: {msg.pose.orientation.w:.3f}")
        rospy.loginfo("=" * 50)
        
        # Change status to navigation mode
        self.operation_status = 4  # Navigation status
        rospy.loginfo("Operation status changed to: 4 (Navigation)")
        
        # Cancel any existing goal timer
        if self.goal_timer is not None:
            self.goal_timer.shutdown()
            rospy.loginfo("Previous goal timer cancelled")
        
        # Set up a 10-second timer to publish goal reached
        self.goal_timer = rospy.Timer(rospy.Duration(200.0), self.goal_reached_callback, oneshot=True)
        rospy.loginfo("Started 20-second timer for goal reached notification")

    def cancel_navigation_callback(self, msg):
        """Callback for cancel navigation messages"""
        rospy.loginfo("*" * 30)
        rospy.loginfo("CANCEL NAVIGATION RECEIVED:")
        rospy.loginfo(f"Cancel: {msg.data}")
        rospy.loginfo("*" * 30)
        
        # Change status back to idle
        self.operation_status = 0  # Idle status
        rospy.loginfo("Operation status changed to: 0 (Idle)")
        
        # Cancel the goal timer if it exists
        if self.goal_timer is not None:
            self.goal_timer.shutdown()
            self.goal_timer = None
            rospy.loginfo("Goal timer cancelled due to navigation cancellation")

    def cmd_vel_1_callback(self, msg):
        """Callback for manual control velocity commands"""
        rospy.loginfo(">" * 40)
        rospy.loginfo("MANUAL CONTROL COMMAND RECEIVED:")
        rospy.loginfo(f"Linear velocity:")
        rospy.loginfo(f"  X: {msg.linear.x:.3f} m/s")
        # rospy.loginfo(f"  Y: {msg.linear.y:.3f} m/s")
        # rospy.loginfo(f"  Z: {msg.linear.z:.3f} m/s")
        rospy.loginfo(f"Angular velocity:")
        # rospy.loginfo(f"  X: {msg.angular.x:.3f} rad/s")
        # rospy.loginfo(f"  Y: {msg.angular.y:.3f} rad/s")
        rospy.loginfo(f"  Z: {msg.angular.z:.3f} rad/s")
        rospy.loginfo(">" * 40)
        
        # Change status to manual control mode if any velocity is non-zero
        if (msg.linear.x != 0 or msg.linear.y != 0 or msg.linear.z != 0 or
            msg.angular.x != 0 or msg.angular.y != 0 or msg.angular.z != 0):
            if self.operation_status != 1:  # Only log if status is changing
                self.operation_status = 1  # Manual control status
                rospy.loginfo("Operation status changed to: 1 (Manual)")
        else:
            # If all velocities are zero, change back to idle
            if self.operation_status == 1:  # Only change if currently in manual control
                self.operation_status = 0  # Idle status
                rospy.loginfo("Operation status changed to: 0 (Idle) - Manual control stopped")

    def status_change_callback(self, msg):
        """Callback for operation status change commands"""
        rospy.loginfo("+" * 50)
        rospy.loginfo("OPERATION STATUS CHANGE RECEIVED:")
        rospy.loginfo(f"Previous status: {self.operation_status}")
        rospy.loginfo(f"New status: {msg.data}")
        rospy.loginfo("Status meanings:")
        rospy.loginfo("  0: Idle")
        rospy.loginfo("  1: Manual")
        rospy.loginfo("  2: Autofilm")
        rospy.loginfo("  3: Following")
        rospy.loginfo("  4: Navigation")
        rospy.loginfo("+" * 50)
        
        # Update the operation status
        self.operation_status = msg.data
        
        # Immediately publish the new status
        self.publish_operation_status(None)

    def publish_operation_status(self, event):
        """Callback function to publish operation status at 1Hz"""
        try:
            status_msg = Int32()
            status_msg.data = self.operation_status
            
            self.status_pub.publish(status_msg)
            
            # Only log every 10th status message to avoid spam (every 10 seconds)
            if event is None or rospy.get_time() % 10 < 1.0:
                rospy.loginfo(f"Published operation status: {self.operation_status}")
            
        except Exception as e:
            rospy.logerr(f"Error publishing operation status: {e}")

    def get_status_string(self):
        """Convert operation status integer to status string"""
        status_map = {
            0: "IDLE",
            1: "WALK",      # Manual control = walking
            2: "AUTOFILM",
            3: "FOLLOWING", 
            4: "NAVIGATION"
        }
        return status_map.get(self.operation_status, "UNKNOWN")

    def publish_robot_info(self, event):
        """Callback function to publish robot information at 1Hz"""
        try:
            # Create robot info message
            robot_info = {
                "accid": self.accid,
                "title": "notify_robot_info",
                "timestamp": int(time.time() * 1000),  # Current timestamp in milliseconds
                "guid": str(uuid.uuid4()).replace("-", ""),  # Generate unique GUID
                "data": {
                    "accid": self.accid,
                    "sw_version": self.sw_version,
                    "imu": "OK",        # IMU diagnosis info
                    "camera": "OK",     # Camera diagnosis info  
                    "motor": "OK",      # Motor diagnosis info
                    "battery": self.battery_level,  # Battery level
                    "status": self.get_status_string()  # Robot operation mode
                }
            }
            
            # Convert to JSON string
            robot_info_json = json.dumps(robot_info, separators=(',', ':'))
            
            # Create ROS message
            info_msg = String()
            info_msg.data = robot_info_json
            
            # Publish the robot info
            self.robot_info_pub.publish(info_msg)
            
            # Only log every 10th info message to avoid spam (every 10 seconds)
            if event is None or rospy.get_time() % 10 < 1.0:
                rospy.loginfo(f"Published robot info: status={self.get_status_string()}, battery={self.battery_level}%")
            
            # Simulate battery drain (very slowly)
            if self.battery_level > 10:
                # Drain battery by 1% every ~10 minutes (600 seconds)
                if rospy.get_time() % 600 < 1.0:
                    self.battery_level -= 1
            
        except Exception as e:
            rospy.logerr(f"Error publishing robot info: {e}")

    def goal_reached_callback(self, event):
        """Callback to publish goal reached status after 10 seconds"""
        goal_reached_msg = Bool()
        goal_reached_msg.data = True
        
        self.goal_reached_pub.publish(goal_reached_msg)
        rospy.loginfo("GOAL REACHED: Published True on /goal_reached topic")
        
        # Change status back to idle after reaching goal
        self.operation_status = 0  # Idle status
        rospy.loginfo("Operation status changed to: 0 (Idle) - Goal reached")
        
        # Reset the timer reference
        self.goal_timer = None

    def generate_realistic_pointcloud(self):
        """Generate realistic point cloud data simulating an indoor environment"""
        points = []
        
        # Generate floor points (y = 0, scattered across x-z plane)
        floor_points = 2000
        for _ in range(floor_points):
            x = np.random.uniform(-5.0, 5.0)
            z = np.random.uniform(0.5, 8.0)
            y = np.random.normal(0.0, 0.02)  # Slight noise for realistic floor
            
            # Add RGB color (grayish floor)
            r, g, b = 80, 80, 90
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            points.append([x, y, z, rgb])
        
        # Generate wall points (vertical surfaces)
        wall_points = 1500
        for _ in range(wall_points):
            # Left and right walls
            if np.random.random() < 0.5:
                x = np.random.choice([-5.0, 5.0]) + np.random.normal(0, 0.1)
                z = np.random.uniform(0.5, 8.0)
            else:
                # Front and back walls
                x = np.random.uniform(-5.0, 5.0)
                z = np.random.choice([0.5, 8.0]) + np.random.normal(0, 0.1)
            
            y = np.random.uniform(0.0, 3.0)  # Wall height
            
            # Add RGB color (wall color)
            r, g, b = 150, 140, 130
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            points.append([x, y, z, rgb])
        
        # Generate ceiling points
        ceiling_points = 800
        for _ in range(ceiling_points):
            x = np.random.uniform(-5.0, 5.0)
            z = np.random.uniform(0.5, 8.0)
            y = 3.0 + np.random.normal(0.0, 0.05)
            
            # Add RGB color (white ceiling)
            r, g, b = 200, 200, 200
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            points.append([x, y, z, rgb])
        
        # Generate furniture/objects
        furniture_points = 1000
        
        # Table
        table_center = [2.0, 0.8, 3.0]
        for _ in range(300):
            x = table_center[0] + np.random.uniform(-0.5, 0.5)
            y = table_center[1] + np.random.uniform(-0.1, 0.1)
            z = table_center[2] + np.random.uniform(-0.8, 0.8)
            
            r, g, b = 139, 69, 19  # Brown table
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            points.append([x, y, z, rgb])
        
        # Chair
        chair_center = [1.5, 0.5, 2.5]
        for _ in range(200):
            x = chair_center[0] + np.random.uniform(-0.3, 0.3)
            y = chair_center[1] + np.random.uniform(0.0, 0.8)
            z = chair_center[2] + np.random.uniform(-0.3, 0.3)
            
            r, g, b = 100, 100, 100  # Gray chair
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            points.append([x, y, z, rgb])
        
        # Random objects/clutter
        for _ in range(500):
            x = np.random.uniform(-4.0, 4.0)
            y = np.random.uniform(0.1, 2.0)
            z = np.random.uniform(1.0, 7.0)
            
            # Random colors for objects
            r = np.random.randint(50, 255)
            g = np.random.randint(50, 255)
            b = np.random.randint(50, 255)
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            points.append([x, y, z, rgb])
        
        return np.array(points, dtype=np.float32)

    def create_pointcloud2_msg(self, points):
        """Create a PointCloud2 message from points array"""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_depth_optical_frame"
        
        # Define the fields for XYZRGB point cloud
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]
        
        # Create the point cloud message
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16  # 4 bytes * 4 fields (x, y, z, rgb)
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        # Convert points to bytes
        cloud_data = []
        for point in points:
            cloud_data.extend(struct.pack('fffI', point[0], point[1], point[2], int(point[3])))
        
        cloud_msg.data = bytes(cloud_data)
        
        return cloud_msg

    def publish_pointcloud(self, event):
        """Callback function to publish point clouds at 10Hz"""
        try:
            # Generate realistic point cloud data
            points = self.generate_realistic_pointcloud()
            
            # Create PointCloud2 message
            cloud_msg = self.create_pointcloud2_msg(points)
            
            # Publish the point cloud
            self.pointcloud_pub.publish(cloud_msg)
            
        except Exception as e:
            rospy.logerr(f"Error publishing point cloud: {e}")

    def generate_test_image(self):
        """Generate a test image with moving patterns"""
        # Create a colorful test pattern
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Create a moving gradient pattern
        # offset = (self.frame_counter * 5) % 255
        
        # # Generate gradient background
        # for y in range(self.height):
        #     for x in range(self.width):
        #         img[y, x, 0] = (x + offset) % 255  # Blue channel
        #         img[y, x, 1] = (y + offset) % 255  # Green channel
        #         img[y, x, 2] = ((x + y + offset) // 2) % 255  # Red channel
        
        # Add some geometric shapes
        # center_x, center_y = self.width // 2, self.height // 2
        # radius = 100 + int(50 * np.sin(self.frame_counter * 0.1))
        
        # cv2.circle(img, (center_x, center_y), radius, (255, 255, 255), 3)
        # cv2.rectangle(img, (center_x - 150, center_y - 150), 
        #              (center_x + 150, center_y + 150), (0, 255, 255), 2)
        
        # Add frame counter text
        cv2.putText(img, f"Frame: {self.frame_counter}", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        
        # Resize the image to 160x90 for testing
        img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_LINEAR)
        
        return img

    def publish_image(self, event):
        """Callback function to publish images at 30Hz"""
        try:
            # Generate test image
            cv_image = self.generate_test_image()
            
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            
            # Set header information
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = "camera_frame"
            
            # Publish the image
            self.image_pub.publish(ros_image)
            
            self.frame_counter += 1
            
        except Exception as e:
            rospy.logerr(f"Error publishing image: {e}")

    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Fake Robot ROS Node')
    parser.add_argument('--log-level', type=str, default='info',
                        choices=['debug', 'info', 'warn', 'error', 'fatal'],
                        help='Set the ROS logging level (default: info)')
    args, _ = parser.parse_known_args()

    log_level_mapping = {
        'debug': rospy.DEBUG,
        'info': rospy.INFO,
        'warn': rospy.WARN,
        'error': rospy.ERROR,
        'fatal': rospy.FATAL
    }
    rospy_log_level = log_level_mapping.get(args.log_level.lower(), rospy.INFO)

    try:
        fake_robot = FakeRobot(log_level=rospy_log_level)
        fake_robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Fake robot node terminated.")
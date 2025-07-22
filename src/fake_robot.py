#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Header
from cv_bridge import CvBridge
import struct

class FakeRobot:
    def __init__(self):
        rospy.init_node('fake_robot_node', anonymous=True)
        
        # Publisher for camera images
        self.image_pub = rospy.Publisher('/cameraF/camera/color/image_raw', Image, queue_size=1)
        
        # Publisher for point cloud data
        self.pointcloud_pub = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=1)
        
        # CV Bridge for converting between OpenCV and ROS Image messages
        self.bridge = CvBridge()
        
        # Image parameters
        self.width = 1920  # 1080P width
        self.height = 1080  # 1080P height
        self.frame_counter = 0
        
        # Set up timer for 30Hz publishing (images)
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.publish_image)
        
        # Set up timer for 10Hz publishing (point clouds)
        self.pc_timer = rospy.Timer(rospy.Duration(1.0/10.0), self.publish_pointcloud)
        
        # Subscribers for navigation topics
        self.nav_goal_sub = rospy.Subscriber('/navigation_goal', PoseStamped, self.navigation_goal_callback)
        self.cancel_nav_sub = rospy.Subscriber('/cancel_nav', Bool, self.cancel_navigation_callback)
        
        rospy.loginfo("Fake robot node started. Publishing 1080P images at 30Hz on /cameraF/camera/color/image_raw")
        rospy.loginfo("Publishing point clouds at 10Hz on /cloud_registered")
        rospy.loginfo("Subscribed to /navigation_goal and /cancel_nav topics")

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

    def cancel_navigation_callback(self, msg):
        """Callback for cancel navigation messages"""
        rospy.loginfo("*" * 30)
        rospy.loginfo("CANCEL NAVIGATION RECEIVED:")
        rospy.loginfo(f"Cancel: {msg.data}")
        rospy.loginfo("*" * 30)

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
        offset = (self.frame_counter * 5) % 255
        
        # Generate gradient background
        for y in range(self.height):
            for x in range(self.width):
                img[y, x, 0] = (x + offset) % 255  # Blue channel
                img[y, x, 1] = (y + offset) % 255  # Green channel
                img[y, x, 2] = ((x + y + offset) // 2) % 255  # Red channel
        
        # Add some geometric shapes
        center_x, center_y = self.width // 2, self.height // 2
        radius = 100 + int(50 * np.sin(self.frame_counter * 0.1))
        
        cv2.circle(img, (center_x, center_y), radius, (255, 255, 255), 3)
        cv2.rectangle(img, (center_x - 150, center_y - 150), 
                     (center_x + 150, center_y + 150), (0, 255, 255), 2)
        
        # Add frame counter text
        cv2.putText(img, f"Frame: {self.frame_counter}", (50, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        
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
    try:
        fake_robot = FakeRobot()
        fake_robot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Fake robot node terminated.")
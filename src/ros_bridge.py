#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Twist, Pose
from enum import Enum
import queue
import threading
import json

class NavigationState(Enum):
    NOT_INIT = "not_init"
    IDLE = "idle"
    NAVIGATING = "navigating"

class OperationStatus(Enum):
    IDLE = 0
    MANUAL = 1
    AUTOFILM = 2
    FOLLOWING = 3
    NAVIGATION = 4

class RosBridge:
    def __init__(self):
        self.nav_state = NavigationState.NOT_INIT
        self.operation_status = OperationStatus.IDLE  # Current robot operation status
        self.robot_info = None  # Current robot information
        self.current_pose = None  # Current robot pose
        self.nav_goal_pub = rospy.Publisher('/navigation_goal', PoseStamped, queue_size=1)
        self.cancel_nav_pub = rospy.Publisher('/cancel_nav', Bool, queue_size=1)
        self.reset_pos_pub = rospy.Publisher('/reset_robot_pos', PoseStamped, queue_size=1)
        self.cmd_vel_1_pub = rospy.Publisher('/cmd_vel_1', Twist, queue_size=1)
        self.operation_status_change_pub = rospy.Publisher('/operation_status_change', Int32, queue_size=1)
        self.goal_reached_sub = None
        self.video_sub = None
        self.operation_status_sub = rospy.Subscriber('/operation_status', Int32, self._operation_status_callback)
        self.robot_info_sub = rospy.Subscriber('/robot_info', String, self._robot_info_callback)
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self._robot_pose_callback)
        rospy.loginfo("RosBridge initialized, subscribing to operation_status, robot_info, and robot_pose")

    def _robot_pose_callback(self, msg):
        """Callback for robot pose updates"""
        self.current_pose = msg
        # rospy.logdebug(f"Robot pose updated: position=({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}), "
        #               f"orientation=({msg.orientation.x:.3f}, {msg.orientation.y:.3f}, {msg.orientation.z:.3f}, {msg.orientation.w:.3f})")

    def get_current_pose(self):
        """Get the current robot pose"""
        return self.current_pose

    def get_current_position(self):
        """Get the current robot position as dict"""
        if self.current_pose:
            return {
                "x": self.current_pose.position.x,
                "y": self.current_pose.position.y,
                "z": self.current_pose.position.z
            }
        return None

    def get_current_orientation(self):
        """Get the current robot orientation as dict"""
        if self.current_pose:
            return {
                "x": self.current_pose.orientation.x,
                "y": self.current_pose.orientation.y,
                "z": self.current_pose.orientation.z,
                "w": self.current_pose.orientation.w
            }
        return None

    def get_current_pose_dict(self):
        """Get the current robot pose as a dictionary"""
        if self.current_pose:
            return {
                "position": {
                    "x": self.current_pose.position.x,
                    "y": self.current_pose.position.y,
                    "z": self.current_pose.position.z
                },
                "orientation": {
                    "x": self.current_pose.orientation.x,
                    "y": self.current_pose.orientation.y,
                    "z": self.current_pose.orientation.z,
                    "w": self.current_pose.orientation.w
                }
            }
        return None

    def is_pose_available(self):
        """Check if robot pose is available"""
        return self.current_pose is not None

    def set_start_position(self, pose_data):
        if self.nav_state not in [NavigationState.NOT_INIT, NavigationState.IDLE]:
            raise Exception(f"Cannot set start position in {self.nav_state.value} state")

        reset_msg = PoseStamped()
        reset_msg.header.stamp = rospy.Time.now()
        reset_msg.header.frame_id = "map"
        reset_msg.pose.position.x = pose_data.get("position", {}).get("x", 0.0)
        reset_msg.pose.position.y = pose_data.get("position", {}).get("y", 0.0)
        reset_msg.pose.position.z = pose_data.get("position", {}).get("z", 0.0)
        reset_msg.pose.orientation.x = pose_data.get("orientation", {}).get("x", 0.0)
        reset_msg.pose.orientation.y = pose_data.get("orientation", {}).get("y", 0.0)
        reset_msg.pose.orientation.z = pose_data.get("orientation", {}).get("z", 0.0)
        reset_msg.pose.orientation.w = pose_data.get("orientation", {}).get("w", 1.0)
        self.reset_pos_pub.publish(reset_msg)
        self.nav_state = NavigationState.IDLE
        rospy.loginfo("Start position set, navigation state is now IDLE")

    def _operation_status_callback(self, msg):
        """Callback for operation status updates from the robot"""
        try:
            # Convert int to enum if valid
            new_status = OperationStatus(msg.data)
            if self.operation_status != new_status:
                rospy.loginfo(f"Operation status changed from {self.operation_status.name} ({self.operation_status.value}) "
                            f"to {new_status.name} ({new_status.value})")
                self.operation_status = new_status
        except ValueError:
            rospy.logwarn(f"Received invalid operation status: {msg.data}")

    def _robot_info_callback(self, msg):
        """Callback for robot information updates from the robot"""
        try:
            # Parse JSON string
            robot_info_data = json.loads(msg.data)
            self.robot_info = robot_info_data
            
            # Log robot info updates (only occasionally to avoid spam)
            if rospy.get_time() % 10 < 1.0:  # Log every 10 seconds
                data = robot_info_data.get("data", {})
                battery = data.get("battery", "unknown")
                status = data.get("status", "unknown")
                rospy.logdebug(f"Robot info updated: status={status}, battery={battery}%")
                
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to parse robot info JSON: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing robot info: {e}")

    def get_robot_info(self):
        """Get the current robot information"""
        return self.robot_info

    def get_robot_battery(self):
        """Get the current robot battery level"""
        if self.robot_info and "data" in self.robot_info:
            return self.robot_info["data"].get("battery", None)
        return None

    def get_robot_status_string(self):
        """Get the current robot status as string from robot info"""
        if self.robot_info and "data" in self.robot_info:
            return self.robot_info["data"].get("status", None)
        return None

    def get_robot_diagnostics(self):
        """Get robot diagnostic information (IMU, camera, motor)"""
        if self.robot_info and "data" in self.robot_info:
            data = self.robot_info["data"]
            return {
                "imu": data.get("imu", None),
                "camera": data.get("camera", None),
                "motor": data.get("motor", None)
            }
        return None

    def get_robot_sw_version(self):
        """Get the robot software version"""
        if self.robot_info and "data" in self.robot_info:
            return self.robot_info["data"].get("sw_version", None)
        return None

    def get_robot_accid(self):
        """Get the robot account ID"""
        if self.robot_info and "data" in self.robot_info:
            return self.robot_info["data"].get("accid", None)
        return None

    def get_operation_status(self):
        """Get the current robot operation status"""
        return self.operation_status

    def get_operation_status_name(self):
        """Get the current robot operation status name as string"""
        return self.operation_status.name

    def get_operation_status_value(self):
        """Get the current robot operation status value as integer"""
        return self.operation_status.value

    def ensure_manual_control_mode(self):
        """Ensure robot is in manual control mode, send change command if not"""
        if self.operation_status != OperationStatus.MANUAL:
            rospy.loginfo(f"Robot is currently in {self.operation_status.name} mode, changing to MANUAL")
            status_change_msg = Int32()
            status_change_msg.data = OperationStatus.MANUAL.value
            self.operation_status_change_pub.publish(status_change_msg)
            rospy.loginfo("Sent operation status change command to MANUAL (1)")
            return False  # Status was changed
        else:
            rospy.logdebug("Robot is already in MANUAL control mode")
            return True  # Already in manual mode

    def ensure_navigation_mode(self):
        """Ensure robot is in navigation mode, send change command if not"""
        if self.operation_status != OperationStatus.NAVIGATION:
            rospy.loginfo(f"Robot is currently in {self.operation_status.name} mode, changing to NAVIGATION")
            status_change_msg = Int32()
            status_change_msg.data = OperationStatus.NAVIGATION.value
            self.operation_status_change_pub.publish(status_change_msg)
            rospy.loginfo("Sent operation status change command to NAVIGATION (4)")
            return False  # Status was changed
        else:
            rospy.logdebug("Robot is already in NAVIGATION mode")
            return True  # Already in navigation mode

    def change_operation_status(self, new_status):
        """Send a command to change the robot operation status"""
        if isinstance(new_status, OperationStatus):
            status_value = new_status.value
            status_name = new_status.name
        elif isinstance(new_status, int):
            try:
                status_enum = OperationStatus(new_status)
                status_value = new_status
                status_name = status_enum.name
            except ValueError:
                rospy.logerr(f"Invalid operation status value: {new_status}")
                return False
        else:
            rospy.logerr(f"Invalid operation status type: {type(new_status)}")
            return False
        
        status_change_msg = Int32()
        status_change_msg.data = status_value
        self.operation_status_change_pub.publish(status_change_msg)
        rospy.loginfo(f"Sent operation status change command to {status_name} ({status_value})")
        return True

    def start_navigation(self, goal_data, completion_callback):
        if self.nav_state != NavigationState.IDLE:
            raise Exception(f"Cannot start navigation in {self.nav_state.value} state")

        # Ensure robot is in navigation mode before starting navigation
        self.ensure_navigation_mode()
        # Give a brief moment for the status change to take effect
        rospy.sleep(0.1)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_data.get("position", {}).get("x", 0.0)
        goal_msg.pose.position.y = goal_data.get("position", {}).get("y", 0.0)
        goal_msg.pose.position.z = goal_data.get("position", {}).get("z", 0.0)
        goal_msg.pose.orientation.x = goal_data.get("orientation", {}).get("x", 0.0)
        goal_msg.pose.orientation.y = goal_data.get("orientation", {}).get("y", 0.0)
        goal_msg.pose.orientation.z = goal_data.get("orientation", {}).get("z", 0.0)
        goal_msg.pose.orientation.w = goal_data.get("orientation", {}).get("w", 1.0)
        
        self.nav_goal_pub.publish(goal_msg)
        self.nav_state = NavigationState.NAVIGATING
        self.goal_reached_sub = rospy.Subscriber('/goal_reached', Bool, self._goal_reached_callback, callback_args=completion_callback)
        rospy.loginfo("Navigation started in NAVIGATION mode")

    def _goal_reached_callback(self, msg, completion_callback):
        if msg.data:
            self.nav_state = NavigationState.IDLE
            if self.goal_reached_sub:
                self.goal_reached_sub.unregister()
                self.goal_reached_sub = None
            rospy.loginfo("Goal reached, navigation state is now IDLE")
            completion_callback()

    def stop_navigation(self):
        if self.nav_state != NavigationState.NAVIGATING:
            raise Exception(f"Cannot stop navigation in {self.nav_state.value} state")
        
        cancel_msg = Bool()
        cancel_msg.data = True
        self.cancel_nav_pub.publish(cancel_msg)
        self.nav_state = NavigationState.IDLE
        if self.goal_reached_sub:
            self.goal_reached_sub.unregister()
            self.goal_reached_sub = None
        rospy.loginfo("Navigation stopped, state is now IDLE")

    def get_image(self):
        img_queue = queue.Queue(maxsize=1)
        
        def image_callback(msg):
            try:
                img_queue.put_nowait(msg)
            except queue.Full:
                pass # Ignore if queue is full
            finally:
                sub.unregister()
                rospy.logdebug("Image callback executed, subscriber unregistered")

        sub = rospy.Subscriber('/cameraF/camera/color/image_raw', Image, image_callback)
        
        try:
            return img_queue.get(timeout=1.0)
        except queue.Empty:
            raise Exception("Failed to get image within 1 second timeout")

    def get_point_cloud(self):
        pc_queue = queue.Queue(maxsize=1)

        def pc_callback(msg):
            try:
                pc_queue.put_nowait(msg)
            except queue.Full:
                pass
            finally:
                sub.unregister()
                rospy.logdebug("Point cloud callback executed, subscriber unregistered")
        
        sub = rospy.Subscriber('/cloud_registered', PointCloud2, pc_callback)

        try:
            return pc_queue.get(timeout=1.0)
        except queue.Empty:
            raise Exception("Failed to get point cloud within 1 second timeout")

    def setup_video_stream(self):
        if self.video_sub:
            rospy.logwarn("Video stream already set up, stopping previous subscription")
            self.stop_video_stream()

        frame_id = 0
        vs_queue = queue.Queue(maxsize=1)
        def frame_callback(msg):
            nonlocal frame_id
            try:
                if vs_queue.full():
                    vs_queue.get_nowait()
                vs_queue.put_nowait((msg, frame_id))
                frame_id = frame_id + 1
            except Exception as e:
                rospy.logerr(f"Error in video frame callback: {e}")

        self.video_sub = rospy.Subscriber('/cameraF/camera/color/image_raw', Image, frame_callback)
        rospy.loginfo("Video stream subscription setup")
        return vs_queue

    def stop_video_stream(self):
        if self.video_sub:
            self.video_sub.unregister()
            self.video_sub = None
            rospy.loginfo("Video stream subscription stopped")

    def publish_cmd_vel_1(self, data):
        """Publish manual control velocity commands to /cmd_vel_1 topic"""
        # Ensure robot is in manual control mode before sending commands
        self.ensure_manual_control_mode()
        
        twist_msg = Twist()
        
        # Extract linear velocities
        twist_msg.linear.x = data.get("x", 0.0)
        twist_msg.linear.y =  0.0
        twist_msg.linear.z = 0.0
        
        # Extract angular velocities
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = data.get("z", 0.0)

        # Publish the command
        self.cmd_vel_1_pub.publish(twist_msg)
        # rospy.logdebug(f"Published cmd_vel_1: linear=[{twist_msg.linear.x:.3f}, {twist_msg.linear.y:.3f}, {twist_msg.linear.z:.3f}], "
        #               f"angular=[{twist_msg.angular.x:.3f}, {twist_msg.angular.y:.3f}, {twist_msg.angular.z:.3f}]")


#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from enum import Enum
import queue
import threading

class NavigationState(Enum):
    NOT_INIT = "not_init"
    IDLE = "idle"
    NAVIGATING = "navigating"

class RosBridge:
    def __init__(self):
        self.nav_state = NavigationState.NOT_INIT
        self.nav_goal_pub = rospy.Publisher('/navigation_goal', PoseStamped, queue_size=1)
        self.cancel_nav_pub = rospy.Publisher('/cancel_nav', Bool, queue_size=1)
        self.reset_pos_pub = rospy.Publisher('/reset_robot_pos', PoseStamped, queue_size=1)
        self.goal_reached_sub = None
        self.video_sub = None

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

    def start_navigation(self, goal_data, completion_callback):
        if self.nav_state != NavigationState.IDLE:
            raise Exception(f"Cannot start navigation in {self.nav_state.value} state")

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
        rospy.loginfo("Navigation started")

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


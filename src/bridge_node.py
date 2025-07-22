#!/usr/bin/env python3

import rospy
import threading
import json
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import base64
import numpy as np
import io
import time
from PIL import Image as PILImage
import asyncio
import websockets
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

ROBOT_CODE = "robot3234324232"
class WebSocketManager:
    def __init__(self):
        self.loop = None
        self.control_ws = None
        self.video_ws = None
        self.running = False
        
    def start(self, ros_bridge):
        """Start the asyncio event loop in a separate thread"""
        self.ros_bridge = ros_bridge
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()
        
    def _run_loop(self):
        """Run the asyncio event loop"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._manage_connections())
        except Exception as e:
            rospy.logerr(f"WebSocket manager error: {e}")
        finally:
            self.loop.close()
            
    async def _manage_connections(self):
        """Manage both WebSocket connections"""
        self.running = True
        
        # Start both connections concurrently
        control_task = asyncio.create_task(self._control_connection())
        video_task = asyncio.create_task(self._video_connection())
        
        # Wait for both tasks
        await asyncio.gather(control_task, video_task, return_exceptions=True)
        
    async def _control_connection(self):
        """Handle control WebSocket connection"""
        uri = "wss://gj-test.mnt-aihub.com:8443/control"
        headers = {
            "client_type": "robot",
            "client_id": ROBOT_CODE
        }
        
        while self.running:
            try:
                async with websockets.connect(uri, extra_headers=headers) as websocket:
                    self.control_ws = websocket
                    rospy.loginfo("Control WebSocket connected")
                    
                    while self.running:
                        try:
                            message = await websocket.recv()
                            rospy.loginfo(f"Received from control ws: {message}")
                            # Publish to ROS topic
                            self.ros_bridge.publish_control_message(message)
                        except websockets.exceptions.ConnectionClosed:
                            rospy.logwarn("Control WebSocket connection closed")
                            break
                        except Exception as e:
                            rospy.logerr(f"Control WS recv error: {e}")
                            break
                            
            except Exception as e:
                rospy.logerr(f"Control WebSocket connection error: {e}")
                if self.running:
                    await asyncio.sleep(5)  # Wait before reconnecting
                    
        self.control_ws = None
        
    async def _video_connection(self):
        """Handle video WebSocket connection"""
        uri = "wss://gj-test.mnt-aihub.com:8443/video"
        headers = {
            "client_type": "robot",
            "client_id": ROBOT_CODE
        }
        
        while self.running:
            try:
                async with websockets.connect(uri, extra_headers=headers) as websocket:
                    self.video_ws = websocket
                    rospy.loginfo("Video WebSocket connected")
                    
                    # Keep connection alive (video is mainly for sending)
                    await websocket.wait_closed()
                    
            except Exception as e:
                rospy.logerr(f"Video WebSocket connection error: {e}")
                if self.running:
                    await asyncio.sleep(5)  # Wait before reconnecting
                    
        self.video_ws = None
        
    def send_control_message(self, message):
        """Send message to control WebSocket"""
        if self.loop and self.control_ws:
            asyncio.run_coroutine_threadsafe(
                self.control_ws.send(message), 
                self.loop
            )
        else:
            rospy.logwarn("Control WebSocket not connected")
            
    def send_video_message(self, message):
        """Send message to video WebSocket"""
        if self.loop and self.video_ws:
            asyncio.run_coroutine_threadsafe(
                self.video_ws.send(message), 
                self.loop
            )
        else:
            rospy.logwarn("Video WebSocket not connected")
            
    def stop(self):
        """Stop the WebSocket manager"""
        self.running = False
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._close_connections(), self.loop)
            
    async def _close_connections(self):
        """Close all WebSocket connections"""
        if self.control_ws:
            await self.control_ws.close()
        if self.video_ws:
            await self.video_ws.close()

class ROSWebSocketBridge:
    def __init__(self):
        rospy.init_node('ros_websocket_bridge', anonymous=True)
        
        # Initialize WebSocket manager
        self.ws_manager = WebSocketManager()
        self.ws_manager.start(self)
        
        # Wait a bit for connections to establish
        rospy.sleep(2)
        
        # Initialize ROS subscribers
        self.init_subscribers()
        
        self.init_publishers()

        ## request task ids 
        self.init_request_task_ids()
        
        
    def init_request_task_ids(self):
        """Request task IDs from the control WebSocket"""
        task_ids = [
            "request_image_capture",
            "request_pointcloud_capture",
            "request_video_start",
            "request_video_stop",
            "request_start_navigation",
            "request_stop_navigation",
            "request_start_point",
            "request_robot_info"
        ]
        
        self.task_id_map = {task_id: None for task_id in task_ids}
        self.video_streaming = True
        
        # Start heartbeat timer
        self.start_heartbeat()
        
    def start_heartbeat(self):
        """Start the heartbeat timer"""
        self.heartbeat_timer = rospy.Timer(rospy.Duration(5.0), self.send_heartbeat)
        rospy.loginfo("Heartbeat timer started - sending every 5 seconds")
        
    def send_heartbeat(self, event):
        """Send heartbeat message to control WebSocket"""
        heartbeat_data = {
            "title": "response_heartbeat",
            "timestamp": int(time.time() * 1000),
            "guid": "uuid-3456-req-vp",
            "targetType": "robot",
            "targetId": "heartbeat",
            "clientId": "heartbeat",
            "data": {
                "robotCode": ROBOT_CODE,
                "result": "ok"
            }
        }
        
        json_data = json.dumps(heartbeat_data)
        self.ws_manager.send_control_message(json_data)
        rospy.logdebug("Heartbeat sent to control WebSocket")
        
    def init_subscribers(self):
        """Initialize ROS topic subscribers"""
        rospy.Subscriber('/cameraF/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.pointcloud_callback)
        
        
    def init_publishers(self):
        """Initialize ROS topic publishers"""
        self.nav_goal_pub = rospy.Publisher('/navigation_goal', PoseStamped, queue_size=1)
        self.cancel_nav_pub = rospy.Publisher('/cancel_nav', Bool, queue_size=1)
        self.reset_pos_pub = rospy.Publisher('/reset_robot_pos', PoseStamped, queue_size=1)
        
    def pointcloud_callback(self, msg):
        """Handle PointCloud2 messages - send to control WebSocket if requested"""
        if self.task_id_map["request_pointcloud_capture"] is not None:
            # Prepare fields info
            fields_info = {}
            for field in msg.fields:
                fields_info[f"{field.name}_size"] = field.count * np.dtype(field.datatype).itemsize if hasattr(np, 'dtype') else 4

            control_data = {
                "title": "response_pointcloud_capture",
                "timestamp": int(time.time() * 1000),
                "guid": "uuid-5678-req-pc",
                "targetType": "robot",
                "targetId": "1",
                "clientId": ROBOT_CODE,
                "taskHistoryCode": self.task_id_map["request_pointcloud_capture"],
                "data": {
                    "robotCode": ROBOT_CODE,
                    "result": "ok",
                    "height": 1,
                    "width": 16200,
                    "fields": fields_info,
                    "is_bigendian": bool(msg.is_bigendian),
                    "point_step": msg.point_step,
                    "row_step": msg.row_step,
                    "data": msg.data.hex(),
                    "is_dense": bool(msg.is_dense)
                }
            }
            json_control_data = json.dumps(control_data)
            self.ws_manager.send_control_message(json_control_data)
            rospy.logdebug("PointCloud sent to control WebSocket, task ID: " + str(self.task_id_map["request_pointcloud_capture"]))
            self.task_id_map["request_pointcloud_capture"] = None

    def image_callback(self, msg):
        """Handle image messages - send to video WebSocket"""
        # Convert ROS image to numpy array
        if msg.encoding == "rgb8":
            image_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        elif msg.encoding == "bgr8":
            image_array = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            image_array = image_array[..., ::-1]  # Convert BGR to RGB
        else:
            rospy.logwarn(f"Unsupported encoding: {msg.encoding}")
            return
            
        # Convert numpy array to PIL Image
        image = PILImage.fromarray(image_array, 'RGB')
        
        # Convert image to JPEG format and encode to base64
        img_buffer = io.BytesIO()
        image.save(img_buffer, format='JPEG', quality=50)
        img_buffer.seek(0)
        img_base64 = base64.b64encode(img_buffer.getvalue()).decode('utf-8')
        
        if self.video_streaming:
            data = {
                "guid": "uuid-9012-req-vs",
                "timestamp": int(time.time() * 1000),
                "frame": f"data:image/jpg;base64,{img_base64}"
            }
            json_data = json.dumps(data)
            
            # Send to video WebSocket
            self.ws_manager.send_video_message(json_data)
        
        if self.task_id_map["request_image_capture"] is not None:
            # Send to control WebSocket if task ID is set
            control_data = {
                "title": "response_image_capture",
                "timestamp": int(time.time() * 1000),
                "guid": "uuid-1234-req-img",
                "targetType": "client",
                "targetId": "1",
                "clientId": ROBOT_CODE,
                "taskHistoryCode": self.task_id_map["request_image_capture"],  # You may want to set this dynamically
                "data": {
                    "robotCode": ROBOT_CODE,
                    "result": "ok",
                    "height": msg.height,
                    "width": msg.width,
                    "encoding": msg.encoding,
                    "is_bigendian": msg.is_bigendian,
                    "step": msg.step,
                    "data": msg.data.hex()
                }
            }
            json_control_data = json.dumps(control_data)
            self.ws_manager.send_control_message(json_control_data)
            rospy.logdebug("Image sent to control WebSocket, task ID: " + str(self.task_id_map["request_image_capture"]))
            self.task_id_map["request_image_capture"] = None
            
            
    def publish_control_message(self, message):
        """Handle incoming control messages from WebSocket"""
        try:
            msg_json = json.loads(message)
            title = msg_json.get("title")
            task_id = msg_json.get("taskHistoryCode")
            guid = msg_json.get("guid")
            if title and task_id and title.startswith("request_"):
                self.task_id_map[title] = task_id
                rospy.loginfo(f"Saved task ID for {title}: {task_id}")
            if title == "request_video_start":
                self.video_streaming = True
                rospy.loginfo("Video streaming started")
                response_data = {
                    "title": "response_video_start",
                    "timestamp": int(time.time() * 1000),
                    "guid": guid,
                    "targetType": "robot",
                    "targetId": "1",
                    "clientId": ROBOT_CODE,
                    "taskHistoryCode": task_id,
                    "data": {
                        "robotCode": ROBOT_CODE,
                        "result": "ok"
                    }
                }
                self.ws_manager.send_control_message(json.dumps(response_data))
                self.task_id_map["request_video_start"] = None
            
            elif title == "request_video_stop":
                self.video_streaming = False
                rospy.loginfo("Video streaming stopped")
                response_data = {
                    "title": "response_video_stop",
                    "timestamp": int(time.time() * 1000),
                    "guid": guid,
                    "targetType": "robot",
                    "targetId": "1",
                    "clientId": ROBOT_CODE,
                    "taskHistoryCode": task_id,
                    "data": {
                        "robotCode": ROBOT_CODE,
                        "result": "ok"
                    }
                }
                self.ws_manager.send_control_message(json.dumps(response_data))
                self.task_id_map["request_video_stop"] = None
                
            elif title == "request_start_navigation":    
                rospy.loginfo("Start navigation requested")
                # Parse navigation goal from message
                position = msg_json.get("data", {}).get("position", {})
                orientation = msg_json.get("data", {}).get("orientation", {})
                goal_msg = PoseStamped()
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.header.frame_id = "map"
                goal_msg.pose.position.x = position.get("x", 0.0)
                goal_msg.pose.position.y = position.get("y", 0.0)
                goal_msg.pose.position.z = position.get("z", 0.0)
                goal_msg.pose.orientation.x = orientation.get("x", 0.0)
                goal_msg.pose.orientation.y = orientation.get("y", 0.0)
                goal_msg.pose.orientation.z = orientation.get("z", 0.0)
                goal_msg.pose.orientation.w = orientation.get("w", 1.0)
                self.nav_goal_pub.publish(goal_msg)
                rospy.loginfo(f"Published navigation goal: position={position}, orientation={orientation}")
                
                response_data = {
                    "title": "response_start_navigation",
                    "timestamp": int(time.time() * 1000),
                    "guid": guid,
                    "targetType": "robot",
                    "targetId": "1",
                    "clientId": ROBOT_CODE,
                    "taskHistoryCode": task_id,
                    "requestType": msg_json.get("requestType"), 
                    "data": {
                        "robotCode": ROBOT_CODE,
                        "result": "ok" ## TODO: add failure details if needed, like if navigation fails
                    }
                }
                
                self.ws_manager.send_control_message(json.dumps(response_data))
                self.task_id_map["request_start_navigation"] = None
                
            elif title == "request_stop_navigation":    
                rospy.loginfo("Stop navigation requested")

                response_data = {
                    "title": "response_stop_navigation",
                    "timestamp": int(time.time() * 1000),
                    "guid": guid,
                    "targetType": "robot",
                    "targetId": "1",
                    "clientId": ROBOT_CODE,
                    "taskHistoryCode": task_id,
                    "requestType": msg_json.get("requestType"), 
                    "data": {
                        "robotCode": ROBOT_CODE,
                        "result": "ok"
                    }
                }
                # Cancel current navigation goal in move_base
                cancel_msg = Bool()
                cancel_msg.data = True
                self.cancel_nav_pub.publish(cancel_msg)
                self.ws_manager.send_control_message(json.dumps(response_data))
                self.task_id_map["request_stop_navigation"] = None
                
            elif title == "request_robot_info":
                rospy.loginfo("Robot info requested")
                response_data = {
                    "title": "response_robot_info",
                    "timestamp": int(time.time() * 1000),
                    "guid": guid,
                    "targetType": "client",
                    "targetId": "1",
                    "clientId": ROBOT_CODE,
                    "taskHistoryCode": task_id,
                    "data": {
                        "robotCode": ROBOT_CODE,
                        "accid": "PF_TRON1A_042",
                        "sw_version": "robot-tron1-2.0.10.20241111103012",
                        "imu": "OK",
                        "camera": "OK",
                        "motor": "OK",
                        "battery": 95,
                        "status": "WALK",
                        "gridPosition": {
                            "x": 100,
                            "y": 200,
                            "z": 0
                        },
                        "result": "ok"
                    }
                }
                self.ws_manager.send_control_message(json.dumps(response_data))
                self.task_id_map["request_robot_info"] = None
                
            elif title == "request_start_point":
                # Parse position and orientation from message
                position = msg_json.get("data", {}).get("position", {})
                orientation = msg_json.get("data", {}).get("orientation", {})

                reset_msg = PoseStamped()
                reset_msg.header.stamp = rospy.Time.now()
                reset_msg.header.frame_id = "map"
                reset_msg.pose.position.x = position.get("x", 0.0)
                reset_msg.pose.position.y = position.get("y", 0.0)
                reset_msg.pose.position.z = position.get("z", 0.0)
                reset_msg.pose.orientation.x = orientation.get("x", 0.0)
                reset_msg.pose.orientation.y = orientation.get("y", 0.0)
                reset_msg.pose.orientation.z = orientation.get("z", 0.0)
                reset_msg.pose.orientation.w = orientation.get("w", 1.0)

                # Publish to /reset_robot_pos topic
                self.reset_pos_pub.publish(reset_msg)
                rospy.loginfo(f"Published reset position: position={position}, orientation={orientation}")

                response_data = {
                    "title": "response_start_point",
                    "timestamp": int(time.time() * 1000),
                    "guid": guid,
                    "targetType": "client",
                    "targetId": "1",
                    "clientId": ROBOT_CODE,
                    "data": {
                        "robotCode": ROBOT_CODE,
                        "result": "ok"
                    }
                }
                self.ws_manager.send_control_message(json.dumps(response_data))
                self.task_id_map["request_start_point"] = None
            # You can add further processing here if needed
        except Exception as e:
            rospy.logerr(f"Failed to process control message: {e}")

    def send_control_to_ws(self, msg):
        """Send ROS topic message to control WebSocket"""
        self.ws_manager.send_control_message(msg.data)
        
    def run(self):
        """Main run loop"""
        rospy.loginfo("ROS WebSocket Bridge started")
        try:
            rospy.spin()
        finally:
            self.ws_manager.stop()

if __name__ == '__main__':
    try:
        bridge = ROSWebSocketBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
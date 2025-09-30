#!/usr/bin/env python3

import asyncio
import websockets
import json
import time
import io
from PIL import Image as PILImage
import numpy as np
from ros_bridge import RosBridge
from sensor_msgs import msg as RosSensorMsg
import contextvars
import functools
import logging
import queue
import uuid

# Configure logging
class CustomFormatter(logging.Formatter):
    # Define color codes
    COLORS = {
        'DEBUG': '\033[94m',    # Blue
        'INFO': '\033[92m',     # Green
        'WARNING': '\033[93m',  # Yellow
        'ERROR': '\033[91m',    # Red
        'CRITICAL': '\033[95m', # Magenta
        'RESET': '\033[0m'
    }

    def format(self, record):
        if hasattr(record, 'guid') and record.guid:
            self._style._fmt = '[%(asctime)s][%(levelname)s][%(guid)s] - %(message)s'
        else:
            self._style._fmt = '[%(asctime)s][%(levelname)s] - %(message)s'
        log_fmt = super().format(record)
        color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
        reset = self.COLORS['RESET']
        return f"{color}{log_fmt}{reset}"

logger = logging.getLogger('websocket_client')
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(CustomFormatter())
logger.addHandler(handler)

# WebSocket logging
# logger_w = logging.getLogger("websockets")
# logger_w.setLevel(logging.DEBUG)
# logger_w.addHandler(logging.StreamHandler())

def set_log_level(level_str: str):
    """Sets the logging level for the websocket_client logger."""
    level_map = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'warn': logging.WARNING,
        'error': logging.ERROR,
        'critical': logging.CRITICAL,
        'fatal': logging.CRITICAL,
    }
    level = level_map.get(level_str.lower(), logging.INFO)
    logger.setLevel(level)
    # This message will only appear if the new level is INFO or DEBUG.
    logger.info(f"WebSocket client log level set to {logging.getLevelName(level)}")

async def to_thread(func, /, *args, **kwargs):
    loop = asyncio.get_running_loop()
    ctx = contextvars.copy_context()
    func_call = functools.partial(ctx.run, func, *args, **kwargs)
    return await loop.run_in_executor(None, func_call)

DEFAULT_VIDEO_URI = "ws://localhost:8444/video"
DEFAULT_CONTROL_URI = "ws://localhost:8443/control"

class WebsocketClient:
    def __init__(self, ros_bridge: RosBridge, robot_code: str, control_uri: str = None, video_uri: str = None):
        self.ros_bridge = ros_bridge
        self.robot_code = robot_code
        self.control_uri = control_uri or DEFAULT_CONTROL_URI
        self.video_uri = video_uri or DEFAULT_VIDEO_URI
        self.control_ws = None
        self.video_ws = None
        self.video_stream_task = None
        self.video_stream_queue: queue.Queue = None
        self.headers = {"client_type": "robot", "client_id": self.robot_code}

        logger.info(f"Robot code: {self.robot_code}")
        logger.info(f"Control URI: {self.control_uri}")
        logger.info(f"Video URI: {self.video_uri}")

    async def connect(self):
        await asyncio.gather(
            self._connect_control_websocket(self.control_uri, self.headers, self._handle_control_messages),
        )

    async def _connect_control_websocket(self, uri, headers, message_handler):
        while True:
            logger.info(f"Connecting to control WebSocket...")
            try:
                async with websockets.connect(uri, extra_headers=headers, ping_interval=20) as websocket:
                    self.control_ws = websocket
                    logger.info("Control WebSocket connected")
                    asyncio.create_task(self._send_heartbeat())
                    await message_handler(websocket)
            except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError) as e:
                logger.warning(f"Control WebSocket connection closed: {e}")
            except Exception as e:
                logger.error(f"Error with Control WebSocket: {e}")
            finally:
                self.control_ws = None
                logger.info("Reconnecting Control WebSocket in 2 seconds...")
                await asyncio.sleep(2)

    async def _connect_video_websocket(self, uri, headers, message_handler):
        while True:
            logger.info(f"Connecting to video WebSocket...")
            try:
                async with websockets.connect(uri, extra_headers=headers, ping_interval=20) as websocket:
                    self.video_ws = websocket
                    logger.info("Video WebSocket connected")
                    await message_handler(websocket)
            except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError) as e:
                logger.warning(f"Video WebSocket connection closed: {e}")
            except asyncio.CancelledError:
                logger.info("Video WebSocket connection cancelled, not reconnecting.")
                break
            except Exception as e:
                logger.error(f"Error with Video WebSocket: {e}")
            finally:
                self.video_ws = None

            logger.info("Reconnecting Video WebSocket in 0.5 seconds...")
            await asyncio.sleep(0.5)

    async def _send_heartbeat(self):
        while self.control_ws and self.control_ws.open:
            try:
                heartbeat_data = {
                    "title": "response_heartbeat",
                    "timestamp": int(time.time() * 1000),
                    "guid": uuid.uuid4().hex,
                    "targetType": "robot",
                    "targetId": "heartbeat",
                    "clientId": self.robot_code,
                    "data": {"robotCode": self.robot_code, "result": "ok"}
                }
                await self.control_ws.send(json.dumps(heartbeat_data))
                await asyncio.sleep(5)
            except websockets.exceptions.ConnectionClosed:
                break

    async def _handle_control_messages(self, websocket):
        async for message in websocket:
            try:
                msg_json = json.loads(message)
                logger.debug(f"Received control message: {msg_json}")
                robot_code = msg_json.get("targetId", "None")
                if robot_code != self.robot_code:
                    logger.warning(f"Received message for different robot: {robot_code}, ignoring")
                    continue
                await self._process_command(msg_json)
            except json.JSONDecodeError:
                logger.warning(f"Received non-JSON control message: {message}")
            except Exception as e:
                logger.error(f"Error processing control message: {e}")
    
    async def _handle_video_messages(self, websocket):
        async for message in websocket:
            try:
                msg_json = json.loads(message)
                logger.debug(f"Received video message: {msg_json}")
            except json.JSONDecodeError:
                logger.warning(f"Received non-JSON video message: {message}")
            except Exception as e:
                logger.error(f"Error processing video message: {e}")

    async def _process_command(self, msg):
        title = msg.get("title")
        if not title:
            logger.warning("Received message without title, ignoring")
            return

        handler_map = {
            "request_image_capture": self.handle_image_capture,
            "request_pointcloud_capture": self.handle_pointcloud_capture,
            "request_video_start": self.handle_video_start,
            "request_video_stop": self.handle_video_stop,
            "request_start_navigation": self.handle_start_navigation,
            "request_stop_navigation": self.handle_stop_navigation,
            "request_start_point": self.handle_set_start_position,
            "request_robot_info": self.handle_robot_info,
            "request_robot_manual_navigation": self.handle_robot_manual_navigation,
            "request_robot_positioning": self.handle_robot_positioning,
            "request_standup_start": self.handle_standup_start,
            "request_sitdown_start": self.handle_sitdown_start,
        }

        handler = handler_map.get(title)
        if handler:
            logger.info(f"Handling command: {title}", extra={'guid': msg.get('guid')})
            await handler(msg)
        else:
            logger.warning(f"No handler for command: {title}")

    async def handle_image_capture(self, msg):
        try:
            image_msg = await to_thread(self.ros_bridge.get_image)
            image_buffer = await to_thread(self._ros_image_to_jpeg, image_msg, quality=95)
            response = {"title": "response_image_capture"}
            response["data"] = {
                "result": "ok",
                # "height": image_msg.height,
                # "width": image_msg.width,
                # "encoding": image_msg.encoding,
                # "is_bigendian": image_msg.is_bigendian,
                # "step": image_msg.step,
                "data": image_buffer.getvalue().hex()
            }
            logger.info(f"Image captured successfully", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_image_capture: {e}", extra={'guid': msg.get('guid')})
            response["data"] = {"result": "error", "message": str(e)}
        
        await self.send_control_response(msg, response)

    def _ros_image_to_jpeg(self, ros_image, quality):
        img_buffer = io.BytesIO()

        if type(ros_image) == RosSensorMsg.CompressedImage:
            img_buffer.write(ros_image.data)
        elif type(ros_image) == RosSensorMsg.Image:
            if ros_image.encoding == "rgb8":
                image_array = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
            elif ros_image.encoding == "bgr8":
                image_array = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
                image_array = image_array[..., ::-1]
            else:
                raise ValueError(f"Unsupported image encoding: {ros_image.encoding}")
            
            image = PILImage.fromarray(image_array, 'RGB')
            # Rotate and resize if image from realsense
            if image.width < 1920:
                image = image.transpose(PILImage.ROTATE_180)
                image = image.resize((1920, 1080))
            image.save(img_buffer, format='JPEG', quality=quality)
        else:
            raise ValueError(f"Unsupported ROS image type: {type(ros_image)}")

        return img_buffer

    async def handle_pointcloud_capture(self, msg):
        try:
            pc_msg = await to_thread(self.ros_bridge.get_point_cloud)

            def convert_fields(fields):
                """
                Converts a list of PointField objects to a dictionary with field sizes.
                """
                datatype_to_size = {
                    RosSensorMsg.PointField.INT8: 1, RosSensorMsg.PointField.UINT8: 1,
                    RosSensorMsg.PointField.INT16: 2, RosSensorMsg.PointField.UINT16: 2,
                    RosSensorMsg.PointField.INT32: 4, RosSensorMsg.PointField.UINT32: 4,
                    RosSensorMsg.PointField.FLOAT32: 4, RosSensorMsg.PointField.FLOAT64: 8,
                }
                fields_info_dict = {}
                for field in fields:
                    if field.datatype in datatype_to_size:
                        size = field.count * datatype_to_size[field.datatype]
                        fields_info_dict[f"{field.name}_size"] = size
                return fields_info_dict

            fields_info = convert_fields(pc_msg.fields)
            response = {"title": "response_pointcloud_capture"}
            response["data"] = {
                "result": "ok",
                "height": pc_msg.height,
                "width": pc_msg.width,
                "fields": fields_info,
                "is_bigendian": pc_msg.is_bigendian,
                "point_step": pc_msg.point_step,
                "row_step": pc_msg.row_step,
                "data": pc_msg.data.hex(),
                "is_dense": pc_msg.is_dense
            }
            logger.info("Point cloud captured successfully", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_pointcloud_capture: {e}", extra={'guid': msg.get('guid')})
            response["data"] = {"result": "error", "message": str(e)}

        await self.send_control_response(msg, response)

    async def handle_video_start(self, msg):
        try:
            response = {"title": "response_video_start"}
            if self.video_stream_task:
                response["data"] = {"result": "error", "message": "Video stream already running"}
                logger.warning("Video stream already running, cannot start again", extra={'guid': msg.get('guid')})
            else:
                self.video_stream_queue = await to_thread(self.ros_bridge.setup_video_stream)
                self.video_stream_task = asyncio.create_task(self._video_stream_task())
                logger.info("Video stream started", extra={'guid': msg.get('guid')})
                response["data"] = {"result": "ok"}
        except Exception as e:
            logger.error(f"Error in handle_video_start: {e}", extra={'guid': msg.get('guid')})
            response["data"] = {"result": "error", "message": str(e)}
        
        await self.send_control_response(msg, response)

    def _get_video_frame(self):
        if not self.video_stream_queue:
            raise Exception("Video stream queue is not initialized")
        
        try:
            ros_image, frame_id = self.video_stream_queue.get(timeout=1.0)
            img_buffer = self._ros_image_to_jpeg(ros_image, quality=95)
            return (img_buffer, frame_id)
            
        except queue.Empty:
            raise Exception("Failed to get video frame within 1 second timeout")

    async def _stream_video_frames(self):
        while True:
            try:
                img_buffer, frame_id = await to_thread(self._get_video_frame)
                if self.video_ws and self.video_ws.open:
                    await self.video_ws.send(img_buffer.getvalue())
                    logger.debug(f"Sent video frame: {frame_id}, size: {len(img_buffer.getvalue())} bytes")
            except websockets.exceptions.ConnectionClosed as e:
                logger.warning(f"Video WebSocket connection closed. Reason: {e}")
            except websockets.exceptions.ConnectionClosedOK:
                logger.warning("Video WebSocket connection closed normally.")
            except websockets.exceptions.ConnectionClosedError as e:
                logger.warning(f"Video WebSocket connection closed with error. Reason: {e}")         

    async def _video_stream_task(self):
        try:
            await asyncio.gather(
                self._connect_video_websocket(self.video_uri, self.headers, self._handle_video_messages),
                self._stream_video_frames()
            )
        except asyncio.CancelledError:
            logger.debug("Video stream task cancelled")
        except Exception as e:
            logger.error(f"Error in video stream task: {e}")
        finally:
            logger.debug("Cleanning up video stream task...")
            await to_thread(self.ros_bridge.stop_video_stream)
            self.video_stream_queue = None
            self.video_stream_task = None
            logger.debug("Video stream task cleaned up")

    async def handle_video_stop(self, msg):
        try:
            response = {"title": "response_video_stop"}
            if self.video_stream_task:
                self.video_stream_task.cancel()
                logger.info("Video stream stopped", extra={'guid': msg.get('guid')})
                response["data"] = {"result": "ok"}
            else:
                logger.warning("Video stream not running, cannot stop", extra={'guid': msg.get('guid')})
                response["data"] = {"result": "error", "message": "Video stream not running"}
        except Exception as e:
            logger.error(f"Error in handle_video_stop: {e}", extra={'guid': msg.get('guid')})
            response["data"] = {"result": "error", "message": str(e)}
        
        await self.send_control_response(msg, response)

    async def handle_start_navigation(self, msg):
        try:
            goal_data = msg.get("data", {})

            # Construct the goal reached callback
            loop = asyncio.get_event_loop()
            def _on_goal_reached():
                response = {
                    "title": "response_arrive_navigation",
                    "data": {"result": "ok"}
                }
                asyncio.run_coroutine_threadsafe(self.send_control_response(msg, response), loop)

            await to_thread(self.ros_bridge.start_navigation, goal_data, _on_goal_reached)
            logger.info(f"Started navigation with goal: {goal_data}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_start_navigation", 
                "data": {"result": "ok"}
            }
        except Exception as e:
            logger.error(f"Error in handle_start_navigation: {e}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_start_navigation", 
                "data": {"result": "error", "message": str(e)}
            }
        
        await self.send_control_response(msg, response)

    async def handle_stop_navigation(self, msg):
        try:
            await to_thread(self.ros_bridge.stop_navigation)
            logger.info(f"Stopped navigation", extra={'guid': msg.get('guid')})
            response = {"title": "response_stop_navigation", "data": {"result": "ok"}}
        except Exception as e:
            logger.error(f"Error in handle_stop_navigation: {e}", extra={'guid': msg.get('guid')})
            response = {"title": "response_stop_navigation", "data": {"result": "error", "message": str(e)}}
        
        await self.send_control_response(msg, response)

    async def handle_set_start_position(self, msg):
        try:
            pose_data = msg.get("data", {})
            await to_thread(self.ros_bridge.set_start_position, pose_data)
            logger.info(f"Set start position: {pose_data}", extra={'guid': msg.get('guid')})
            response = {"title": "response_start_point", "data": {"result": "ok"}}
        except Exception as e:
            logger.error(f"Error in handle_set_start_position: {e}", extra={'guid': msg.get('guid')})
            response = {"title": "response_start_point", "data": {"result": "error", "message": str(e)}}
        
        await self.send_control_response(msg, response)

    async def handle_robot_info(self, msg):
        try:
            robot_info = await to_thread(self.ros_bridge.get_robot_info)
            info = robot_info.get("data", {})
            
            # Get current robot position from ROS bridge
            current_position = await to_thread(self.ros_bridge.get_current_position)
            if current_position:
                info["worldPosition"] = current_position
            else:
                # Fallback to default if position is not available
                info["worldPosition"] = {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0
                }
                logger.warning("Robot position not available, using default (0,0,0)", extra={'guid': msg.get('guid')})
            
            response = {
                "title": "response_robot_info",
                "data": {
                    **info,
                    "result": "ok"
                }
            }
        except Exception as e:
            logger.error(f"Error in handle_robot_info: {e}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_robot_info",
                "data": {
                    "worldPosition": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0
                    },
                    "result": "error",
                    "message": str(e)
                }
            }
        
        logger.info(f"Robot info response: {response}", extra={'guid': msg.get('guid')})
        await self.send_control_response(msg, response)

    async def handle_robot_manual_navigation(self, msg):
        try:
            data = msg.get("data", {})
            print(data)
            x_velocity = data.get("x", 0)  # Forward/backward velocity ratio [-1, 1]
            z_velocity = data.get("z", 0)  # Rotation angular velocity ratio [-1, 1]
            
            # Validate velocity ranges
            if not (-1 <= x_velocity <= 1):
                raise ValueError(f"x velocity {x_velocity} is out of range [-1, 1]")
            if not (-1 <= z_velocity <= 1):
                raise ValueError(f"z velocity {z_velocity} is out of range [-1, 1]")
            
            logger.info(f"Set manual navigation velocity: x={x_velocity}, z={z_velocity}", 
                       extra={'guid': msg.get('guid')})
            
            response = {
                "title": "response_robot_manual_navigation",
                "data": {"result": "ok"}
            }
        except Exception as e:
            logger.error(f"Error in handle_robot_manual_navigation: {e}", 
                        extra={'guid': msg.get('guid')})
            response = {
                "title": "response_robot_manual_navigation",
                "data": {"result": "error", "message": str(e)}
            }
        # Send the received msg out 30 times to "cmd_vel_1"
        for _ in range(30):
            await to_thread(self.ros_bridge.publish_cmd_vel_1, data)
        await self.send_control_response(msg, response)

    async def handle_robot_positioning(self, msg):
        try:
            # Get robot position and orientation from ROS bridge
            pose_info = await to_thread(self.ros_bridge.get_current_pose_dict)
            if pose_info is None:
                raise ValueError("Robot pose information is not available")
            response = {
                "title": "response_robot_positioning",
                "data": {
                    "position": pose_info["position"],
                    "orientation": pose_info["orientation"],
                    "result": "ok"
                }
            }
            logger.info("Robot positioning response sent", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_robot_positioning: {e}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_robot_positioning",
                "data": {
                    "result": "error",
                    "message": str(e)
                }
            }
        await self.send_control_response(msg, response)

    async def handle_standup_start(self, msg):
        try:
            logger.info(f"Starting standup...", extra={'guid': msg.get('guid')})
            
            # Call the ROS bridge method to execute the stand up sequence
            await to_thread(self.ros_bridge.stand_up)

            response = {
                "title": "response_standup_start", 
                "data": {
                    "result": "ok"
                }
            }
            logger.info(f"Standup completed successfully", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_standup_start: {e}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_standup_start",
                "data": {
                    "result": "error",
                    "message": str(e)
                }
            }
        
        await self.send_control_response(msg, response)
        
    async def handle_sitdown_start(self, msg):
        try:
            logger.info(f"Starting sitdown...", extra={'guid': msg.get('guid')})

            # Call the ROS bridge method to execute the sit down sequence
            await to_thread(self.ros_bridge.sit_down)
            
            response = {
                "title": "response_sitdown_start", 
                "data": {
                    "result": "ok"
                }
            }
            logger.info(f"Sitdown completed successfully", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_sitdown_start: {e}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_sitdown_start",
                "data": {
                    "result": "error",
                    "message": str(e)
                }
            }
        
        await self.send_control_response(msg, response)

    async def send_control_response(self, original_msg, response_data):
        if self.control_ws and self.control_ws.open:
            response = {
                "timestamp": int(time.time() * 1000),
                "guid": original_msg.get("guid"),
                "targetType": "client",
                "targetId": original_msg.get("clientId"),
                "clientId": self.robot_code,
                **response_data
            }

            # Add optional fields if they exist in the original message
            if "taskHistoryCode" in original_msg:
                response["taskHistoryCode"] = original_msg.get("taskHistoryCode")
            if "recordCode" in original_msg:
                response["recordCode"] = original_msg.get("recordCode")
            if "requestType" in original_msg:
                response["requestType"] = original_msg.get("requestType")

            response["data"]["robotCode"] = self.robot_code
            logger.debug(f"Sending control response: {response}")
            await self.control_ws.send(json.dumps(response))
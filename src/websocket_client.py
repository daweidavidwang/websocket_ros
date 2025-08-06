#!/usr/bin/env python3

import asyncio
import websockets
import json
import time
import io
from PIL import Image as PILImage
import numpy as np
from ros_bridge import RosBridge
from sensor_msgs.msg import PointField
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

class WebsocketClient:
    def __init__(self, ros_bridge: RosBridge, robot_code: str):
        self.ros_bridge = ros_bridge
        self.robot_code = robot_code
        self.control_ws = None
        self.video_ws = None
        self.video_stream_task = None
        self.video_stream_queue:None|queue.Queue = None

    async def connect(self):
        control_uri = "wss://gj-test.mnt-aihub.com:8443/control"
        video_uri = "wss://gj-test.mnt-aihub.com:8443/video"
        headers = {"client_type": "robot", "client_id": self.robot_code}

        await asyncio.gather(
            self._connect_websocket(control_uri, headers, self._handle_control_messages, "control"),
            self._connect_websocket(video_uri, headers, self._handle_video_messages, "video")
        )

    async def _connect_websocket(self, uri, headers, message_handler, name):
        while True:
            try:
                async with websockets.connect(uri, extra_headers=headers, ping_interval=20) as websocket:
                    if name == "control":
                        self.control_ws = websocket
                        logger.info(f"{name.capitalize()} WebSocket connected")
                        asyncio.create_task(self._send_heartbeat())
                        await message_handler(websocket)
                    elif name == "video":
                        self.video_ws = websocket
                        logger.info(f"{name.capitalize()} WebSocket connected")
                        await websocket.wait_closed()

            except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError) as e:
                logger.warning(f"{name.capitalize()} WebSocket connection closed: {e}")
            except Exception as e:
                logger.error(f"Error with {name} WebSocket: {e}")
            finally:
                if name == "control": self.control_ws = None
                if name == "video": self.video_ws = None
                logger.info(f"Reconnecting {name} WebSocket in 5 seconds...")
                await asyncio.sleep(5)

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
            response = {"title": "response_image_capture"}
            response["data"] = {
                "result": "ok",
                "height": image_msg.height,
                "width": image_msg.width,
                "encoding": image_msg.encoding,
                "is_bigendian": image_msg.is_bigendian,
                "step": image_msg.step,
                "data": image_msg.data.hex()
            }
            logger.info(f"Image captured successfully", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_image_capture: {e}", extra={'guid': msg.get('guid')})
            response["data"] = {"result": "error", "message": str(e)}
        
        await self.send_control_response(msg, response)

    async def handle_pointcloud_capture(self, msg):
        try:
            pc_msg = await to_thread(self.ros_bridge.get_point_cloud)

            def convert_fields(fields):
                """
                Converts a list of PointField objects to a dictionary with field sizes.
                """
                datatype_to_size = {
                    PointField.INT8: 1, PointField.UINT8: 1,
                    PointField.INT16: 2, PointField.UINT16: 2,
                    PointField.INT32: 4, PointField.UINT32: 4,
                    PointField.FLOAT32: 4, PointField.FLOAT64: 8,
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
                self.video_stream_task = asyncio.create_task(self._stream_video_frames())
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

            if ros_image.encoding == "rgb8":
                image_array = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
            elif ros_image.encoding == "bgr8":
                image_array = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
                image_array = image_array[..., ::-1]
            else:
                raise ValueError(f"Unsupported image encoding: {ros_image.encoding}")
            
            image = PILImage.fromarray(image_array, 'RGB')
            img_buffer = io.BytesIO()
            image.save(img_buffer, format='JPEG', quality=50)
            return (img_buffer, frame_id)
            
        except queue.Empty:
            raise Exception("Failed to get video frame within 1 second timeout")

    async def _stream_video_frames(self):
        try:
            while self.video_ws:
                img_buffer, frame_id = await to_thread(self._get_video_frame)
                await self.video_ws.send(img_buffer.getvalue())
                logger.debug(f"Sent video frame: {frame_id}, size: {len(img_buffer.getvalue())} bytes")
        except asyncio.CancelledError:
            logger.debug(f"Video streaming task cancelled")
        except websockets.exceptions.ConnectionClosed:
            logger.warning("Video WebSocket connection closed, stopping video stream")
        except websockets.exceptions.ConnectionClosedOK:
            logger.warning("Video WebSocket connection closed normally, stopping video stream")
        except websockets.exceptions.ConnectionClosedError:
            logger.warning("Video WebSocket connection closed with error, stopping video stream")
        except Exception as e:
            logger.error(f"Error in video streaming task: {e}")
        finally:
            asyncio.create_task(self._cleanup_video_stream_task())

    async def _cleanup_video_stream_task(self):
            logger.debug("Cleanning up video stream task...")
            self.video_stream_task = None
            await to_thread(self.ros_bridge.stop_video_stream)
            self.video_stream_queue = None
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
            info["worldPosition"] = {
                "x": 0,
                "y": 0,
                "z": 0
            }
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
                    "result": "error",
                    "message": str(e)
                }
            }
        print(f"Robot info response: {response}")
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
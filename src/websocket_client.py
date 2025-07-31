#!/usr/bin/env python3

import asyncio
import websockets
import json
import time
import base64
import io
from PIL import Image as PILImage
import numpy as np
from ros_bridge import RosBridge
from sensor_msgs.msg import PointField
import contextvars
import functools
import logging

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

ROBOT_CODE = "robot3234324232"

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
    def __init__(self, ros_bridge: RosBridge):
        self.ros_bridge = ros_bridge
        self.control_ws = None
        self.video_ws = None
        self.video_stream_task = None
        self.video_stream_queue = None

    async def connect(self):
        control_uri = "wss://gj-test.mnt-aihub.com:8443/control"
        video_uri = "wss://gj-test.mnt-aihub.com:8443/video"
        headers = {"client_type": "robot", "client_id": ROBOT_CODE}

        await asyncio.gather(
            self._connect_websocket(control_uri, headers, self._handle_control_messages, "control"),
            self._connect_websocket(video_uri, headers, None, "video")
        )

    async def _connect_websocket(self, uri, headers, message_handler, name):
        while True:
            try:
                async with websockets.connect(uri, extra_headers=headers) as websocket:
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
                    "guid": "uuid-3456-req-vp",
                    "targetType": "robot",
                    "targetId": "heartbeat",
                    "clientId": "heartbeat",
                    "data": {"robotCode": ROBOT_CODE, "result": "ok"}
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
                await self._process_command(msg_json)
            except json.JSONDecodeError:
                logger.warning(f"Received non-JSON message: {message}")
            except Exception as e:
                logger.error(f"Error processing message: {e}")

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
            response = {
                "title": "response_image_capture",
                "data": {
                    "result": "ok",
                    "height": image_msg.height,
                    "width": image_msg.width,
                    "encoding": image_msg.encoding,
                    "is_bigendian": image_msg.is_bigendian,
                    "step": image_msg.step,
                    "data": image_msg.data.hex()
                }
            }
            logger.info(f"Image captured successfully", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_image_capture: {e}", extra={'guid': msg.get('guid')})
            response = {"title": "response_image_capture", "data": {"result": "error", "message": str(e)}}
        
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
            response = {
                "title": "response_pointcloud_capture",
                "data": {
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
            }
            logger.info("Point cloud captured successfully", extra={'guid': msg.get('guid')})
        except Exception as e:
            logger.error(f"Error in handle_pointcloud_capture: {e}", extra={'guid': msg.get('guid')})
            response = {"title": "response_pointcloud_capture", "data": {"result": "error", "message": str(e)}}

        await self.send_control_response(msg, response)

    async def handle_video_start(self, msg):
        try:
            if self.video_stream_task:
                response = {"title": "response_video_start", "data": {"result": "error", "message": "Video stream already running"}}
                logger.warning("Video stream already running, cannot start again", extra={'guid': msg.get('guid')})
            else:
                self.video_stream_queue = asyncio.Queue(maxsize=1)

                loop = asyncio.get_event_loop()
                def _ros_video_callback(ros_image):
                    asyncio.run_coroutine_threadsafe(self._put_frame_to_queue(ros_image), loop)

                await to_thread(self.ros_bridge.setup_video_stream, _ros_video_callback)
                self.video_stream_task = asyncio.create_task(self._stream_video_frames())
                logger.info("Video stream started", extra={'guid': msg.get('guid')})
                response = {"title": "response_video_start", "data": {"result": "ok"}}
        except Exception as e:
            logger.error(f"Error in handle_video_start: {e}", extra={'guid': msg.get('guid')})
            response = {"title": "response_video_start", "data": {"result": "error", "message": str(e)}}
        
        await self.send_control_response(msg, response)

    async def _put_frame_to_queue(self, ros_image):
        if self.video_stream_queue.full():
            self.video_stream_queue.get_nowait()
        self.video_stream_queue.put_nowait(ros_image)

    async def _stream_video_frames(self):
        try:
            while self.video_ws and self.video_ws.open:
                ros_image, frame_id = await self.video_stream_queue.get()
                
                if ros_image.encoding == "rgb8":
                    image_array = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
                elif ros_image.encoding == "bgr8":
                    image_array = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
                    image_array = image_array[..., ::-1]
                else:
                    continue

                image = PILImage.fromarray(image_array, 'RGB')
                img_buffer = io.BytesIO()
                image.save(img_buffer, format='JPEG', quality=95)
                
                await self.video_ws.send(img_buffer.getvalue())
                logger.debug(f"Sent video frame: {frame_id}")
        except asyncio.CancelledError:
            logger.debug(f"Video streaming task cancelled")
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
            if self.video_stream_task:
                self.video_stream_task.cancel()
                logger.info("Video stream stopped", extra={'guid': msg.get('guid')})
                response = {"title": "response_video_stop", "data": {"result": "ok"}}
            else:
                logger.warning("Video stream not running, cannot stop", extra={'guid': msg.get('guid')})
                response = {"title": "response_video_stop", "data": {"result": "error", "message": "Video stream not running"}}
        except Exception as e:
            logger.error(f"Error in handle_video_stop: {e}", extra={'guid': msg.get('guid')})
            response = {"title": "response_video_stop", "data": {"result": "error", "message": str(e)}}
        
        await self.send_control_response(msg, response)

    async def handle_start_navigation(self, msg):
        try:
            goal_data = msg.get("data", {})
            request_type = msg.get("requestType", "")

            loop = asyncio.get_event_loop()
            def _on_goal_reached():
                response = {
                    "title": "response_arrive_navigation",
                    "requestType": request_type,
                    "data": {"result": "ok"}
                }
                asyncio.run_coroutine_threadsafe(self.send_control_response(msg, response), loop)

            await to_thread(self.ros_bridge.start_navigation, goal_data, _on_goal_reached)
            logger.info(f"Started navigation with goal: {goal_data}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_start_navigation", 
                "requestType": request_type, 
                "data": {"result": "ok"}
            }
        except Exception as e:
            logger.error(f"Error in handle_start_navigation: {e}", extra={'guid': msg.get('guid')})
            response = {
                "title": "response_start_navigation", 
                "requestType": request_type, 
                "data": {"result": "error", "message": str(e)}
            }
        
        await self.send_control_response(msg, response)

    async def handle_stop_navigation(self, msg):
        try:
            request_type = msg.get("requestType", "")
            await to_thread(self.ros_bridge.stop_navigation)
            logger.info(f"Stopped navigation", extra={'guid': msg.get('guid')})
            response = {"title": "response_stop_navigation", "requestType": request_type, "data": {"result": "ok"}}
        except Exception as e:
            logger.error(f"Error in handle_stop_navigation: {e}", extra={'guid': msg.get('guid')})
            response = {"title": "response_stop_navigation", "requestType": request_type, "data": {"result": "error", "message": str(e)}}
        
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
        # Mock data as per original implementation
        response = {
            "title": "response_robot_info",
            "data": {
                "robotCode": ROBOT_CODE,
                "accid": "PF_TRON1A_042",
                "sw_version": "robot-tron1-2.0.10.20241111103012",
                "imu": "OK", "camera": "OK", "motor": "OK",
                "battery": 95, "status": "WALK",
                "gridPosition": {"x": 100, "y": 200, "z": 0},
                "result": "ok"
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
                "clientId": ROBOT_CODE,
                "taskHistoryCode": original_msg.get("taskHistoryCode"),
                **response_data
            }
            logger.debug(f"Sending control response: {response}")
            await self.control_ws.send(json.dumps(response))
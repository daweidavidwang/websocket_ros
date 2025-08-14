#!/usr/bin/env python3

import asyncio
import websockets
import json
import time
import ssl
import logging
import argparse
from pathlib import Path

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
        log_fmt = super().format(record)
        color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
        reset = self.COLORS['RESET']
        return f"{color}{log_fmt}{reset}"

logger = logging.getLogger('mock_video_server')
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
handler.setFormatter(CustomFormatter())
handler.setFormatter(logging.Formatter('[%(asctime)s][%(levelname)s] - %(message)s'))
logger.addHandler(handler)

def set_log_level(level_str: str):
    """Sets the logging level for the mock_video_server logger."""
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
    logger.info(f"Mock video server log level set to {logging.getLevelName(level)}")

class MockVideoServer:
    def __init__(self, host="localhost", control_port=8443, video_port=8444, auto_start_video=True):
        self.host = host
        self.control_port = control_port
        self.video_port = video_port
        self.auto_start_video = auto_start_video
        self.connected_clients = {}
        self.video_frame_count = {}
        self.start_time = time.time()

    async def handle_control_connection(self, websocket, path):
        """Handle control WebSocket connections"""
        client_id = None
        try:
            # Get client info from headers
            client_type = websocket.request_headers.get("client_type", "unknown")
            client_id = websocket.request_headers.get("client_id", "unknown")
            
            logger.info(f"Control connection established - Client: {client_id}, Type: {client_type}")
            
            # Store connection info
            self.connected_clients[client_id] = {
                "control_ws": websocket,
                "client_type": client_type,
                "connected_at": time.time()
            }
            
            # Send request_video_start command after connection is established (if enabled)
            if self.auto_start_video:
                await self.send_video_start_request(websocket, client_id)
                
                # Start additional test commands task
                test_commands_task = asyncio.create_task(
                    self.send_additional_test_commands(websocket, client_id)
                )
            
            # Listen for incoming messages
            async for message in websocket:
                try:
                    msg_json = json.loads(message)
                    title = msg_json.get("title", "")
                    
                    logger.debug(f"Received control message from {client_id}: {title}")
                    
                    # Handle heartbeat
                    if title == "response_heartbeat":
                        logger.debug(f"Heartbeat received from {client_id}")
                    
                    # Handle other control messages
                    elif title.startswith("request_"):
                        await self.handle_control_request(websocket, msg_json, client_id)
                    
                except json.JSONDecodeError:
                    logger.warning(f"Received non-JSON control message from {client_id}: {message}")
                except Exception as e:
                    logger.error(f"Error processing control message from {client_id}: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Control connection closed for client: {client_id}")
        except Exception as e:
            logger.error(f"Error in control connection for client {client_id}: {e}")
        finally:
            # Cancel test commands task if it exists
            if 'test_commands_task' in locals() and not test_commands_task.done():
                test_commands_task.cancel()
                logger.debug(f"Cancelled test commands task for client: {client_id}")
            
            if client_id and client_id in self.connected_clients:
                del self.connected_clients[client_id]
                logger.info(f"Cleaned up connection info for client: {client_id}")

    async def handle_control_request(self, websocket, msg_json, client_id):
        """Handle control requests and send mock responses"""
        title = msg_json.get("title", "")
        guid = msg_json.get("guid", "")
        
        # Create mock response based on request type
        response = {
            "timestamp": int(time.time() * 1000),
            "guid": guid,
            "targetType": "client",
            "targetId": msg_json.get("clientId", ""),
            "clientId": "mock_server",
        }
        
        if title == "request_video_start":
            response.update({
                "title": "response_video_start",
                "data": {"result": "ok", "message": "Video stream ready to receive"}
            })
        elif title == "request_video_stop":
            response.update({
                "title": "response_video_stop", 
                "data": {"result": "ok", "message": "Video stream stopped"}
            })
        elif title == "request_image_capture":
            response.update({
                "title": "response_image_capture",
                "data": {"result": "ok", "message": "Image capture acknowledged"}
            })
        else:
            response.update({
                "title": f"response_{title.replace('request_', '')}",
                "data": {"result": "ok", "message": f"Mock response for {title}"}
            })
        
        await websocket.send(json.dumps(response))
        logger.info(f"Sent mock response for {title} to {client_id}")

    async def send_video_start_request(self, websocket, client_id):
        """Send request_video_start command to client to initiate video streaming"""
        try:
            # Wait a short time to ensure connection is stable
            await asyncio.sleep(1)
            
            request_data = {
                "title": "request_video_start",
                "timestamp": int(time.time() * 1000),
                "guid": f"mock_video_start_{int(time.time())}",
                "targetType": "robot",
                "targetId": client_id,
                "clientId": "mock_server",
                "data": {
                    "quality": "medium",
                    "fps": 30,
                    "format": "jpeg"
                }
            }
            
            await websocket.send(json.dumps(request_data))
            logger.info(f"Sent request_video_start to client: {client_id}")
            
        except Exception as e:
            logger.error(f"Error sending video start request to {client_id}: {e}")

    async def send_additional_test_commands(self, websocket, client_id):
        """Send additional test commands after video streaming is established"""
        try:
            # Wait for video stream to be established
            await asyncio.sleep(5)
            
            # Send robot info request
            robot_info_request = {
                "title": "request_robot_info",
                "timestamp": int(time.time() * 1000),
                "guid": f"mock_robot_info_{int(time.time())}",
                "targetType": "robot",
                "targetId": client_id,
                "clientId": "mock_server",
                "data": {}
            }
            
            await websocket.send(json.dumps(robot_info_request))
            logger.info(f"Sent request_robot_info to client: {client_id}")
            
            # Wait a bit more, then send image capture request
            await asyncio.sleep(3)
            
            image_capture_request = {
                "title": "request_image_capture",
                "timestamp": int(time.time() * 1000),
                "guid": f"mock_image_capture_{int(time.time())}",
                "targetType": "robot",
                "targetId": client_id,
                "clientId": "mock_server",
                "data": {}
            }
            
            await websocket.send(json.dumps(image_capture_request))
            logger.info(f"Sent request_image_capture to client: {client_id}")
            
            # Optional: Send video stop after 30 seconds for testing
            await asyncio.sleep(30)
            
            video_stop_request = {
                "title": "request_video_stop",
                "timestamp": int(time.time() * 1000),
                "guid": f"mock_video_stop_{int(time.time())}",
                "targetType": "robot",
                "targetId": client_id,
                "clientId": "mock_server",
                "data": {}
            }
            
            await websocket.send(json.dumps(video_stop_request))
            logger.info(f"Sent request_video_stop to client: {client_id}")
            
        except Exception as e:
            logger.error(f"Error sending additional test commands to {client_id}: {e}")

    async def handle_video_connection(self, websocket, path):
        """Handle video WebSocket connections and receive frames"""
        client_id = None
        frame_count = 0
        total_bytes = 0
        
        try:
            # Get client info from headers
            client_type = websocket.request_headers.get("client_type", "unknown")
            client_id = websocket.request_headers.get("client_id", "unknown")
            
            logger.info(f"Video connection established - Client: {client_id}, Type: {client_type}")
            
            # Initialize frame counter for this client
            self.video_frame_count[client_id] = 0
            
            # Update connection info if client already exists
            if client_id in self.connected_clients:
                self.connected_clients[client_id]["video_ws"] = websocket
            else:
                self.connected_clients[client_id] = {
                    "video_ws": websocket,
                    "client_type": client_type,
                    "connected_at": time.time()
                }
            
            # Listen for video frames
            async for message in websocket:
                try:
                    # Check if message is JSON (control message) or binary (video frame)
                    if isinstance(message, str):
                        # Handle JSON control messages on video channel
                        try:
                            msg_json = json.loads(message)
                            logger.debug(f"Received video control message from {client_id}: {msg_json}")
                        except json.JSONDecodeError:
                            logger.warning(f"Received non-JSON string message on video channel from {client_id}")
                    else:
                        # Handle binary video frame data
                        frame_size = len(message)
                        frame_count += 1
                        total_bytes += frame_size
                        self.video_frame_count[client_id] = frame_count
                        
                        # Log frame info periodically
                        if frame_count % 30 == 0:  # Log every 30 frames (~1 second at 30fps)
                            avg_frame_size = total_bytes / frame_count
                            elapsed_time = time.time() - self.start_time
                            fps = frame_count / elapsed_time if elapsed_time > 0 else 0
                            
                            logger.info(f"Video stats for {client_id}: "
                                      f"Frames: {frame_count}, "
                                      f"Avg size: {avg_frame_size:.1f} bytes, "
                                      f"Total: {total_bytes/1024/1024:.2f} MB, "
                                      f"FPS: {fps:.1f}")
                        else:
                            logger.debug(f"Received video frame from {client_id}: "
                                       f"Frame #{frame_count}, Size: {frame_size} bytes")
                        
                        # Optionally save frame to file (for debugging)
                        # if frame_count <= 5:  # Save first 5 frames
                        #     frame_path = f"debug_frame_{client_id}_{frame_count}.jpg"
                        #     with open(frame_path, "wb") as f:
                        #         f.write(message)
                        #     logger.debug(f"Saved frame to {frame_path}")
                        
                except Exception as e:
                    logger.error(f"Error processing video message from {client_id}: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Video connection closed for client: {client_id} "
                       f"(received {frame_count} frames, {total_bytes/1024/1024:.2f} MB total)")
        except Exception as e:
            logger.error(f"Error in video connection for client {client_id}: {e}")
        finally:
            if client_id:
                # Clean up video connection info
                if client_id in self.connected_clients and "video_ws" in self.connected_clients[client_id]:
                    del self.connected_clients[client_id]["video_ws"]
                if client_id in self.video_frame_count:
                    final_count = self.video_frame_count[client_id]
                    del self.video_frame_count[client_id]
                    logger.info(f"Video connection cleanup for {client_id}: {final_count} frames received")

    def create_ssl_context(self, cert_path=None, key_path=None):
        """Create SSL context for secure WebSocket connections"""
        if cert_path and key_path and Path(cert_path).exists() and Path(key_path).exists():
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(cert_path, key_path)
            logger.info(f"Using SSL certificate: {cert_path}")
            return ssl_context
        else:
            # Create self-signed certificate for testing
            logger.warning("No SSL certificate provided, using self-signed certificate")
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.check_hostname = False
            ssl_context.verify_mode = ssl.CERT_NONE
            return ssl_context

    async def start_server(self, use_ssl=False, cert_path=None, key_path=None):
        """Start the mock video server"""
        ssl_context = None
        if use_ssl:
            ssl_context = self.create_ssl_context(cert_path, key_path)
        
        protocol = "wss" if use_ssl else "ws"
        
        # Start control server
        control_server = await websockets.serve(
            self.handle_control_connection,
            self.host,
            self.control_port,
            ssl=ssl_context,
            ping_interval=20,
            ping_timeout=10,
            max_size=(2**20) * 10  # 10 MB max message size
        )
        
        # Start video server  
        video_server = await websockets.serve(
            self.handle_video_connection,
            self.host, 
            self.video_port,
            ssl=ssl_context,
            ping_interval=20,
            ping_timeout=10,
            max_size=(2**20) * 2  # 2 MB max message size
        )
        
        logger.info(f"Mock video server started:")
        logger.info(f"  Control endpoint: {protocol}://{self.host}:{self.control_port}/control")
        logger.info(f"  Video endpoint: {protocol}://{self.host}:{self.video_port}/video")
        logger.info("Waiting for connections...")
        
        # Print connection status periodically
        async def print_status():
            while True:
                await asyncio.sleep(30)  # Print every 30 seconds
                if self.connected_clients:
                    logger.info(f"Connected clients: {len(self.connected_clients)}")
                    for client_id, info in self.connected_clients.items():
                        frame_count = self.video_frame_count.get(client_id, 0)
                        uptime = time.time() - info["connected_at"]
                        has_control = "control_ws" in info
                        has_video = "video_ws" in info
                        logger.info(f"  {client_id}: Control={has_control}, Video={has_video}, "
                                   f"Frames={frame_count}, Uptime={uptime:.1f}s")
                else:
                    logger.info("No connected clients")
        
        # Start status reporting task
        status_task = asyncio.create_task(print_status())
        
        try:
            # Keep servers running
            await asyncio.gather(
                control_server.wait_closed(),
                video_server.wait_closed()
            )
        finally:
            status_task.cancel()

def main():
    parser = argparse.ArgumentParser(description="Mock Video Server for WebSocket testing")
    parser.add_argument("--host", default="localhost", help="Server host (default: localhost)")
    parser.add_argument("--control-port", type=int, default=8443, help="Control port (default: 8443)")
    parser.add_argument("--video-port", type=int, default=8444, help="Video port (default: 8444)")
    parser.add_argument("--no-ssl", action="store_true", help="Disable SSL (use ws:// instead of wss://)")
    parser.add_argument("--cert", help="Path to SSL certificate file")
    parser.add_argument("--key", help="Path to SSL private key file")
    parser.add_argument("--log-level", default="info", 
                       choices=["debug", "info", "warning", "error", "critical"],
                       help="Set logging level (default: info)")
    parser.add_argument("--auto-start-video", action="store_true", default=True,
                       help="Automatically send request_video_start to clients (default: enabled)")
    parser.add_argument("--no-auto-start-video", action="store_true",
                       help="Disable automatic video start requests")
    
    args = parser.parse_args()
    
    # Set log level
    set_log_level(args.log_level)
    
    # Determine auto start video setting
    auto_start_video = args.auto_start_video and not args.no_auto_start_video
    
    # Create and start server
    server = MockVideoServer(args.host, args.control_port, args.video_port, auto_start_video)
    
    try:
        asyncio.run(server.start_server(
            use_ssl=not args.no_ssl,
            cert_path=args.cert,
            key_path=args.key
        ))
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
    except Exception as e:
        logger.error(f"Server error: {e}")

if __name__ == "__main__":
    main()

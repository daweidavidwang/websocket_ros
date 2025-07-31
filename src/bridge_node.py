#!/usr/bin/env python3

import rospy
import asyncio
from ros_bridge import RosBridge
from websocket_client import WebsocketClient, set_log_level as set_ws_log_level
import threading
import argparse

def main():
    parser = argparse.ArgumentParser(description='ROS WebSocket Bridge')
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

    # Set the log level for the websocket client logger
    set_ws_log_level(args.log_level)

    rospy.init_node('ros_websocket_bridge', anonymous=True, log_level=rospy_log_level)
    
    ros_bridge = RosBridge()
    ws_client = WebsocketClient(ros_bridge)
    
    def run_websocket_client():
        """Run the asyncio event loop"""
        try:
            asyncio.run(ws_client.connect())
        except Exception as e:
            rospy.logerr(f"WebSocket manager error: {e}")
        finally:
            asyncio.get_event_loop().close()

    try:
        thread = threading.Thread(target=run_websocket_client, daemon=True)
        thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo("Shutting down ROS WebSocket bridge.")

if __name__ == '__main__':
    main()

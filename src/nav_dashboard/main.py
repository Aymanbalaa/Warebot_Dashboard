"""
Main entry point: spins up both the ROS2 node and the FastAPI web server.
"""

import asyncio
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
import uvicorn

from nav_dashboard.ros_bridge import RosBridge
from nav_dashboard.web_server import create_app


def main():
    rclpy.init()
    bridge = RosBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)

    # Spin ROS2 in a background thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    bridge.get_logger().info('Starting web server on http://0.0.0.0:8420')

    app = create_app(bridge)

    try:
        uvicorn.run(app, host='0.0.0.0', port=8420, log_level='warning')
    except KeyboardInterrupt:
        pass
    finally:
        bridge.get_logger().info('Shutting down...')
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

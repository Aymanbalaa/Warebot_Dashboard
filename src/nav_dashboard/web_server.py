"""
FastAPI + WebSocket server that bridges ROS2 data to the web UI.
"""

import asyncio
import json
import os

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles

from nav_dashboard.ros_bridge import RosBridge


def create_app(bridge: RosBridge) -> FastAPI:
    app = FastAPI(title="Nav Dashboard")

    # Serve static web files
    web_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'web')
    if not os.path.isdir(web_dir):
        # Installed location
        import ament_index_python
        try:
            pkg_share = ament_index_python.get_package_share_directory('nav_dashboard')
            web_dir = os.path.join(pkg_share, 'web')
        except Exception:
            web_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'web')

    @app.get("/")
    async def index():
        return FileResponse(os.path.join(web_dir, 'index.html'))

    @app.get("/app.js")
    async def app_js():
        return FileResponse(os.path.join(web_dir, 'app.js'), media_type='application/javascript')

    @app.get("/style.css")
    async def app_css():
        return FileResponse(os.path.join(web_dir, 'style.css'), media_type='text/css')

    @app.get("/logo.png")
    async def logo():
        return FileResponse(os.path.join(web_dir, 'logo-white.png'), media_type='image/png')

    # ─── Mobile endpoint ─────────────────────────────────
    @app.get("/mobile")
    async def mobile_index():
        return FileResponse(os.path.join(web_dir, 'mobile.html'))

    @app.get("/mobile.js")
    async def mobile_js():
        return FileResponse(os.path.join(web_dir, 'mobile.js'), media_type='application/javascript')

    @app.get("/mobile.css")
    async def mobile_css():
        return FileResponse(os.path.join(web_dir, 'mobile.css'), media_type='text/css')

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        await ws.accept()
        bridge.get_logger().info('WebSocket client connected')

        # Send full initial state
        state = bridge.get_full_state()
        try:
            if state['map']:
                await ws.send_text(json.dumps({'type': 'map', **state['map']}))
            if state['pose']:
                await ws.send_text(json.dumps({'type': 'pose', **state['pose']}))
            if state['scan']:
                await ws.send_text(json.dumps({'type': 'scan', **state['scan']}))
            if state['global_plan']:
                await ws.send_text(json.dumps({'type': 'global_plan', 'points': state['global_plan']}))
        except Exception:
            return

        # Register queue for push updates
        q = bridge.register_ws_queue()

        # Two tasks: read from queue → send, read from ws → handle commands
        async def sender():
            try:
                while True:
                    msg = await q.get()
                    await ws.send_text(msg)
            except Exception:
                pass

        async def receiver():
            try:
                while True:
                    raw = await ws.receive_text()
                    data = json.loads(raw)
                    cmd = data.get('cmd')

                    if cmd == 'initial_pose':
                        bridge.publish_initial_pose(
                            float(data['x']), float(data['y']), float(data['yaw'])
                        )
                    elif cmd == 'nav_goal':
                        bridge.publish_nav_goal(
                            float(data['x']), float(data['y']), float(data['yaw'])
                        )
                    elif cmd == 'cancel_nav':
                        bridge.cancel_nav()
                    elif cmd == 'get_state':
                        s = bridge.get_full_state()
                        await ws.send_text(json.dumps({'type': 'full_state', **s}))

            except WebSocketDisconnect:
                pass
            except Exception as e:
                bridge.get_logger().warn(f'WS receiver error: {e}')

        sender_task = asyncio.create_task(sender())
        receiver_task = asyncio.create_task(receiver())

        try:
            done, pending = await asyncio.wait(
                [sender_task, receiver_task],
                return_when=asyncio.FIRST_COMPLETED,
            )
            for t in pending:
                t.cancel()
        finally:
            bridge.unregister_ws_queue(q)
            bridge.get_logger().info('WebSocket client disconnected')

    # Status check for heartbeat
    @app.get("/api/status")
    async def status():
        state = bridge.get_full_state()
        return {
            'nav_status': state['nav_status'],
            'pose': state['pose'],
            'odom': state['odom'],
            'battery': state['battery'],
            'cmd_vel': state['cmd_vel'],
            'has_map': state['map'] is not None,
        }

    return app

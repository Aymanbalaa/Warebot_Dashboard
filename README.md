# Warebot Dashboard (nav_dashboard)

This repository contains a **ROS 2 web dashboard** for robot navigation.
It runs a ROS 2 bridge node plus a FastAPI server, then serves a browser UI that shows live robot/navigation data and lets operators send navigation commands.

## What it does

- Streams robot state to the browser over WebSocket:
  - map and costmap
  - robot pose (TF)
  - laser scan points
  - odometry and velocity
  - global/local plans
  - Nav2 action status
  - battery (if available)
- Accepts control commands from the UI:
  - set initial pose
  - send navigation goal
  - cancel navigation
- Provides:
  - desktop UI at `/`
  - mobile UI at `/mobile`
  - health/status endpoint at `/api/status`

## Tech stack

- ROS 2 Python node (`rclpy`)
- FastAPI + Uvicorn web server
- WebSocket bridge for real-time updates
- Static frontend files in `/web`

## Run options

### Quick run (without colcon build)

```bash
./run.sh
```

Then open: `http://localhost:8420`

### ROS 2 package run

```bash
ros2 run nav_dashboard dashboard
```

### Launch file

```bash
ros2 launch nav_dashboard dashboard.launch.py
```

## Repository layout

- `src/nav_dashboard/ros_bridge.py` – ROS 2 subscriptions/publishers and state bridge
- `src/nav_dashboard/web_server.py` – FastAPI routes + WebSocket endpoint
- `src/nav_dashboard/main.py` – starts ROS executor and web server together
- `web/` – frontend assets (desktop + mobile)
- `launch/dashboard.launch.py` – ROS 2 launch entry

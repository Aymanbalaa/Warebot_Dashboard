/* ═══════════════════════════════════════════════════════════
   NAV DASHBOARD — Main Application
   Canvas-based map renderer with lidar, robot pose, paths,
   interactive initial pose & nav goal setting.
   ═══════════════════════════════════════════════════════════ */

(() => {
    "use strict";

    // ─── State ────────────────────────────────────────────
    const state = {
        ws: null,
        connected: false,

        // Map
        map: null,           // { width, height, resolution, origin_x, origin_y, origin_yaw, imageData }
        mapImage: null,      // OffscreenCanvas with rendered map
        costmapImage: null,

        // Robot
        pose: null,          // { x, y, yaw }
        scan: null,          // { points }
        odom: null,
        cmdVel: null,
        battery: null,
        navStatus: 'idle',

        // Paths
        globalPlan: null,
        localPlan: null,

        // View transform
        camera: { x: 0, y: 0, zoom: 40 },  // pixels per meter

        // Interaction
        tool: 'initial_pose',  // 'pan', 'initial_pose', 'nav_goal'
        dragging: false,
        dragStart: null,
        cameraStart: null,

        // Pose/goal placement
        placing: false,
        placeStart: null,    // map coords {x, y}
        placeAngle: 0,

        // Layers
        layers: {
            map: true,
            costmap: false,
            scan: true,
            robot: true,
            path: true,
            localPath: true,
        },

        followRobot: false,
    };

    // ─── DOM References ───────────────────────────────────
    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');
    const coordText = document.getElementById('coordText');
    const zoomText = document.getElementById('zoomText');
    const connStatus = document.getElementById('connStatus');
    const arrowHint = document.getElementById('arrowHint');

    // Stats
    const els = {
        statX: document.getElementById('statX'),
        statY: document.getElementById('statY'),
        statYaw: document.getElementById('statYaw'),
        statLinVel: document.getElementById('statLinVel'),
        statAngVel: document.getElementById('statAngVel'),
        statBattery: document.getElementById('statBattery'),
        statMapSize: document.getElementById('statMapSize'),
        statMapRes: document.getElementById('statMapRes'),
        navStatusText: document.getElementById('navStatusText'),
        navStatusBadge: document.getElementById('navStatusBadge'),
    };

    // ─── Resize ───────────────────────────────────────────
    function resize() {
        const rect = canvas.parentElement.getBoundingClientRect();
        canvas.width = rect.width * devicePixelRatio;
        canvas.height = rect.height * devicePixelRatio;
        canvas.style.width = rect.width + 'px';
        canvas.style.height = rect.height + 'px';
    }
    window.addEventListener('resize', resize);
    resize();

    // ─── Coordinate Transforms ────────────────────────────
    // Map coords (meters) → screen pixels
    function mapToScreen(mx, my) {
        const sx = (mx - state.camera.x) * state.camera.zoom + canvas.width / 2;
        const sy = -(my - state.camera.y) * state.camera.zoom + canvas.height / 2;
        return [sx, sy];
    }

    // Screen pixels → map coords (meters)
    function screenToMap(sx, sy) {
        const mx = (sx - canvas.width / 2) / state.camera.zoom + state.camera.x;
        const my = -(sy - canvas.height / 2) / state.camera.zoom + state.camera.y;
        return [mx, my];
    }

    // ─── Map Image Rendering ──────────────────────────────
    function renderMapImage(mapData, dataB64) {
        const raw = atob(dataB64);
        const w = mapData.width;
        const h = mapData.height;

        const offscreen = new OffscreenCanvas(w, h);
        const octx = offscreen.getContext('2d');
        const imgData = octx.createImageData(w, h);
        const pixels = imgData.data;

        for (let i = 0; i < raw.length; i++) {
            let val = raw.charCodeAt(i);
            let r, g, b, a;

            if (val === 255) {
                // Unknown
                r = 30; g = 35; b = 50; a = 180;
            } else if (val === 0) {
                // Free space
                r = 25; g = 32; b = 48; a = 255;
            } else if (val >= 1 && val <= 50) {
                // Low occupancy - subtle gradient
                const t = val / 50;
                r = Math.floor(25 + t * 40);
                g = Math.floor(32 + t * 20);
                b = Math.floor(48 + t * 30);
                a = 255;
            } else {
                // Occupied (walls) — bright cyan/blue
                const t = Math.min(1, (val - 50) / 50);
                r = Math.floor(40 + t * 80);
                g = Math.floor(80 + t * 140);
                b = Math.floor(160 + t * 95);
                a = 255;
            }

            // Flip Y: ROS map row 0 = bottom
            const row = h - 1 - Math.floor(i / w);
            const col = i % w;
            const pi = (row * w + col) * 4;

            pixels[pi] = r;
            pixels[pi + 1] = g;
            pixels[pi + 2] = b;
            pixels[pi + 3] = a;
        }

        octx.putImageData(imgData, 0, 0);
        return offscreen;
    }

    function renderCostmapImage(mapData, dataB64) {
        const raw = atob(dataB64);
        const w = mapData.width;
        const h = mapData.height;

        const offscreen = new OffscreenCanvas(w, h);
        const octx = offscreen.getContext('2d');
        const imgData = octx.createImageData(w, h);
        const pixels = imgData.data;

        for (let i = 0; i < raw.length; i++) {
            let val = raw.charCodeAt(i);
            const row = h - 1 - Math.floor(i / w);
            const col = i % w;
            const pi = (row * w + col) * 4;

            if (val === 0 || val === 255) {
                pixels[pi + 3] = 0; // transparent
            } else {
                // Purple-red gradient for costmap
                const t = val / 100;
                pixels[pi] = Math.floor(100 + t * 155);
                pixels[pi + 1] = Math.floor(20 * (1 - t));
                pixels[pi + 2] = Math.floor(180 * (1 - t));
                pixels[pi + 3] = Math.floor(60 + t * 120);
            }
        }

        octx.putImageData(imgData, 0, 0);
        return offscreen;
    }

    // ─── Drawing ──────────────────────────────────────────
    function draw() {
        const dpr = devicePixelRatio;
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Background
        ctx.fillStyle = '#080c14';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Draw grid
        drawGrid();

        // Draw map
        if (state.layers.map && state.mapImage && state.map) {
            drawMapLayer(state.mapImage, state.map);
        }

        // Draw costmap
        if (state.layers.costmap && state.costmapImage && state.costmapMeta) {
            drawMapLayer(state.costmapImage, state.costmapMeta);
        }

        // Global plan
        if (state.layers.path && state.globalPlan) {
            drawPath(state.globalPlan, '#3b82f6', 2.5, [8, 4]);
        }

        // Local plan
        if (state.layers.localPath && state.localPlan) {
            drawPath(state.localPlan, '#8b5cf6', 2, []);
        }

        // Lidar scan
        if (state.layers.scan && state.scan && state.pose) {
            drawScan();
        }

        // Robot
        if (state.layers.robot && state.pose) {
            drawRobot();
        }

        // Placement arrow
        if (state.placing && state.placeStart) {
            drawPlacementArrow();
        }

        requestAnimationFrame(draw);
    }

    function drawGrid() {
        const zoom = state.camera.zoom;
        // Determine grid spacing in meters
        let gridSize = 1;
        if (zoom < 15) gridSize = 5;
        if (zoom < 5) gridSize = 10;
        if (zoom > 80) gridSize = 0.5;
        if (zoom > 200) gridSize = 0.1;

        const [left, top] = screenToMap(0, 0);
        const [right, bottom] = screenToMap(canvas.width, canvas.height);

        ctx.strokeStyle = '#111827';
        ctx.lineWidth = 1;

        const startX = Math.floor(left / gridSize) * gridSize;
        const startY = Math.floor(bottom / gridSize) * gridSize;

        ctx.beginPath();
        for (let x = startX; x <= right; x += gridSize) {
            const [sx] = mapToScreen(x, 0);
            ctx.moveTo(sx, 0);
            ctx.lineTo(sx, canvas.height);
        }
        for (let y = startY; y <= top; y += gridSize) {
            const [, sy] = mapToScreen(0, y);
            ctx.moveTo(0, sy);
            ctx.lineTo(canvas.width, sy);
        }
        ctx.stroke();

        // Origin axes
        const [ox, oy] = mapToScreen(0, 0);
        ctx.lineWidth = 1.5;
        // X axis (red)
        ctx.strokeStyle = '#ef444460';
        ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + 60 * zoom / 40, oy); ctx.stroke();
        // Y axis (green)
        ctx.strokeStyle = '#10b98160';
        ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - 60 * zoom / 40); ctx.stroke();
    }

    function drawMapLayer(image, meta) {
        const zoom = state.camera.zoom;
        const res = meta.resolution;
        const w = meta.width * res;
        const h = meta.height * res;

        const [sx, sy] = mapToScreen(meta.origin_x, meta.origin_y + h);

        ctx.imageSmoothingEnabled = zoom < 60;
        ctx.drawImage(image, sx, sy, w * zoom, h * zoom);
    }

    function drawPath(points, color, width, dash) {
        if (points.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.setLineDash(dash);
        ctx.lineJoin = 'round';
        ctx.beginPath();

        const [sx0, sy0] = mapToScreen(points[0][0], points[0][1]);
        ctx.moveTo(sx0, sy0);
        for (let i = 1; i < points.length; i++) {
            const [sx, sy] = mapToScreen(points[i][0], points[i][1]);
            ctx.lineTo(sx, sy);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    function drawScan() {
        ctx.fillStyle = '#06b6d4';
        ctx.globalAlpha = 0.7;

        for (const pt of state.scan.points) {
            // Points are already in map frame (transformed on backend)
            const [sx, sy] = mapToScreen(pt[0], pt[1]);
            ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
        }
        ctx.globalAlpha = 1;
    }

    function drawRobot() {
        const pose = state.pose;
        const [sx, sy] = mapToScreen(pose.x, pose.y);
        const size = Math.max(12, state.camera.zoom * 0.3);

        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(-pose.yaw);

        // Robot body — triangle pointing right (forward)
        ctx.fillStyle = '#3b82f6';
        ctx.shadowColor = '#3b82f680';
        ctx.shadowBlur = 15;
        ctx.beginPath();
        ctx.moveTo(size * 1.3, 0);
        ctx.lineTo(-size * 0.7, -size * 0.8);
        ctx.lineTo(-size * 0.4, 0);
        ctx.lineTo(-size * 0.7, size * 0.8);
        ctx.closePath();
        ctx.fill();
        ctx.shadowBlur = 0;

        // Direction line
        ctx.strokeStyle = '#60a5fa';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(size * 2, 0);
        ctx.stroke();

        // Center dot
        ctx.fillStyle = 'white';
        ctx.beginPath();
        ctx.arc(0, 0, 3, 0, Math.PI * 2);
        ctx.fill();

        ctx.restore();

        // Pose ring
        ctx.strokeStyle = '#3b82f640';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(sx, sy, size * 2, 0, Math.PI * 2);
        ctx.stroke();
    }

    function drawPlacementArrow() {
        const [sx, sy] = mapToScreen(state.placeStart.x, state.placeStart.y);
        const angle = -state.placeAngle;  // flip for screen coords

        const len = 40;
        const headLen = 14;

        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(angle);

        // Shaft
        const color = state.tool === 'initial_pose' ? '#10b981' : '#f59e0b';
        ctx.strokeStyle = color;
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(len, 0);
        ctx.stroke();

        // Arrowhead
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(len + headLen, 0);
        ctx.lineTo(len - 2, -8);
        ctx.lineTo(len - 2, 8);
        ctx.closePath();
        ctx.fill();

        // Ring
        ctx.strokeStyle = color + '40';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.arc(0, 0, 20, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);

        // Label
        ctx.fillStyle = color;
        ctx.font = '11px Inter';
        ctx.textAlign = 'center';
        const label = state.tool === 'initial_pose' ? 'POSE' : 'GOAL';
        ctx.fillText(label, 0, -28);

        ctx.restore();
    }

    // ─── WebSocket ────────────────────────────────────────
    function connect() {
        const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${proto}//${location.host}/ws`;

        state.ws = new WebSocket(url);

        state.ws.onopen = () => {
            state.connected = true;
            connStatus.classList.add('connected');
            connStatus.querySelector('.status-text').textContent = 'Connected';
        };

        state.ws.onclose = () => {
            state.connected = false;
            connStatus.classList.remove('connected');
            connStatus.querySelector('.status-text').textContent = 'Disconnected';
            setTimeout(connect, 2000);
        };

        state.ws.onerror = () => {
            state.ws.close();
        };

        state.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                handleMessage(data);
            } catch (e) {
                console.warn('Bad WS message:', e);
            }
        };
    }

    function handleMessage(data) {
        switch (data.type) {
            case 'map':
                state.map = {
                    width: data.width,
                    height: data.height,
                    resolution: data.resolution,
                    origin_x: data.origin_x,
                    origin_y: data.origin_y,
                    origin_yaw: data.origin_yaw,
                };
                state.mapImage = renderMapImage(state.map, data.data);
                els.statMapSize.textContent = `${data.width}×${data.height}`;
                els.statMapRes.textContent = `${(data.resolution * 100).toFixed(0)} cm`;
                break;

            case 'costmap':
                state.costmapMeta = {
                    width: data.width,
                    height: data.height,
                    resolution: data.resolution,
                    origin_x: data.origin_x,
                    origin_y: data.origin_y,
                };
                state.costmapImage = renderCostmapImage(state.costmapMeta, data.data);
                break;

            case 'pose':
                state.pose = { x: data.x, y: data.y, yaw: data.yaw };
                els.statX.textContent = data.x.toFixed(2) + ' m';
                els.statY.textContent = data.y.toFixed(2) + ' m';
                els.statYaw.textContent = (data.yaw * 180 / Math.PI).toFixed(1) + '°';

                if (state.followRobot) {
                    state.camera.x = data.x;
                    state.camera.y = data.y;
                }
                break;

            case 'scan':
                state.scan = { points: data.points };
                break;

            case 'global_plan':
                state.globalPlan = data.points;
                break;

            case 'local_plan':
                state.localPlan = data.points;
                break;

            case 'nav_status':
                state.navStatus = data.status;
                els.navStatusText.textContent = data.status.toUpperCase();
                els.navStatusBadge.className = 'nav-status-badge ' + data.status;
                break;
        }
    }

    // ─── Mouse Interaction ────────────────────────────────
    function getMousePos(e) {
        const rect = canvas.getBoundingClientRect();
        return [
            (e.clientX - rect.left) * devicePixelRatio,
            (e.clientY - rect.top) * devicePixelRatio,
        ];
    }

    canvas.addEventListener('mousedown', (e) => {
        const [mx, my] = getMousePos(e);

        if (state.tool === 'pan' || e.button === 1 || (e.button === 0 && e.shiftKey)) {
            // Pan
            state.dragging = true;
            state.dragStart = [mx, my];
            state.cameraStart = { ...state.camera };
            canvas.style.cursor = 'grabbing';
            return;
        }

        if (state.tool === 'initial_pose' || state.tool === 'nav_goal') {
            const [mapX, mapY] = screenToMap(mx, my);
            state.placing = true;
            state.placeStart = { x: mapX, y: mapY };
            state.placeAngle = 0;
            arrowHint.classList.remove('hidden');
            arrowHint.querySelector('span').textContent = 'Drag to set orientation, then release';
        }
    });

    canvas.addEventListener('mousemove', (e) => {
        const [mx, my] = getMousePos(e);
        const [mapX, mapY] = screenToMap(mx, my);
        coordText.textContent = `x: ${mapX.toFixed(2)}, y: ${mapY.toFixed(2)}`;

        if (state.dragging) {
            const dx = (mx - state.dragStart[0]) / state.camera.zoom;
            const dy = (my - state.dragStart[1]) / state.camera.zoom;
            state.camera.x = state.cameraStart.x - dx;
            state.camera.y = state.cameraStart.y + dy;
            return;
        }

        if (state.placing && state.placeStart) {
            const dx = mapX - state.placeStart.x;
            const dy = mapY - state.placeStart.y;
            if (Math.hypot(dx, dy) > 0.05) {
                state.placeAngle = Math.atan2(dy, dx);
            }
        }
    });

    canvas.addEventListener('mouseup', (e) => {
        if (state.dragging) {
            state.dragging = false;
            canvas.style.cursor = state.tool === 'pan' ? 'grab' : 'crosshair';
            return;
        }

        if (state.placing && state.placeStart) {
            const cmd = state.tool;
            const payload = {
                cmd: cmd,
                x: state.placeStart.x,
                y: state.placeStart.y,
                yaw: state.placeAngle,
            };
            if (state.ws && state.connected) {
                state.ws.send(JSON.stringify(payload));
            }
            state.placing = false;
            state.placeStart = null;
            arrowHint.classList.add('hidden');
        }
    });

    // Zoom
    canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const [mx, my] = getMousePos(e);
        const [mapX, mapY] = screenToMap(mx, my);

        const factor = e.deltaY > 0 ? 0.9 : 1.1;
        state.camera.zoom = Math.max(2, Math.min(500, state.camera.zoom * factor));

        // Zoom towards cursor
        const [newMapX, newMapY] = screenToMap(mx, my);
        state.camera.x -= newMapX - mapX;
        state.camera.y -= newMapY - mapY;

        zoomText.textContent = Math.round(state.camera.zoom / 40 * 100) + '%';
    }, { passive: false });

    // Prevent context menu
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());

    // ─── Keyboard Shortcuts ───────────────────────────────
    document.addEventListener('keydown', (e) => {
        if (e.target.tagName === 'INPUT') return;

        switch (e.key.toLowerCase()) {
            case 'p':
                selectTool('pan');
                break;
            case 'i':
                selectTool('initial_pose');
                break;
            case 'g':
                selectTool('nav_goal');
                break;
            case 'escape':
                if (state.placing) {
                    state.placing = false;
                    state.placeStart = null;
                    arrowHint.classList.add('hidden');
                } else {
                    cancelNav();
                }
                break;
            case 'f':
                fitMap();
                break;
            case 'c':
                centerRobot();
                break;
        }
    });

    // ─── Tool Selection ───────────────────────────────────
    function selectTool(tool) {
        state.tool = tool;
        document.querySelectorAll('.tool-btn[data-tool]').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.tool === tool);
        });
        canvas.style.cursor = tool === 'pan' ? 'grab' : 'crosshair';

        // Update hint
        if (tool === 'initial_pose') {
            arrowHint.querySelector('span').textContent = 'Click on map to set initial pose, drag for orientation';
        } else if (tool === 'nav_goal') {
            arrowHint.querySelector('span').textContent = 'Click on map to set nav goal, drag for orientation';
        }
    }

    document.querySelectorAll('.tool-btn[data-tool]').forEach(btn => {
        btn.addEventListener('click', () => selectTool(btn.dataset.tool));
    });

    document.getElementById('btnCancel').addEventListener('click', cancelNav);

    function cancelNav() {
        if (state.ws && state.connected) {
            state.ws.send(JSON.stringify({ cmd: 'cancel_nav' }));
        }
    }

    // ─── View Controls ────────────────────────────────────
    function fitMap() {
        if (!state.map) return;
        const m = state.map;
        const mapW = m.width * m.resolution;
        const mapH = m.height * m.resolution;
        const cx = m.origin_x + mapW / 2;
        const cy = m.origin_y + mapH / 2;
        state.camera.x = cx;
        state.camera.y = cy;

        const zx = canvas.width / (mapW * 1.2);
        const zy = canvas.height / (mapH * 1.2);
        state.camera.zoom = Math.min(zx, zy);
        zoomText.textContent = Math.round(state.camera.zoom / 40 * 100) + '%';
    }

    function centerRobot() {
        if (!state.pose) return;
        state.camera.x = state.pose.x;
        state.camera.y = state.pose.y;
    }

    document.getElementById('btnFitMap').addEventListener('click', fitMap);
    document.getElementById('btnCenterRobot').addEventListener('click', centerRobot);

    // Follow robot toggle
    document.getElementById('followRobot').addEventListener('change', (e) => {
        state.followRobot = e.target.checked;
    });

    // ─── Layer Toggles ────────────────────────────────────
    const layerMap = { layerMap: 'map', layerCostmap: 'costmap', layerScan: 'scan', layerRobot: 'robot', layerPath: 'path', layerLocalPath: 'localPath' };
    for (const [id, key] of Object.entries(layerMap)) {
        document.getElementById(id).addEventListener('change', (e) => {
            state.layers[key] = e.target.checked;
        });
    }

    // ─── Periodic Stats Update ────────────────────────────
    setInterval(() => {
        if (!state.connected) return;

        fetch('/api/status')
            .then(r => r.json())
            .then(data => {
                if (data.cmd_vel) {
                    els.statLinVel.textContent = data.cmd_vel.linear_x.toFixed(2) + ' m/s';
                    els.statAngVel.textContent = data.cmd_vel.angular_z.toFixed(2) + ' r/s';
                }
                if (data.battery) {
                    els.statBattery.textContent = data.battery.percentage.toFixed(0) + '%';
                }
            })
            .catch(() => {});
    }, 1000);

    // ─── Init ─────────────────────────────────────────────
    connect();
    selectTool('initial_pose');
    requestAnimationFrame(draw);

})();

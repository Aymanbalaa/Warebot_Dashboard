/* ═══════════════════════════════════════════════════════════
   NAVDASH MOBILE — Touch-first map interaction

   Touch gestures:
   - 1 finger pan (when in pan mode or no tool)
   - 2 finger pinch-zoom + pan
   - 1 finger tap + drag to place pose/goal with orientation
   - Tap with tool active → place at 0° heading
   ═══════════════════════════════════════════════════════════ */

(() => {
    "use strict";

    // ─── State ────────────────────────────────────────────
    const S = {
        ws: null,
        connected: false,

        map: null,
        mapImage: null,
        costmapImage: null,
        costmapMeta: null,

        pose: null,
        scan: null,
        odom: null,
        cmdVel: null,
        battery: null,
        navStatus: 'idle',
        globalPlan: null,
        localPlan: null,

        camera: { x: 0, y: 0, zoom: 40 },

        tool: 'initial_pose',

        layers: {
            map: true, costmap: false, scan: true,
            robot: true, path: true, localPath: true,
        },
        followRobot: false,

        // Touch state
        touches: {},          // active touches by id
        touchMode: null,      // null, 'pan', 'place', 'pinch'
        pinchStartDist: 0,
        pinchStartZoom: 0,
        pinchCenter: null,
        panStart: null,
        cameraStart: null,

        // Placement
        placing: false,
        placeStart: null,
        placeAngle: 0,
        placeDragged: false,  // did user drag after initial tap?
    };

    // ─── DOM ──────────────────────────────────────────────
    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');
    const coordText = document.getElementById('coordText');
    const zoomText = document.getElementById('zoomText');
    const connStatus = document.getElementById('connStatus');
    const placeHint = document.getElementById('placeHint');
    const placeHintText = document.getElementById('placeHintText');

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
        navBadge: document.getElementById('navBadge'),
    };

    // ─── Canvas Sizing ────────────────────────────────────
    function resize() {
        const dpr = devicePixelRatio || 1;
        canvas.width = window.innerWidth * dpr;
        canvas.height = window.innerHeight * dpr;
        canvas.style.width = window.innerWidth + 'px';
        canvas.style.height = window.innerHeight + 'px';
    }
    window.addEventListener('resize', resize);
    resize();

    // ─── Coord Transforms ─────────────────────────────────
    function mapToScreen(mx, my) {
        const sx = (mx - S.camera.x) * S.camera.zoom + canvas.width / 2;
        const sy = -(my - S.camera.y) * S.camera.zoom + canvas.height / 2;
        return [sx, sy];
    }

    function screenToMap(sx, sy) {
        const mx = (sx - canvas.width / 2) / S.camera.zoom + S.camera.x;
        const my = -(sy - canvas.height / 2) / S.camera.zoom + S.camera.y;
        return [mx, my];
    }

    function cssToCanvas(clientX, clientY) {
        const dpr = devicePixelRatio || 1;
        return [clientX * dpr, clientY * dpr];
    }

    // ─── Map Rendering ────────────────────────────────────
    function renderMapImage(meta, dataB64) {
        const raw = atob(dataB64);
        const w = meta.width, h = meta.height;
        const off = new OffscreenCanvas(w, h);
        const octx = off.getContext('2d');
        const img = octx.createImageData(w, h);
        const px = img.data;

        for (let i = 0; i < raw.length; i++) {
            let v = raw.charCodeAt(i);
            let r, g, b, a;
            if (v === 255) { r=30; g=35; b=50; a=180; }
            else if (v === 0) { r=25; g=32; b=48; a=255; }
            else if (v <= 50) { const t=v/50; r=25+t*40|0; g=32+t*20|0; b=48+t*30|0; a=255; }
            else { const t=Math.min(1,(v-50)/50); r=40+t*80|0; g=80+t*140|0; b=160+t*95|0; a=255; }

            const row = h - 1 - (i / w | 0);
            const col = i % w;
            const pi = (row * w + col) * 4;
            px[pi]=r; px[pi+1]=g; px[pi+2]=b; px[pi+3]=a;
        }
        octx.putImageData(img, 0, 0);
        return off;
    }

    function renderCostmapImage(meta, dataB64) {
        const raw = atob(dataB64);
        const w = meta.width, h = meta.height;
        const off = new OffscreenCanvas(w, h);
        const octx = off.getContext('2d');
        const img = octx.createImageData(w, h);
        const px = img.data;

        for (let i = 0; i < raw.length; i++) {
            let v = raw.charCodeAt(i);
            const row = h - 1 - (i / w | 0);
            const col = i % w;
            const pi = (row * w + col) * 4;
            if (v === 0 || v === 255) { px[pi+3]=0; }
            else {
                const t=v/100;
                px[pi]=100+t*155|0; px[pi+1]=20*(1-t)|0; px[pi+2]=180*(1-t)|0; px[pi+3]=60+t*120|0;
            }
        }
        octx.putImageData(img, 0, 0);
        return off;
    }

    // ─── Drawing ──────────────────────────────────────────
    function draw() {
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = '#080c14';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        drawGrid();

        if (S.layers.map && S.mapImage && S.map) drawMapLayer(S.mapImage, S.map);
        if (S.layers.costmap && S.costmapImage && S.costmapMeta) drawMapLayer(S.costmapImage, S.costmapMeta);
        if (S.layers.path && S.globalPlan) drawPath(S.globalPlan, '#3b82f6', 2.5, [8, 4]);
        if (S.layers.localPath && S.localPlan) drawPath(S.localPlan, '#8b5cf6', 2, []);
        if (S.layers.scan && S.scan && S.pose) drawScan();
        if (S.layers.robot && S.pose) drawRobot();
        if (S.placing && S.placeStart) drawPlacementArrow();

        requestAnimationFrame(draw);
    }

    function drawGrid() {
        const z = S.camera.zoom;
        let gs = 1;
        if (z < 15) gs = 5;
        if (z < 5) gs = 10;
        if (z > 80) gs = 0.5;

        const [l, t] = screenToMap(0, 0);
        const [r, b] = screenToMap(canvas.width, canvas.height);

        ctx.strokeStyle = '#111827';
        ctx.lineWidth = 1;
        ctx.beginPath();
        const sx0 = Math.floor(l / gs) * gs;
        const sy0 = Math.floor(b / gs) * gs;
        for (let x = sx0; x <= r; x += gs) { const [sx] = mapToScreen(x, 0); ctx.moveTo(sx, 0); ctx.lineTo(sx, canvas.height); }
        for (let y = sy0; y <= t; y += gs) { const [, sy] = mapToScreen(0, y); ctx.moveTo(0, sy); ctx.lineTo(canvas.width, sy); }
        ctx.stroke();

        // Origin axes
        const [ox, oy] = mapToScreen(0, 0);
        ctx.lineWidth = 1.5;
        ctx.strokeStyle = '#ef444460';
        ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + 60 * z / 40, oy); ctx.stroke();
        ctx.strokeStyle = '#10b98160';
        ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - 60 * z / 40); ctx.stroke();
    }

    function drawMapLayer(image, meta) {
        const z = S.camera.zoom;
        const w = meta.width * meta.resolution;
        const h = meta.height * meta.resolution;
        const [sx, sy] = mapToScreen(meta.origin_x, meta.origin_y + h);
        ctx.imageSmoothingEnabled = z < 60;
        ctx.drawImage(image, sx, sy, w * z, h * z);
    }

    function drawPath(pts, color, width, dash) {
        if (pts.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.setLineDash(dash);
        ctx.lineJoin = 'round';
        ctx.beginPath();
        const [sx0, sy0] = mapToScreen(pts[0][0], pts[0][1]);
        ctx.moveTo(sx0, sy0);
        for (let i = 1; i < pts.length; i++) {
            const [sx, sy] = mapToScreen(pts[i][0], pts[i][1]);
            ctx.lineTo(sx, sy);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    function drawScan() {
        ctx.fillStyle = '#06b6d4';
        ctx.globalAlpha = 0.7;
        for (const pt of S.scan.points) {
            // Points are already in map frame (transformed on backend)
            const [sx, sy] = mapToScreen(pt[0], pt[1]);
            ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
        }
        ctx.globalAlpha = 1;
    }

    function drawRobot() {
        const p = S.pose;
        const [sx, sy] = mapToScreen(p.x, p.y);
        const sz = Math.max(14, S.camera.zoom * 0.35);

        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(-p.yaw);

        ctx.fillStyle = '#3b82f6';
        ctx.shadowColor = '#3b82f680';
        ctx.shadowBlur = 18;
        ctx.beginPath();
        ctx.moveTo(sz * 1.3, 0);
        ctx.lineTo(-sz * 0.7, -sz * 0.8);
        ctx.lineTo(-sz * 0.4, 0);
        ctx.lineTo(-sz * 0.7, sz * 0.8);
        ctx.closePath();
        ctx.fill();
        ctx.shadowBlur = 0;

        ctx.strokeStyle = '#60a5fa';
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(sz * 2, 0); ctx.stroke();

        ctx.fillStyle = 'white';
        ctx.beginPath(); ctx.arc(0, 0, 3, 0, Math.PI * 2); ctx.fill();
        ctx.restore();

        ctx.strokeStyle = '#3b82f640';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(sx, sy, sz * 2, 0, Math.PI * 2); ctx.stroke();
    }

    function drawPlacementArrow() {
        const [sx, sy] = mapToScreen(S.placeStart.x, S.placeStart.y);
        const angle = -S.placeAngle;
        const len = 45, headLen = 16;
        const color = S.tool === 'initial_pose' ? '#10b981' : '#f59e0b';

        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(angle);

        ctx.strokeStyle = color;
        ctx.lineWidth = 3.5;
        ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(len, 0); ctx.stroke();

        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(len + headLen, 0);
        ctx.lineTo(len - 2, -9);
        ctx.lineTo(len - 2, 9);
        ctx.closePath();
        ctx.fill();

        // Pulsing ring
        ctx.strokeStyle = color + '50';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.beginPath(); ctx.arc(0, 0, 24, 0, Math.PI * 2); ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = color;
        ctx.font = '600 12px Inter';
        ctx.textAlign = 'center';
        ctx.fillText(S.tool === 'initial_pose' ? 'POSE' : 'GOAL', 0, -34);

        ctx.restore();
    }

    // ─── Touch Handling ───────────────────────────────────

    function getTouchList(e) {
        return Array.from(e.touches);
    }

    function touchDist(a, b) {
        const dx = a.clientX - b.clientX;
        const dy = a.clientY - b.clientY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    function touchCenter(a, b) {
        return [(a.clientX + b.clientX) / 2, (a.clientY + b.clientY) / 2];
    }

    canvas.addEventListener('touchstart', (e) => {
        e.preventDefault();
        const touches = getTouchList(e);

        if (touches.length === 2) {
            // Pinch zoom — always takes priority
            S.touchMode = 'pinch';
            S.placing = false;
            S.pinchStartDist = touchDist(touches[0], touches[1]);
            S.pinchStartZoom = S.camera.zoom;
            const [cx, cy] = touchCenter(touches[0], touches[1]);
            S.pinchCenter = cssToCanvas(cx, cy);
            S.panStart = cssToCanvas(cx, cy);
            S.cameraStart = { ...S.camera };
            return;
        }

        if (touches.length === 1) {
            const t = touches[0];
            const [cx, cy] = cssToCanvas(t.clientX, t.clientY);

            if (S.tool === 'initial_pose' || S.tool === 'nav_goal') {
                // Start placement
                const [mx, my] = screenToMap(cx, cy);
                S.touchMode = 'place';
                S.placing = true;
                S.placeStart = { x: mx, y: my };
                S.placeAngle = 0;
                S.placeDragged = false;
                S.panStart = [cx, cy];
                placeHint.classList.remove('hidden');
                placeHintText.textContent = 'Drag to set orientation, release to confirm';
            } else {
                // Pan
                S.touchMode = 'pan';
                S.panStart = [cx, cy];
                S.cameraStart = { ...S.camera };
            }
        }
    }, { passive: false });

    canvas.addEventListener('touchmove', (e) => {
        e.preventDefault();
        const touches = getTouchList(e);

        if (S.touchMode === 'pinch' && touches.length >= 2) {
            const dist = touchDist(touches[0], touches[1]);
            const scale = dist / S.pinchStartDist;
            const newZoom = Math.max(2, Math.min(500, S.pinchStartZoom * scale));

            // Zoom towards pinch center
            const [mapXBefore, mapYBefore] = screenToMap(S.pinchCenter[0], S.pinchCenter[1]);
            S.camera.zoom = newZoom;
            const [mapXAfter, mapYAfter] = screenToMap(S.pinchCenter[0], S.pinchCenter[1]);
            S.camera.x -= mapXAfter - mapXBefore;
            S.camera.y -= mapYAfter - mapYBefore;

            // Pan with pinch center movement
            const [cx, cy] = touchCenter(touches[0], touches[1]);
            const [ncx, ncy] = cssToCanvas(cx, cy);
            const dx = (ncx - S.panStart[0]) / S.camera.zoom;
            const dy = (ncy - S.panStart[1]) / S.camera.zoom;
            S.camera.x -= dx;
            S.camera.y += dy;
            S.panStart = [ncx, ncy];

            zoomText.textContent = Math.round(S.camera.zoom / 40 * 100) + '%';
            return;
        }

        if (touches.length === 1) {
            const t = touches[0];
            const [cx, cy] = cssToCanvas(t.clientX, t.clientY);

            if (S.touchMode === 'pan') {
                const dx = (cx - S.panStart[0]) / S.camera.zoom;
                const dy = (cy - S.panStart[1]) / S.camera.zoom;
                S.camera.x = S.cameraStart.x - dx;
                S.camera.y = S.cameraStart.y + dy;
            }

            if (S.touchMode === 'place' && S.placeStart) {
                const [mx, my] = screenToMap(cx, cy);
                const ddx = mx - S.placeStart.x;
                const ddy = my - S.placeStart.y;
                const dist = Math.sqrt(ddx * ddx + ddy * ddy);

                if (dist > 0.08) {
                    S.placeAngle = Math.atan2(ddy, ddx);
                    S.placeDragged = true;
                }

                // If dragged far from start without much angle movement, switch to pan
                const screenDist = Math.sqrt(
                    (cx - S.panStart[0]) ** 2 + (cy - S.panStart[1]) ** 2
                );
                // Don't auto-switch to pan — placement always uses single finger
            }

            // Update coord display
            const [mx, my] = screenToMap(cx, cy);
            coordText.textContent = `x: ${mx.toFixed(2)}, y: ${my.toFixed(2)}`;
        }
    }, { passive: false });

    canvas.addEventListener('touchend', (e) => {
        e.preventDefault();
        const remaining = getTouchList(e);

        if (S.touchMode === 'pinch') {
            if (remaining.length < 2) {
                S.touchMode = null;
                // If one finger remains, start pan from it
                if (remaining.length === 1) {
                    const [cx, cy] = cssToCanvas(remaining[0].clientX, remaining[0].clientY);
                    S.touchMode = 'pan';
                    S.panStart = [cx, cy];
                    S.cameraStart = { ...S.camera };
                }
            }
            return;
        }

        if (S.touchMode === 'place' && S.placeStart) {
            // Send the placement command
            const payload = {
                cmd: S.tool,
                x: S.placeStart.x,
                y: S.placeStart.y,
                yaw: S.placeAngle,
            };
            wsSend(payload);
            S.placing = false;
            S.placeStart = null;
            S.touchMode = null;
            placeHint.classList.add('hidden');

            // Brief feedback vibration
            if (navigator.vibrate) navigator.vibrate(30);
            return;
        }

        if (remaining.length === 0) {
            S.touchMode = null;
        }
    }, { passive: false });

    canvas.addEventListener('touchcancel', (e) => {
        S.touchMode = null;
        S.placing = false;
        placeHint.classList.add('hidden');
    });

    // ─── Also support mouse for testing on desktop ────────
    let mouseDown = false;
    canvas.addEventListener('mousedown', (e) => {
        const [cx, cy] = [e.offsetX * devicePixelRatio, e.offsetY * devicePixelRatio];
        if (S.tool === 'initial_pose' || S.tool === 'nav_goal') {
            const [mx, my] = screenToMap(cx, cy);
            S.touchMode = 'place';
            S.placing = true;
            S.placeStart = { x: mx, y: my };
            S.placeAngle = 0;
            S.panStart = [cx, cy];
            mouseDown = true;
        } else {
            S.touchMode = 'pan';
            S.panStart = [cx, cy];
            S.cameraStart = { ...S.camera };
            mouseDown = true;
        }
    });

    canvas.addEventListener('mousemove', (e) => {
        const [cx, cy] = [e.offsetX * devicePixelRatio, e.offsetY * devicePixelRatio];
        const [mx, my] = screenToMap(cx, cy);
        coordText.textContent = `x: ${mx.toFixed(2)}, y: ${my.toFixed(2)}`;

        if (!mouseDown) return;

        if (S.touchMode === 'pan') {
            const dx = (cx - S.panStart[0]) / S.camera.zoom;
            const dy = (cy - S.panStart[1]) / S.camera.zoom;
            S.camera.x = S.cameraStart.x - dx;
            S.camera.y = S.cameraStart.y + dy;
        }

        if (S.touchMode === 'place' && S.placeStart) {
            const ddx = mx - S.placeStart.x;
            const ddy = my - S.placeStart.y;
            if (Math.hypot(ddx, ddy) > 0.05) S.placeAngle = Math.atan2(ddy, ddx);
        }
    });

    canvas.addEventListener('mouseup', () => {
        if (S.touchMode === 'place' && S.placeStart) {
            wsSend({ cmd: S.tool, x: S.placeStart.x, y: S.placeStart.y, yaw: S.placeAngle });
            S.placing = false;
            S.placeStart = null;
        }
        S.touchMode = null;
        mouseDown = false;
    });

    canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const [cx, cy] = [e.offsetX * devicePixelRatio, e.offsetY * devicePixelRatio];
        const [mapX, mapY] = screenToMap(cx, cy);
        S.camera.zoom = Math.max(2, Math.min(500, S.camera.zoom * (e.deltaY > 0 ? 0.9 : 1.1)));
        const [nmx, nmy] = screenToMap(cx, cy);
        S.camera.x -= nmx - mapX;
        S.camera.y -= nmy - mapY;
        zoomText.textContent = Math.round(S.camera.zoom / 40 * 100) + '%';
    }, { passive: false });

    // ─── WebSocket ────────────────────────────────────────
    function connect() {
        const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
        S.ws = new WebSocket(`${proto}//${location.host}/ws`);

        S.ws.onopen = () => {
            S.connected = true;
            connStatus.classList.add('connected');
        };

        S.ws.onclose = () => {
            S.connected = false;
            connStatus.classList.remove('connected');
            setTimeout(connect, 2000);
        };

        S.ws.onerror = () => S.ws.close();

        S.ws.onmessage = (e) => {
            try { handleMsg(JSON.parse(e.data)); } catch {}
        };
    }

    function wsSend(obj) {
        if (S.ws && S.connected) S.ws.send(JSON.stringify(obj));
    }

    function handleMsg(d) {
        switch (d.type) {
            case 'map':
                S.map = { width: d.width, height: d.height, resolution: d.resolution, origin_x: d.origin_x, origin_y: d.origin_y, origin_yaw: d.origin_yaw };
                S.mapImage = renderMapImage(S.map, d.data);
                els.statMapSize.textContent = `${d.width}×${d.height}`;
                els.statMapRes.textContent = `${(d.resolution * 100).toFixed(0)}cm`;
                break;
            case 'costmap':
                S.costmapMeta = { width: d.width, height: d.height, resolution: d.resolution, origin_x: d.origin_x, origin_y: d.origin_y };
                S.costmapImage = renderCostmapImage(S.costmapMeta, d.data);
                break;
            case 'pose':
                S.pose = { x: d.x, y: d.y, yaw: d.yaw };
                els.statX.textContent = d.x.toFixed(2);
                els.statY.textContent = d.y.toFixed(2);
                els.statYaw.textContent = (d.yaw * 180 / Math.PI).toFixed(1) + '°';
                if (S.followRobot) { S.camera.x = d.x; S.camera.y = d.y; }
                break;
            case 'scan':
                S.scan = { points: d.points };
                break;
            case 'global_plan':
                S.globalPlan = d.points;
                break;
            case 'local_plan':
                S.localPlan = d.points;
                break;
            case 'nav_status':
                S.navStatus = d.status;
                els.navStatusText.textContent = d.status.toUpperCase();
                els.navBadge.className = 'nav-badge ' + d.status;
                if (d.status === 'succeeded' && navigator.vibrate) navigator.vibrate([50, 50, 50]);
                if (d.status === 'aborted' && navigator.vibrate) navigator.vibrate([100, 50, 100, 50, 100]);
                break;
        }
    }

    // ─── UI Wiring ────────────────────────────────────────

    // Tool selection
    function selectTool(tool) {
        S.tool = tool;
        document.querySelectorAll('.dock-btn[data-tool]').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.tool === tool);
        });
        if (tool === 'initial_pose') placeHintText.textContent = 'Tap on map, drag to set pose orientation';
        if (tool === 'nav_goal') placeHintText.textContent = 'Tap on map, drag to set goal orientation';
    }

    document.querySelectorAll('.dock-btn[data-tool]').forEach(btn => {
        btn.addEventListener('click', () => selectTool(btn.dataset.tool));
    });

    // Cancel navigation
    document.getElementById('btnCancel').addEventListener('click', () => {
        wsSend({ cmd: 'cancel_nav' });
        if (navigator.vibrate) navigator.vibrate(50);
    });

    // Center robot
    const btnCenter = document.getElementById('btnCenter');
    if (btnCenter) {
        btnCenter.addEventListener('click', () => {
            if (S.pose) { S.camera.x = S.pose.x; S.camera.y = S.pose.y; }
        });
    }

    // Stats drawer toggle via menu button
    const statsDrawer = document.getElementById('statsDrawer');
    document.getElementById('btnMenu').addEventListener('click', () => {
        statsDrawer.classList.toggle('hidden');
    });

    // Layers sheet
    const layersSheet = document.getElementById('layersSheet');
    document.getElementById('btnLayers').addEventListener('click', () => {
        layersSheet.classList.remove('hidden');
    });
    document.getElementById('layersBackdrop').addEventListener('click', () => {
        layersSheet.classList.add('hidden');
    });

    // Layer toggles
    const layerIds = { layerMap: 'map', layerCostmap: 'costmap', layerScan: 'scan', layerRobot: 'robot', layerPath: 'path', layerLocalPath: 'localPath' };
    for (const [id, key] of Object.entries(layerIds)) {
        document.getElementById(id).addEventListener('change', (e) => { S.layers[key] = e.target.checked; });
    }
    document.getElementById('followRobot').addEventListener('change', (e) => { S.followRobot = e.target.checked; });

    // Periodic stats fetch
    setInterval(() => {
        if (!S.connected) return;
        fetch('/api/status').then(r => r.json()).then(d => {
            if (d.cmd_vel) {
                els.statLinVel.textContent = d.cmd_vel.linear_x.toFixed(2) + ' m/s';
                els.statAngVel.textContent = d.cmd_vel.angular_z.toFixed(2) + ' r/s';
            }
            if (d.battery) els.statBattery.textContent = d.battery.percentage.toFixed(0) + '%';
        }).catch(() => {});
    }, 1500);

    // ─── Init ─────────────────────────────────────────────
    connect();
    selectTool('pan');
    requestAnimationFrame(draw);

})();

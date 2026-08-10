import os
import threading
import time
import math

import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from flask import Flask, Response, request, jsonify

from zed_common import config
from zed_common import mission_targets as mt
from zed_common import path_planner as pp
from zed_common import mission_state as ms
from zed_common.coord_utils import FloorPlanMeta, world_to_pixel, in_bounds, pixel_to_world

KR_FONT_PATH = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
try:
    _KR_FONT = ImageFont.truetype(KR_FONT_PATH, 18)
except OSError:
    _KR_FONT = ImageFont.load_default()


def put_text_kr(img_bgr, text, org, color_bgr):
    img_pil = PILImage.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)
    color_rgb = (color_bgr[2], color_bgr[1], color_bgr[0])
    draw.text(org, text, font=_KR_FONT, fill=color_rgb)
    return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _orient(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])


def _on_segment(a, b, c):
    return (min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9 and
            min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9)


def segments_intersect(p1, p2, p3, p4):
    """이동 선분(p1->p2)이 게이트 선분(p3->p4)을 실제로 가로질렀는지 판정."""
    d1, d2 = _orient(p3, p4, p1), _orient(p3, p4, p2)
    d3, d4 = _orient(p1, p2, p3), _orient(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    if d1 == 0 and _on_segment(p3, p4, p1):
        return True
    if d2 == 0 and _on_segment(p3, p4, p2):
        return True
    if d3 == 0 and _on_segment(p1, p2, p3):
        return True
    if d4 == 0 and _on_segment(p1, p2, p4):
        return True
    return False


class DashboardNode(Node):
    def __init__(self):
        super().__init__('zed_dashboard_node')
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_frame = None
        self.latest_pose = None
        self.trail = []
        self.intrinsics = None

        self.floor_img = None
        self.floor_meta = None
        self._floor_mtime = None

        self.targets = mt.load_targets()
        self.active_target = next(iter(self.targets), None)
        self._targets_mtime = None

        self.path = []
        self.current_gate_idx = 0
        self._path_mtime = None
        self._progress_mtime = None

        self.create_subscription(Image, config.IMAGE_TOPIC, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, config.CAMERA_INFO_TOPIC, self.on_caminfo, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, config.POSE_TOPIC, self.on_pose, 20)

        self._reload_floor_plan()
        self._reload_path(reset_progress=True)
        self.get_logger().info("대시보드 노드 시작")

    # ---- 리로드 ----
    def _reload_floor_plan(self):
        try:
            mtime = os.path.getmtime(config.FLOOR_PLAN_META_PATH)
        except OSError:
            return
        if mtime == self._floor_mtime:
            return
        img = cv2.imread(config.FLOOR_PLAN_IMAGE_PATH)
        if img is None:
            return
        try:
            meta = FloorPlanMeta.load(config.FLOOR_PLAN_META_PATH)
        except Exception as e:
            self.get_logger().warn(f"평면도 메타 로드 실패: {e}", throttle_duration_sec=5.0)
            return
        with self.lock:
            self.floor_img = img
            self.floor_meta = meta
        self._floor_mtime = mtime

    def _reload_targets_if_changed(self):
        try:
            mtime = os.path.getmtime(config.MISSION_TARGETS_PATH)
        except OSError:
            return
        if mtime == self._targets_mtime:
            return
        with self.lock:
            self.targets = mt.load_targets()
            if self.active_target not in self.targets:
                self.active_target = next(iter(self.targets), None)
        self._targets_mtime = mtime

    def _reload_path(self, reset_progress=False):
        try:
            mtime = os.path.getmtime(config.MISSION_PATH_PATH)
        except OSError:
            return
        if mtime != self._path_mtime:
            path = pp.load_path()
            with self.lock:
                self.path = path
            self._path_mtime = mtime
        self._reload_progress()

    def _reload_progress(self):
        try:
            mtime = os.path.getmtime(config.MISSION_PROGRESS_PATH)
        except OSError:
            return
        if mtime == self._progress_mtime:
            return
        prog = pp.load_progress()
        with self.lock:
            self.current_gate_idx = prog.get("current_gate_idx", 0)
        self._progress_mtime = mtime

    # ---- 구독 콜백 ----
    def on_caminfo(self, msg: CameraInfo):
        k = msg.k
        self.intrinsics = (k[0], k[4], k[2], k[5])

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.latest_frame = frame

    def on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        with self.lock:
            self.latest_pose = (p.x, p.y, yaw)
            self.trail.append((p.x, p.y))
            if len(self.trail) > config.PATH_TRAIL_MAX_POINTS:
                self.trail.pop(0)

    # ---- 렌더링 ----
    def render_map(self):
        self._reload_floor_plan()
        self._reload_targets_if_changed()
        self._reload_path()
        with self.lock:
            floor_img = self.floor_img
            floor_meta = self.floor_meta
            trail = list(self.trail)
            targets = dict(self.targets)
            active = self.active_target
            path = list(self.path)
            gate_idx = self.current_gate_idx

        if floor_img is None or floor_meta is None:
            placeholder = np.full((480, 480, 3), 255, np.uint8)
            return put_text_kr(placeholder, "평면도 없음 - 매핑 노드를 켜고 기다려주세요", (15, 220), (0, 0, 0))

        img = floor_img.copy()
        pixel_trail = []
        for x, y in trail:
            u, v = world_to_pixel(x, y, floor_meta)
            if in_bounds(u, v, floor_meta):
                pixel_trail.append((u, v))
        for i in range(1, len(pixel_trail)):
            cv2.line(img, pixel_trail[i - 1], pixel_trail[i], (200, 150, 0), 1)

        # 게이트 경로선 + 진행상황
        gate_pixels = []
        for g in path:
            u, v = world_to_pixel(g['x'], g['y'], floor_meta)
            gate_pixels.append((u, v) if in_bounds(u, v, floor_meta) else None)
        for i in range(1, len(gate_pixels)):
            if gate_pixels[i - 1] and gate_pixels[i]:
                cv2.line(img, gate_pixels[i - 1], gate_pixels[i], config.GATE_LINE_COLOR, 2)
        for i, gp in enumerate(gate_pixels):
            if i < gate_idx:
                color = config.GATE_DONE_COLOR
            elif i == gate_idx:
                color = config.GATE_CURRENT_COLOR
            else:
                color = config.GATE_LINE_COLOR
            g = path[i]
            if 'left_x' in g:
                lu, lv = world_to_pixel(g['left_x'], g['left_y'], floor_meta)
                ru, rv = world_to_pixel(g['right_x'], g['right_y'], floor_meta)
                if in_bounds(lu, lv, floor_meta) and in_bounds(ru, rv, floor_meta):
                    cv2.line(img, (lu, lv), (ru, rv), color, 3)
            if gp is None:
                continue
            cv2.circle(img, gp, 6, color, 2)
            img = put_text_kr(img, g['name'], (gp[0] + 8, gp[1] + 6), color)

        for name, t in targets.items():
            u, v = world_to_pixel(t['x'], t['y'], floor_meta)
            if in_bounds(u, v, floor_meta):
                color = (0, 0, 255) if name == active else config.TARGET_MARKER_COLOR
                cv2.drawMarker(img, (u, v), color, cv2.MARKER_TILTED_CROSS, 16, 2)
                img = put_text_kr(img, name, (u + 8, v - 20), color)

        if pixel_trail:
            cur = pixel_trail[-1]
            cv2.circle(img, cur, config.POSE_DOT_RADIUS_PX, (0, 0, 255), -1)
            with self.lock:
                pose = self.latest_pose
            if pose is not None:
                px, py, yaw = pose
                hx = px + config.POSE_HEADING_LEN_M * math.cos(yaw)
                hy = py + config.POSE_HEADING_LEN_M * math.sin(yaw)
                hu, hv = world_to_pixel(hx, hy, floor_meta)
                cv2.arrowedLine(img, cur, (hu, hv), (0, 0, 255), 2, tipLength=0.5)
        return img

    def _draw_bearing_arrow(self, frame, pose, target_xy, label):
        x, y, yaw = pose
        dx, dy = target_xy[0] - x, target_xy[1] - y
        dist = math.hypot(dx, dy)
        bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi
        h, w = frame.shape[:2]
        ccx, ccy = w // 2, h // 2
        r = min(w, h) * 0.42
        ax = int(max(20, min(w - 20, ccx - r * math.sin(bearing))))
        cv2.arrowedLine(frame, (ccx, ccy), (ax, ccy), config.ARROW_COLOR, 4, tipLength=0.35)
        frame = put_text_kr(frame, f"{label} {dist:.1f}m", (ax - 60, ccy - 34), config.ARROW_COLOR)
        return frame

    def render_cam(self):
        with self.lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else np.zeros((480, 640, 3), np.uint8)
            pose = self.latest_pose
            active = self.active_target
            targets = dict(self.targets)
            path = list(self.path)
            gate_idx = self.current_gate_idx

        if pose is None:
            return frame

        if path and gate_idx < len(path):
            gate = path[gate_idx]
            return self._draw_bearing_arrow(frame, pose, (gate['x'], gate['y']),
                                             f"{gate['name']} (다음 게이트)")

        if active and active in targets:
            t = targets[active]
            return self._draw_bearing_arrow(frame, pose, (t['x'], t['y']), active)

        return frame

    def set_active(self, name):
        with self.lock:
            if name in self.targets:
                self.active_target = name

    def register_target(self, u, v, name):
        with self.lock:
            floor_meta = self.floor_meta
        if floor_meta is None:
            raise RuntimeError("아직 평면도가 없습니다.")
        x, y = pixel_to_world(u, v, floor_meta)
        with self.lock:
            self.targets = mt.register_target(name, x, y)
            if self.active_target is None:
                self.active_target = name
        return x, y

    def delete_target(self, name):
        with self.lock:
            self.targets = mt.remove_target(name)
            if self.active_target == name:
                self.active_target = next(iter(self.targets), None)


app = Flask(__name__)
node: DashboardNode = None

PAGE = """
<!doctype html><html><head><meta charset="utf-8"><title>ZED Dashboard</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;display:flex}
.col{padding:8px} img{max-width:45vw;border:1px solid #444}
#targets{list-style:none;padding:4px;margin:0;position:fixed;right:8px;bottom:8px;width:220px;
  max-height:200px;overflow-y:auto;background:#000;border:1px solid #444;font-size:11px;z-index:10}
#targets li{padding:2px 4px;cursor:pointer;display:flex;justify-content:space-between;align-items:center}
#targets li.active{color:#0f0;font-weight:bold}
#targets li span.del{color:#f55;padding:0 8px;cursor:pointer;font-weight:bold}
#pathinfo{margin-top:12px;color:#0cf}
</style></head><body>
<div class="col"><h3>Floor Plan (클릭해서 목표 등록)</h3>
<img id="mapimg" src="/stream/map"><ul id="targets"></ul>
<div id="pathinfo"></div>
<div id="modectrl" style="margin-top:12px">
  <button onclick="setMode('gate_follow')">항로추종 모드</button>
  <button onclick="startStationKeep()">위치유지 시작(현재 활성목표)</button>
  <button onclick="setMode('idle')">정지</button>
  <div id="modeinfo" style="margin-top:6px;color:#ff0"></div>
</div></div>
<div class="col"><h3>AR Camera View</h3><img src="/stream/cam"></div>
<script>
async function refreshTargets(){
  const r = await fetch('/api/targets'); const d = await r.json();
  const ul = document.getElementById('targets'); ul.innerHTML='';
  d.targets.forEach(n=>{
    const li=document.createElement('li');
    if(n===d.active) li.className='active';
    const label=document.createElement('span'); label.textContent=n;
    label.onclick=()=>fetch('/api/active',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({name:n})}).then(refreshTargets);
    const del=document.createElement('span'); del.textContent='×'; del.className='del';
    del.onclick=(ev)=>{ev.stopPropagation();
      fetch('/api/delete_target',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({name:n})}).then(refreshTargets);};
    li.appendChild(label); li.appendChild(del);
    ul.appendChild(li);
  });
  const pr = await fetch('/api/path'); const pd = await pr.json();
  const pi = document.getElementById('pathinfo');
  pi.textContent = pd.path.length ? `게이트 ${pd.current_index+1}/${pd.path.length} 진행 중 (${pd.path.map(g=>g.name).join(' -> ')})` : '경로 없음';
}
document.getElementById('mapimg').addEventListener('click', async (e)=>{
  const rect = e.target.getBoundingClientRect();
  const u = Math.round((e.clientX-rect.left)*(e.target.naturalWidth/rect.width));
  const v = Math.round((e.clientY-rect.top)*(e.target.naturalHeight/rect.height));
  const name = prompt('목표 이름:'); if(!name) return;
  await fetch('/api/register_target',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({u,v,name})});
  refreshTargets();
});
async function refreshMode(){
  const mr = await fetch('/api/mode'); const md = await mr.json();
  let txt = `모드: ${md.mode}` + (md.station_target ? ` (목표: ${md.station_target})` : '');
  if (md.mode === 'station_keep') {
    const sr = await fetch('/api/station_progress'); const sd = await sr.json();
    txt += ` | 유지 ${sd.held_seconds}s / ${5.0}s` + (sd.success ? ' ✅성공' : '');
  }
  document.getElementById('modeinfo').textContent = txt;
}
function setMode(mode, target){
  fetch('/api/set_mode',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({mode:mode, station_target: target||null})}).then(refreshMode);
}
async function startStationKeep(){
  const r = await fetch('/api/targets'); const d = await r.json();
  if(!d.active){ alert('먼저 목표를 하나 클릭해서 활성화하세요'); return; }
  setMode('station_keep', d.active);
}
refreshTargets(); setInterval(refreshTargets, 3000);
refreshMode(); setInterval(refreshMode, 1000);
</script></body></html>
"""


def mjpeg_generator(render_fn):
    while True:
        ok, buf = cv2.imencode('.jpg', render_fn())
        if ok:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')
        time.sleep(1.0 / config.STREAM_FPS)


@app.route('/')
def index():
    return PAGE


@app.route('/stream/map')
def stream_map():
    return Response(mjpeg_generator(node.render_map), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/cam')
def stream_cam():
    return Response(mjpeg_generator(node.render_cam), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/targets')
def api_targets():
    node._reload_targets_if_changed()
    return jsonify({"targets": list(node.targets.keys()), "active": node.active_target})


@app.route('/api/path')
def api_path():
    node._reload_path()
    return jsonify({"path": node.path, "current_index": node.current_gate_idx})


@app.route('/api/mode')
def api_mode():
    return jsonify(ms.load_mode())


@app.route('/api/set_mode', methods=['POST'])
def api_set_mode():
    d = request.json
    ms.save_mode(d.get('mode', 'idle'), d.get('station_target'))
    return jsonify({"ok": True})


@app.route('/api/station_progress')
def api_station_progress():
    return jsonify(ms.load_station_progress())


@app.route('/api/active', methods=['POST'])
def api_active():
    node.set_active(request.json['name'])
    return jsonify({"ok": True})


@app.route('/api/register_target', methods=['POST'])
def api_register():
    d = request.json
    try:
        x, y = node.register_target(int(d['u']), int(d['v']), d['name'])
    except RuntimeError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, "x": x, "y": y})


@app.route('/api/delete_target', methods=['POST'])
def api_delete():
    node.delete_target(request.json['name'])
    return jsonify({"ok": True})


def main():
    global node
    rclpy.init()
    node = DashboardNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app.run(host=config.HTTP_HOST, port=config.HTTP_PORT, threaded=True)


if __name__ == '__main__':
    main()

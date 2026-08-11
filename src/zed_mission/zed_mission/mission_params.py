"""
대회 당일 공지되는 미션 파라미터(미션3 도킹 표식 색/모양, 미션4 탐색 목표색)만
입력받는 독립 노드. 대시보드(카메라스트림+매핑)와 완전히 분리해서, 대시보드가
재시작되거나 디버깅 중이어도 이 노드/입력값은 영향받지 않게 함.

입력은 자체 웹페이지(HTTP_HOST:PARAMS_HTTP_PORT, 대시보드와 다른 포트)에서 받고,
mission_params 토픽(transient_local)으로 발행한다. helm_node / mission_manager_node가
이 토픽을 구독해서 씀. 파일(docking_target.json / search_color.json)에도 같이 저장해서
이 노드 자체가 재시작돼도 마지막 값을 복원할 수 있게 함.
"""

import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String as RosString
from flask import Flask, request, jsonify

from zed_common import config
from zed_common import mission_state as ms

MODE_QOS = QoSProfile(depth=1)
MODE_QOS.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
MODE_QOS.reliability = QoSReliabilityPolicy.RELIABLE


class MissionParamsNode(Node):
    def __init__(self):
        super().__init__('zed_mission_params_node')
        self.params_pub = self.create_publisher(RosString, 'mission_params', MODE_QOS)

        dt = ms.load_docking_target()
        self.params = {
            "docking_color": dt.get("color"),
            "docking_shape": dt.get("shape"),
            "search_color": ms.load_search_color().get("color", "red"),
        }
        self.publish_params()
        self.get_logger().info(
            f"미션 파라미터 노드 시작 - http://<jetson-ip>:{config.PARAMS_HTTP_PORT} 에서 입력")

    def publish_params(self):
        msg = RosString()
        msg.data = json.dumps(self.params)
        self.params_pub.publish(msg)

    def set_docking_target(self, color, shape):
        self.params["docking_color"] = color
        self.params["docking_shape"] = shape
        ms.save_docking_target(color, shape)
        self.publish_params()
        self.get_logger().info(f"미션3 도킹 표식 지정: {color}/{shape}")

    def set_search_color(self, color):
        self.params["search_color"] = color
        ms.save_search_color(color)
        self.publish_params()
        self.get_logger().info(f"미션4 탐색 목표색 지정: {color}")


app = Flask(__name__)
node: MissionParamsNode = None

PAGE = """
<!doctype html><html><head><meta charset="utf-8"><title>미션 파라미터</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;padding:24px;max-width:480px}
h2{color:#0cf} .row{margin:16px 0;padding:12px;background:#1a1a1a;border-radius:8px}
select,button{font-size:16px;padding:8px;margin:4px 4px 4px 0}
button{background:#0cf;border:none;border-radius:4px;cursor:pointer;font-weight:bold}
#status{margin-top:16px;color:#0f0;font-size:14px;white-space:pre-line}
</style></head><body>
<h2>미션 파라미터 (대회당일 공지값 입력)</h2>
<div class="row">
  <b>미션3 도킹 표식</b><br>
  <select id="dockColor">
    <option value="red">빨강</option><option value="green">초록</option>
    <option value="blue">파랑</option><option value="orange">주황</option>
    <option value="yellow">노랑</option>
  </select>
  <select id="dockShape">
    <option value="triangle">삼각형</option><option value="circle">원형</option>
    <option value="square">네모</option>
  </select>
  <button onclick="setDock()">지정</button>
</div>
<div class="row">
  <b>미션4 탐색 목표색</b><br>
  <select id="searchColor">
    <option value="red">빨강(시계방향)</option>
    <option value="green">초록(시계방향)</option>
    <option value="white">흰색(반시계방향)</option>
  </select>
  <button onclick="setSearch()">지정</button>
</div>
<div id="status">불러오는 중...</div>
<script>
async function refresh(){
  const r = await fetch('/api/params'); const d = await r.json();
  document.getElementById('status').textContent =
    `현재 값\\n미션3: ${d.docking_color || '미지정'} / ${d.docking_shape || '미지정'}\\n미션4: ${d.search_color}`;
  if (d.docking_color) document.getElementById('dockColor').value = d.docking_color;
  if (d.docking_shape) document.getElementById('dockShape').value = d.docking_shape;
  if (d.search_color) document.getElementById('searchColor').value = d.search_color;
}
async function setDock(){
  const color = document.getElementById('dockColor').value;
  const shape = document.getElementById('dockShape').value;
  await fetch('/api/set_docking_target', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({color, shape})});
  refresh();
}
async function setSearch(){
  const color = document.getElementById('searchColor').value;
  await fetch('/api/set_search_color', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({color})});
  refresh();
}
refresh(); setInterval(refresh, 3000);
</script></body></html>
"""


@app.route('/')
def index():
    return PAGE


@app.route('/api/params')
def api_params():
    return jsonify(node.params)


@app.route('/api/set_docking_target', methods=['POST'])
def api_set_docking_target():
    d = request.json
    node.set_docking_target(d.get('color'), d.get('shape'))
    return jsonify({"ok": True})


@app.route('/api/set_search_color', methods=['POST'])
def api_set_search_color():
    d = request.json
    node.set_search_color(d.get('color', 'red'))
    return jsonify({"ok": True})


def main():
    global node
    rclpy.init()
    node = MissionParamsNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app.run(host=config.HTTP_HOST, port=config.PARAMS_HTTP_PORT, threaded=True)


if __name__ == '__main__':
    main()

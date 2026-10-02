#!/usr/bin/env python3
"""Phone web control and fail-safe AUTO/STOP/MANUAL command multiplexer."""

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32


WEB_PAGE = r'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>T870 원격 제어</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:18px;text-align:center}
.card{max-width:520px;margin:auto;background:#222;padding:18px;border-radius:16px}
h1{font-size:24px}.state{font-size:30px;font-weight:bold;margin:12px}.stop{color:#ff5252}.auto{color:#55c2ff}.gps{color:#ffca55}.manual{color:#69f08c}
button{font-size:20px;padding:15px;margin:6px;border:0;border-radius:10px;touch-action:none}
.mode button{min-width:120px}.danger{background:#d32f2f;color:white}
.controls{display:flex;gap:20px;justify-content:center;align-items:center;margin:18px 0}.throttleBox{min-width:120px}.steerBox{flex:1;min-width:200px}
.throttle{writing-mode:vertical-lr;direction:rtl;width:48px;height:280px;touch-action:none}.steer{width:95%;height:48px;touch-action:none}
.zero{font-size:15px;padding:10px;background:#555;color:white}.small{color:#bbb;font-size:14px}
.gpsPanel{margin:18px 0;padding:14px;background:#181818;border-radius:12px}.gpsState{font-size:18px;font-weight:bold;margin:8px}.recording{color:#ff5252}.idle{color:#aaa}
.gpsPanel button{min-width:150px}.startRecord{background:#2e7d32;color:white}
</style></head><body><div class="card">
<h1>T870 원격 제어</h1>
<div id="state" class="state stop">STOP</div>
<div class="mode"><button onclick="setMode('AUTO')">차선 AUTO</button><button onclick="setMode('GPS')">GPS PATH</button><button class="danger" onclick="setMode('STOP')">STOP</button><button onclick="setMode('MANUAL')">MANUAL</button></div>
<div class="controls">
  <div class="throttleBox"><div>전진 ▲</div><input id="speed" class="throttle" type="range" min="-1.20" max="1.20" step="0.01" value="0"><div>후진 ▼</div><b id="speedText">0.00 m/s</b><br><button class="zero" onclick="zeroSpeed()">속도 0</button></div>
  <div class="steerBox"><div>조향</div><b id="steerText">0.00 rad/s</b><br><input id="steer" class="steer" type="range" min="-0.25" max="0.25" step="0.01" value="0"><br><button class="zero" onclick="zeroSteer()">조향 중앙</button></div>
</div>
<div class="gpsPanel">
  <h2>GPS 경로 기록</h2>
  <div id="gpsState" class="gpsState idle">중지 · 0점</div>
  <button class="startRecord" onclick="setGpsRecording(true)">GPS 기록 시작</button>
  <button class="danger" onclick="setGpsRecording(false)">GPS 기록 중지</button>
  <p class="small">기록 중에는 이동 거리 1 m마다 좌표를 저장합니다.</p>
</div>
<p class="small">MANUAL 모드에서는 슬라이더 값으로 바로 주행합니다.<br>Safari가 닫히거나 통신이 끊기면 자동으로 정지합니다.<br>+조향은 우회전, -조향은 좌회전입니다.</p>
</div><script>
let pin=localStorage.t870pin||prompt('제어 PIN을 입력하세요')||''; localStorage.t870pin=pin;
let currentMode='STOP';
const speed=document.getElementById('speed'), steer=document.getElementById('steer');
const gpsState=document.getElementById('gpsState');
function labels(){speedText.textContent=Number(speed.value).toFixed(2)+' m/s';steerText.textContent=Number(steer.value).toFixed(2)+' rad/s'}
function sliderChanged(){labels();if(currentMode==='MANUAL')sendManual(true)}
function zeroSpeed(){speed.value=0;sliderChanged()}
function zeroSteer(){steer.value=0;sliderChanged()}
speed.oninput=sliderChanged;steer.oninput=sliderChanged;labels();
async function post(path,data){data.pin=pin;try{return await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})}catch(e){return null}}
function heartbeat(){post('/api/heartbeat',{})}
async function setMode(mode){if((mode==='AUTO'||mode==='GPS')&&!confirm(mode==='GPS'?'저장된 GPS 경로 추종을 시작합니까?':'차선 자율주행 AUTO 모드로 전환합니까?'))return;heartbeat();if(mode==='MANUAL'){speed.value=0;steer.value=0;labels()}else{sendManual(false)}let r=await post('/api/mode',{mode});if(r&&r.ok){currentMode=mode;heartbeat();if(mode==='MANUAL')sendManual(true)}refresh()}
function sendManual(active){post('/api/manual',{active,linear_x:active?Number(speed.value):0,angular_z:active?Number(steer.value):0})}
async function setGpsRecording(enabled){if(enabled&&!confirm('GPS 경로 기록을 시작합니까?'))return;let r=await post('/api/gps_recording',{enabled});if(r&&r.ok)refresh()}
async function refresh(){try{let r=await fetch('/api/state?pin='+encodeURIComponent(pin),{cache:'no-store'});let s=await r.json();currentMode=s.mode;state.textContent=s.mode+(s.traffic_stop?' (신호 정지)':(s.command_fresh?'':' (명령 대기)'));state.className='state '+(s.traffic_stop?'stop':s.mode.toLowerCase());gpsState.textContent=(s.gps_recording?'기록 중':'중지')+' · '+s.gps_point_count+'점';gpsState.className='gpsState '+(s.gps_recording?'recording':'idle')}catch(e){currentMode='STOP';state.textContent='연결 끊김';state.className='state stop'}}
setInterval(()=>{if(currentMode==='MANUAL'&&!document.hidden)sendManual(true)},100);
// Keep the operator lease alive even when Safari reports the page as hidden.
// Mobile Safari can briefly toggle document.hidden while showing dialogs or
// changing browser chrome, which must not cause an unintended vehicle stop.
setInterval(heartbeat,300);
addEventListener('pageshow',heartbeat);
addEventListener('focus',heartbeat);
addEventListener('orientationchange',()=>{heartbeat();setTimeout(heartbeat,250);setTimeout(heartbeat,1000)});
document.addEventListener('visibilitychange',()=>{if(!document.hidden)heartbeat()});
setInterval(refresh,500);refresh();
</script></body></html>'''


def zero_twist():
    return Twist()


class RemoteModeControlNode(Node):
    MODES = ('AUTO', 'GPS', 'STOP', 'MANUAL')

    def __init__(self):
        super().__init__('remote_mode_control_node')
        defaults = {
            'listen_address': '0.0.0.0',
            'web_port': 8080,
            'control_pin': '8700',
            'output_rate_hz': 20.0,
            'auto_timeout_sec': 0.5,
            'manual_timeout_sec': 0.4,
            'transition_stop_sec': 0.5,
            'require_operator_heartbeat': False,
            'operator_timeout_sec': 1.0,
            'max_manual_speed_mps': 1.20,
            'max_manual_steer_radps': 0.40,
            'auto_command_topic': '/cmd_vel/auto',
            'gps_command_topic': '/cmd_vel/gps',
            'startup_mode': 'STOP',
            'traffic_stop_in_gps_mode': False,
            'rc_arm_enabled': True,
            'rc_arm_mode': 'GPS',
            'rc_timeout_sec': 1.5,
            # 이 노드가 /gps/recording_enabled 의 주인이라 매 틱 제 상태를
            # 덮어쓴다. 런치로 녹화를 켜려면 여기도 같이 켜야 한다.
            # 안 그러면 레코더가 켜졌다가 몇 ms 만에 꺼진다.
            'start_gps_recording': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.listen_address = str(self.get_parameter('listen_address').value)
        self.web_port = int(self.get_parameter('web_port').value)
        self.control_pin = str(self.get_parameter('control_pin').value)
        rate = float(self.get_parameter('output_rate_hz').value)
        self.auto_timeout = float(self.get_parameter('auto_timeout_sec').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout_sec').value)
        self.transition_stop = float(self.get_parameter('transition_stop_sec').value)
        self.require_operator_heartbeat = bool(
            self.get_parameter('require_operator_heartbeat').value)
        self.operator_timeout = float(
            self.get_parameter('operator_timeout_sec').value)
        self.max_manual_speed = float(self.get_parameter('max_manual_speed_mps').value)
        self.max_manual_steer = float(self.get_parameter('max_manual_steer_radps').value)
        self.auto_command_topic = str(
            self.get_parameter('auto_command_topic').value)
        self.gps_command_topic = str(
            self.get_parameter('gps_command_topic').value)
        self.traffic_stop_in_gps = bool(
            self.get_parameter('traffic_stop_in_gps_mode').value)
        self.rc_arm_enabled = bool(self.get_parameter('rc_arm_enabled').value)
        self.rc_arm_mode = str(self.get_parameter('rc_arm_mode').value).upper()
        self.rc_timeout = float(self.get_parameter('rc_timeout_sec').value)
        if self.rc_arm_mode not in self.MODES:
            raise ValueError(
                f'rc_arm_mode must be one of {self.MODES}, got {self.rc_arm_mode}')
        if self.rc_timeout <= 0.0:
            raise ValueError('rc_timeout_sec must be positive')
        if rate <= 0.0 or min(self.auto_timeout, self.manual_timeout,
                              self.transition_stop) < 0.0:
            raise ValueError('rates and timeouts must not be negative')

        startup_mode = str(self.get_parameter('startup_mode').value).upper()
        if startup_mode not in self.MODES:
            raise ValueError(
                f'startup_mode must be one of {self.MODES}, got {startup_mode}')
        self.lock = threading.Lock()
        self.mode = startup_mode
        self.auto_command = zero_twist()
        self.gps_command = zero_twist()
        self.manual_command = zero_twist()
        self.last_auto_time = 0.0
        self.last_gps_time = 0.0
        self.last_manual_time = 0.0
        self.transition_until = 0.0
        self.last_operator_time = 0.0
        self.gps_recording_requested = bool(
            self.get_parameter('start_gps_recording').value)
        self.gps_recording_active = False
        self.gps_point_count = 0
        self.traffic_stop_active = False
        self.rc_auto = False
        self.last_rc_time = 0.0
        self.rc_stop_reason = None

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mode = self.create_publisher(String, '/t870/control_mode', 10)
        gps_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_gps_recording = self.create_publisher(
            Bool, '/gps/recording_enabled', gps_qos)
        self.create_subscription(
            Bool, '/gps/recording_active',
            self.gps_recording_active_callback, gps_qos)
        self.create_subscription(
            UInt32, '/gps/recorded_point_count',
            self.gps_point_count_callback, 10)
        self.create_subscription(
            Twist, self.auto_command_topic, self.auto_callback, 10)
        self.create_subscription(
            Twist, self.gps_command_topic, self.gps_callback, 10)
        self.create_subscription(
            Bool, '/vision/traffic_stop', self.traffic_stop_callback, 10)
        self.create_subscription(
            Bool, '/t870/remote_auto', self.remote_auto_callback, 10)
        self.timer = self.create_timer(1.0 / rate, self.control_tick)

        handler = self.make_handler()
        self.httpd = ThreadingHTTPServer(
            (self.listen_address, self.web_port), handler)
        self.http_thread = threading.Thread(
            target=self.httpd.serve_forever, name='t870-web-control', daemon=True)
        self.http_thread.start()
        self.get_logger().warning(
            f'T870 remote control ready on port {self.web_port}; startup mode {self.mode}; '
            f'AUTO source {self.auto_command_topic}')

    @staticmethod
    def valid_twist(msg):
        return math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)

    @staticmethod
    def copy_twist(msg):
        copied = Twist()
        copied.linear.x = float(msg.linear.x)
        copied.angular.z = float(msg.angular.z)
        return copied

    def auto_callback(self, msg):
        if not self.valid_twist(msg):
            self.get_logger().warning('Ignoring non-finite /cmd_vel/auto')
            return
        with self.lock:
            self.auto_command = self.copy_twist(msg)
            self.last_auto_time = time.monotonic()

    def gps_callback(self, msg):
        if not self.valid_twist(msg):
            self.get_logger().warning('Ignoring non-finite GPS /cmd_vel')
            return
        with self.lock:
            self.gps_command = self.copy_twist(msg)
            self.last_gps_time = time.monotonic()

    def gps_recording_active_callback(self, msg):
        with self.lock:
            self.gps_recording_active = bool(msg.data)

    def gps_point_count_callback(self, msg):
        with self.lock:
            self.gps_point_count = int(msg.data)

    def traffic_stop_callback(self, msg):
        with self.lock:
            self.traffic_stop_active = bool(msg.data)

    def set_gps_recording(self, enabled):
        with self.lock:
            self.gps_recording_requested = bool(enabled)
        self.get_logger().warning(
            'GPS path recording START requested' if enabled
            else 'GPS path recording STOP requested')
        return True

    def remote_auto_callback(self, msg):
        # The physical transmitter is the arming switch: MANUAL->AUTO selects
        # rc_arm_mode, AUTO->MANUAL drops straight back to STOP. The edge is
        # what arms, so a MANUAL switch can never leave an autonomous mode
        # latched from a previous run.
        if not self.rc_arm_enabled:
            return
        value = bool(msg.data)
        with self.lock:
            self.last_rc_time = time.monotonic()
            changed = value != self.rc_auto
            self.rc_auto = value
        if not changed:
            return
        self.set_mode(self.rc_arm_mode if value else 'STOP')

    def set_mode(self, mode):
        if mode not in self.MODES:
            return False
        with self.lock:
            if mode != self.mode:
                previous = self.mode
                self.mode = mode
                self.manual_command = zero_twist()
                self.last_manual_time = 0.0
                self.transition_until = time.monotonic() + self.transition_stop
                self.get_logger().warning(
                    f'Control mode {previous} -> {mode}; transition STOP active')
        return True

    def set_manual(self, linear_x, angular_z, active):
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            return False
        command = zero_twist()
        if active:
            command.linear.x = max(
                -self.max_manual_speed, min(self.max_manual_speed, linear_x))
            command.angular.z = max(
                -self.max_manual_steer, min(self.max_manual_steer, angular_z))
        with self.lock:
            self.manual_command = command
            self.last_manual_time = time.monotonic()
        return True

    def selected_command(self):
        now = time.monotonic()
        with self.lock:
            mode = self.mode
            if now < self.transition_until:
                return zero_twist(), mode, False
            if (self.require_operator_heartbeat and mode != 'STOP'
                    and now - self.last_operator_time > self.operator_timeout):
                age = now - self.last_operator_time
                self.mode = 'STOP'
                self.manual_command = zero_twist()
                self.get_logger().warning(
                    f'Operator heartbeat expired ({age:.2f}s); forcing STOP')
                return zero_twist(), 'STOP', False
            # The transmitter outranks the web UI. While it reports MANUAL the
            # firmware drives from the RC sticks anyway, and losing the Arduino
            # status line means the switch position is unknown.
            if self.rc_arm_enabled and mode != 'STOP':
                rc_stale = now - self.last_rc_time > self.rc_timeout
                if rc_stale or not self.rc_auto:
                    reason = ('RC telemetry stale' if rc_stale
                              else 'transmitter in MANUAL')
                    self.mode = 'STOP'
                    self.manual_command = zero_twist()
                    if reason != self.rc_stop_reason:
                        self.rc_stop_reason = reason
                        self.get_logger().warning(f'{reason}; forcing STOP')
                    return zero_twist(), 'STOP', False
                self.rc_stop_reason = None
            # Camera traffic control applies to autonomous sources. MANUAL is
            # deliberately left available so an operator can recover/move the
            # vehicle after confirming the surroundings are safe.
            # Camera stop-line detection belongs to lane AUTO. Applying it to
            # GPS mode caused yellow pavement/objects to latch an unrelated
            # stop and permanently suppress a valid GPS command.
            traffic_applies = mode == 'AUTO' or (
                mode == 'GPS' and self.traffic_stop_in_gps)
            if traffic_applies and self.traffic_stop_active:
                return zero_twist(), mode, True
            if mode == 'AUTO':
                fresh = now - self.last_auto_time <= self.auto_timeout
                return (self.copy_twist(self.auto_command) if fresh else zero_twist(),
                        mode, fresh)
            if mode == 'GPS':
                fresh = now - self.last_gps_time <= self.auto_timeout
                return (self.copy_twist(self.gps_command) if fresh else zero_twist(),
                        mode, fresh)
            if mode == 'MANUAL':
                fresh = now - self.last_manual_time <= self.manual_timeout
                return (self.copy_twist(self.manual_command) if fresh else zero_twist(),
                        mode, fresh)
            return zero_twist(), mode, False

    def control_tick(self):
        command, mode, _ = self.selected_command()
        self.pub_cmd.publish(command)
        self.pub_mode.publish(String(data=mode))
        with self.lock:
            gps_recording = self.gps_recording_requested
        self.pub_gps_recording.publish(Bool(data=gps_recording))

    def state(self):
        _, mode, fresh = self.selected_command()
        with self.lock:
            gps_recording = self.gps_recording_active
            gps_point_count = self.gps_point_count
            traffic_stop = self.traffic_stop_active
        return {
            'mode': mode,
            'command_fresh': fresh,
            'gps_recording': gps_recording,
            'gps_point_count': gps_point_count,
            'traffic_stop': traffic_stop,
        }

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format, *_args):
                return

            def reply(self, code, data, content_type='application/json'):
                body = (data if isinstance(data, bytes)
                        else json.dumps(data).encode('utf-8'))
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                request = urlsplit(self.path)
                if request.path == '/':
                    self.reply(200, WEB_PAGE.encode('utf-8'),
                               'text/html; charset=utf-8')
                elif request.path == '/api/state':
                    supplied_pin = parse_qs(request.query).get('pin', [''])[0]
                    if supplied_pin != node.control_pin:
                        self.reply(403, {'error': 'bad PIN'})
                        return
                    # The authenticated state poll is a second operator-presence
                    # channel. Mobile Safari may suspend interval POST requests
                    # around orientation changes while normal fetches continue.
                    with node.lock:
                        node.last_operator_time = time.monotonic()
                    self.reply(200, node.state())
                else:
                    self.reply(404, {'error': 'not found'})

            def do_POST(self):
                try:
                    length = min(int(self.headers.get('Content-Length', '0')), 4096)
                    data = json.loads(self.rfile.read(length) or b'{}')
                except (ValueError, json.JSONDecodeError):
                    self.reply(400, {'error': 'invalid JSON'})
                    return
                if str(data.get('pin', '')) != node.control_pin:
                    self.reply(403, {'error': 'bad PIN'})
                    return
                with node.lock:
                    node.last_operator_time = time.monotonic()
                if self.path == '/api/mode':
                    ok = node.set_mode(str(data.get('mode', '')).upper())
                elif self.path == '/api/manual':
                    try:
                        ok = node.set_manual(float(data.get('linear_x', 0.0)),
                                             float(data.get('angular_z', 0.0)),
                                             bool(data.get('active', False)))
                    except (TypeError, ValueError):
                        ok = False
                elif self.path == '/api/heartbeat':
                    ok = True
                elif self.path == '/api/gps_recording':
                    ok = node.set_gps_recording(
                        bool(data.get('enabled', False)))
                else:
                    self.reply(404, {'error': 'not found'})
                    return
                self.reply(200 if ok else 400, {'ok': ok})

        return Handler

    def stop(self):
        if rclpy.ok():
            for _ in range(3):
                self.pub_cmd.publish(zero_twist())
                self.pub_gps_recording.publish(Bool(data=False))
        self.httpd.shutdown()
        self.httpd.server_close()
        self.http_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteModeControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

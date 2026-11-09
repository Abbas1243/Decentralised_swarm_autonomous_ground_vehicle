#!/usr/bin/env python3
"""Localhost PID tuner for robot_tune.ino. Python 3 standard library only.

    python3 pid_tuner.py --robot-ip 10.35.59.xxx        # IP is printed in the robot's serial monitor
    then open http://127.0.0.1:8000

The PID loops run on the robot. This tool sends gains/commands, logs telemetry, plots it, and keeps a
trials table where YOU enter the independently measured result (protractor/tape), because the robot's
own "error" is measured against its own estimate and can be confidently wrong.
"""
import argparse, csv, json, math, os, socket, struct, threading, time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

TELEM = struct.Struct('<BBHI12fhhB')   # 61 bytes, magic 0xD1
ACK = struct.Struct('<BBfff')          # 14 bytes, magic 0xD2
CMD = struct.Struct('<BBffff')         # 18 bytes, magic 0xC2
(T_STOP, T_TURN, T_DRIVE, T_WHEEL, T_GAINS, T_GEOM, T_LIMITS,
 T_ZERO, T_HEAD, T_HELLO, T_CALIB, T_REQ, T_SIGN, T_TURNG, T_ACAL, T_SAVE, T_FLIPFB, T_RSTCFG, T_FLIPR, T_SWAPLR) = range(20)
HERE = os.path.dirname(os.path.abspath(__file__))


def parse_telem(data):
    u = TELEM.unpack(data)
    f = u[4:16]
    return dict(st=u[1], seq=u[2], t=u[3] / 1000.0, x=f[0], y=f[1], th=f[2], tg=f[3], te=f[4],
                tgt=f[5], err=f[6], aux=f[7], vl=f[8], vr=f[9], vlt=f[10], vrt=f[11],
                pl=u[16], pr=u[17], fl=u[18])


def mean_std(v):
    v = [q for q in v if q is not None]
    if not v:
        return None
    m = sum(v) / len(v)
    sd = math.sqrt(sum((q - m) ** 2 for q in v) / (len(v) - 1)) if len(v) > 1 else 0.0
    return dict(n=len(v), mean=m, std=sd, maxabs=max(abs(q) for q in v))


class Tuner:
    def __init__(self, a):
        self.lock = threading.Lock()
        self.robot_ip = a.robot_ip
        self.robot_port = 5005 + a.robot_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', a.telem_port))
        self.sock.settimeout(0.5)
        self.samples = deque(maxlen=5000)
        self.n = 0
        self.latest = None
        self.acks = {}
        self.rx_times = deque(maxlen=400)
        self.last_rx = 0.0
        self.last_seq = None
        self.lost = 0
        self.requested = False
        self.move = None
        self.trials = []
        os.makedirs(os.path.join(HERE, 'logs'), exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.tel_path = os.path.join(HERE, 'logs', f'tune_{stamp}.csv')
        self.tri_path = os.path.join(HERE, 'logs', f'trials_{stamp}.csv')
        self.cols = ['i', 'wall', 't', 'st', 'seq', 'x', 'y', 'th', 'tg', 'te', 'tgt', 'err', 'aux',
                     'vl', 'vr', 'vlt', 'vrt', 'pl', 'pr', 'fl']
        self.fh = open(self.tel_path, 'w', newline='')
        self.csvw = csv.writer(self.fh)
        self.csvw.writerow(self.cols)

    # ---- sending ----
    def send(self, t, a=0.0, b=0.0, c=0.0, d=0.0):
        if not self.robot_ip:
            return False
        try:
            self.sock.sendto(CMD.pack(0xC2, t, a, b, c, d), (self.robot_ip, self.robot_port))
            return True
        except OSError:
            return False

    def api_cmd(self, j):
        t = int(j['type'])
        a, b, c, d = (float(j.get(k, 0)) for k in 'abcd')
        ok = self.send(t, a, b, c, d)
        if ok and t in (T_TURN, T_DRIVE, T_TURNG):
            with self.lock:
                self.move = dict(kind='drive' if t == T_DRIVE else 'turn', cmd=a, max=b, seen=False,
                                 samples=[], start=self.latest, gains=self.gain_str())
        return ok

    def gain_str(self):
        parts = []
        for k, nm in ((0, 'spd'), (1, 'turn'), (2, 'dist'), (3, 'hold'), (4, 'gyro')):
            if k in self.acks:
                a = self.acks[k]
                parts.append(f'{nm} {a[0]:g}/{a[1]:g}/{a[2]:g}')
        return '  '.join(parts)

    # ---- background threads ----
    def hello_loop(self):
        while True:
            now = time.time()
            live = self.last_rx and now - self.last_rx < 1.5
            self.send(T_HELLO)
            if live and not self.requested:
                self.send(T_REQ)
                self.requested = True
            if not live:
                self.requested = False
            time.sleep(1.0)

    def rx_loop(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(data) == TELEM.size and data[0] == 0xD1:
                self.on_sample(parse_telem(data))
            elif len(data) == ACK.size and data[0] == 0xD2:
                _, kind, a, b, c = ACK.unpack(data)
                with self.lock:
                    self.acks[kind] = (a, b, c)

    def on_sample(self, s):
        with self.lock:
            now = time.time()
            if self.last_seq is not None:
                gap = (s['seq'] - self.last_seq) & 0xFFFF
                if 1 < gap < 1000:
                    self.lost += gap - 1
            self.last_seq = s['seq']
            self.n += 1
            s['i'] = self.n
            self.samples.append(s)
            self.latest = s
            self.last_rx = now
            self.rx_times.append(now)
            self.csvw.writerow([s['i'], f'{now:.3f}'] + [s[k] for k in self.cols[2:]])
            if self.n % 50 == 0:
                self.fh.flush()
            self._track_move(s)

    # ---- trials ----
    def _track_move(self, s):
        m = self.move
        if not m:
            return
        if s['st'] in (2, 3):
            m['seen'] = True
            m['samples'].append(s)
        elif m['seen'] and s['st'] == 0:
            self._finalize(m, s)
            self.move = None

    def _finalize(self, m, s):
        kind, cmd, st = m['kind'], m['cmd'], m['start'] or m['samples'][0]
        fl = s['fl']
        result = 'fault' if fl & 32 else ('ok' if fl & 8 else ('timeout' if fl & 4 else 'stopped'))
        k = 180.0 / math.pi if kind == 'turn' else 1000.0     # deg or mm
        if kind == 'drive':
            cmd = cmd * 1000.0                                  # the command is in metres, everything below is in mm
        est = (s['tgt'] - s['err']) * k
        prog = [(q['tgt'] - q['err']) * k for q in m['samples']]
        sign = 1 if cmd >= 0 else -1
        est_err = (est - cmd) * sign if cmd else None          # + overshoot, - undershoot
        over = max(0.0, max((p * sign for p in prog), default=0.0) - abs(cmd))
        ge = ((s['tg'] - st['tg']) - (s['te'] - st['te'])) * 180 / math.pi if kind == 'turn' else None
        hold = math.degrees(s['aux']) if kind == 'drive' else None
        dtg = math.degrees(s['tg'] - st['tg']) if kind == 'turn' else None
        dte = math.degrees(s['te'] - st['te']) if kind == 'turn' else None
        gyro_ok = bool((fl & 1) and (fl & 16))
        self.trials.append(dict(
            id=len(self.trials) + 1, kind=kind, unit='deg' if kind == 'turn' else 'mm', cmd=cmd, max=m['max'],
            est=est, est_err=est_err, gyro_minus_enc=ge, dtg=dtg, dte=dte, gyro_ok=gyro_ok, overshoot=over,
            settle=s['t'] - m['samples'][0]['t'], hold=hold, result=result,
            measured=None, true_err=None, gains=m['gains']))
        self.save_trials()

    def api_measured(self, j):
        tid = int(j['id'])
        val = j.get('value')
        with self.lock:
            for t in self.trials:
                if t['id'] == tid:
                    try:
                        t['measured'] = float(val)
                        sign = 1 if t['cmd'] >= 0 else -1
                        t['true_err'] = (t['measured'] - t['cmd']) * sign
                    except (TypeError, ValueError):
                        t['measured'] = t['true_err'] = None
            self.save_trials()

    def save_trials(self):
        keys = ['id', 'kind', 'unit', 'cmd', 'max', 'est', 'est_err', 'gyro_minus_enc', 'dtg', 'dte', 'gyro_ok', 'overshoot',
                'settle', 'hold', 'result', 'measured', 'true_err', 'gains']
        with open(self.tri_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(keys)
            for t in self.trials:
                w.writerow([t[k] for k in keys])

    def stats(self):
        out = {}
        for kind in ('turn', 'drive'):
            rows = [t for t in self.trials if t['kind'] == kind and t['result'] == 'ok']
            out[kind] = dict(est=mean_std([t['est_err'] for t in rows]),
                             true=mean_std([t['true_err'] for t in rows]))
        return out

    # ---- state for the UI ----
    def state(self, since):
        with self.lock:
            now = time.time()
            out = [s for s in self.samples if s['i'] > since][-400:]
            rate = sum(1 for q in self.rx_times if now - q < 1.0)
            return dict(n=self.n, samples=out, robot_ip=self.robot_ip,
                        conn=dict(age=(now - self.last_rx) if self.last_rx else None, rate=rate, lost=self.lost),
                        acks={str(k): v for k, v in self.acks.items()},
                        trials=self.trials, stats=self.stats())


TUNER = None


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype='application/json', code=200, extra=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body)
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == '/':
            self._send(PAGE, 'text/html; charset=utf-8')
        elif u.path == '/api/state':
            since = int(parse_qs(u.query).get('since', ['0'])[0])
            self._send(TUNER.state(since))
        elif u.path in ('/download/telemetry.csv', '/download/trials.csv'):
            p = TUNER.tel_path if 'telemetry' in u.path else TUNER.tri_path
            if 'telemetry' in u.path:
                TUNER.fh.flush()
            try:
                with open(p, 'rb') as f:
                    self._send(f.read(), 'text/csv', extra={'Content-Disposition': f'attachment; filename={os.path.basename(p)}'})
            except OSError:
                self._send({'error': 'no file yet'}, code=404)
        else:
            self._send({'error': 'not found'}, code=404)

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            j = json.loads(self.rfile.read(n) or b'{}')
        except ValueError:
            return self._send({'error': 'bad json'}, code=400)
        if self.path == '/api/cmd':
            self._send({'ok': TUNER.api_cmd(j)})
        elif self.path == '/api/measured':
            TUNER.api_measured(j)
            self._send({'ok': True})
        elif self.path == '/api/ip':
            TUNER.robot_ip = (j.get('ip') or '').strip() or None
            TUNER.requested = False
            self._send({'ok': True})
        else:
            self._send({'error': 'not found'}, code=404)


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Robot PID tuner</title>
<style>
body{font:14px system-ui,sans-serif;margin:0;background:#f4f5f7;color:#1c1e21}
header{background:#1f2937;color:#fff;padding:8px 14px;display:flex;gap:16px;align-items:center;flex-wrap:wrap}
header input{width:130px} main{display:grid;grid-template-columns:440px 1fr;gap:10px;padding:10px}
.card{background:#fff;border:1px solid #d9dce1;border-radius:8px;padding:10px;margin-bottom:10px}
h3{margin:0 0 6px;font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:#555}
input[type=number]{width:72px} .row{margin:6px 0} small{color:#666;display:block}
button{padding:4px 10px;cursor:pointer}
.stop{background:#c62828;color:#fff;font-weight:bold;padding:8px 18px;border:0;border-radius:6px}
.charts{display:grid;grid-template-columns:1fr 1fr;gap:8px}
canvas{width:100%;height:180px;border:1px solid #d9dce1;border-radius:6px;background:#fff}
table{border-collapse:collapse;width:100%;font-size:12px} td,th{border-bottom:1px solid #e3e5e8;padding:3px 5px;text-align:right}
th{background:#f0f1f3} .ack{font-size:12px;color:#555} .ok{color:#2e7d32} .bad{color:#c62828}
@media(max-width:1000px){main{grid-template-columns:1fr}}
</style></head><body>
<header><b>Robot PID tuner</b>
 <span>Robot IP <input id="ip" placeholder="from serial monitor"> <button onclick="setIp()">Set</button></span>
 <span id="conn">...</span><span id="live"></span>
 <button class="stop" onclick="stopAll()">STOP (space)</button></header>
<main><div>
 <div class="card"><h3>Motion commands</h3>
  <div class="row">Turn <input id="turnDeg" type="number" value="90" step="any"> deg (+ = CCW) &nbsp;max <input id="turnRate" type="number" value="45"> deg/s
   <button onclick="runTurn()">Run turn</button></div>
  <div class="row"><b>Gyro-only turn</b> <input id="gDeg" type="number" value="90" step="any"> deg &nbsp;max PWM frac <input id="gMax" type="number" value="0.3" step="any" style="width:55px">
   lead s <input id="gLead" type="number" value="0.05" step="any" style="width:55px"> <button onclick="runGyroTurn()">Run gyro turn</button>
   <button onclick="series('gturn')">Series: gyro turn</button>
   <small>No encoders, no speed loop: heading PD to PWM, brakes when predicted to reach the target. Start with 20 deg and max 0.2.</small></div>
  <div class="row">Drive <input id="driveDist" type="number" value="0.5" step="any"> m (neg = reverse) &nbsp;max <input id="driveV" type="number" value="0.15" step="any"> m/s
   <button onclick="runDrive()">Run drive</button></div>
  <div class="row">Repeat <input id="reps" type="number" value="10" style="width:50px"> times
   <label><input id="alt" type="checkbox" checked> alternate direction (out and back)</label><br>
   <button onclick="series('turn')">Series: turn</button> <button onclick="series('drive')">Series: drive</button>
   <small>A series waits for each move to finish, then pauses 1.5 s. Measure each result and type it into the table.</small></div>
  <div class="row">Wheel step test: L <input id="wL" type="number" value="0.05" step="any"> R <input id="wR" type="number" value="0.05" step="any">
   m/s (PWM fraction if open loop) for <input id="wT" type="number" value="2" style="width:50px"> s <button onclick="wheelTest()">Run</button> <label><input type="checkbox" id="wOL"> open loop</label>
   <small>Use this first: both measured speeds must be positive when the robot drives forward.</small></div>
  <div class="row"><button onclick="cmd(T.ZERO)">Zero pose</button> <button onclick="cmd(T.CALIB)">Recalibrate gyro (keep still!)</button>
   <button onclick="reload()">Load all from robot</button> <button onclick="cmd(T.SIGN)">Sign test (read serial monitor)</button></div>
 </div>
 <div class="card"><h3>PID gains (loops run on the robot)</h3><div id="gains"></div>
  <small>Change a field, press Apply. The robot echoes what it actually holds. Tune order: wheel speed, then turn, then drive.</small></div>
 <div class="card"><h3>Auto-calibration (robot on the floor, STOP aborts)</h3>
  <div class="row"><button onclick="cmd(T.ACAL,1)">1. Polarity + encoder/gyro signs</button> <button onclick="cmd(T.FLIPR)">Swap right motor</button> <button onclick="cmd(T.FLIPFB)">Flip front/back</button> <button onclick="cmd(T.SWAPLR)">Swap left/right</button>
   <small>Needs 30 cm free. Watch the straight nudge (both wheels +). It must move STRAIGHT toward the front. If it ROTATED, press Swap right motor (one motor is wired against the other). If it went straight toward the BACK, press Flip front/back. With a calibrated gyro the robot detects the rotation case itself. Then press Sign test (serial monitor): LEFT-only must curve the robot to its RIGHT; if it curves LEFT, press Swap left/right. Only run steps 2 to 4 once the nudge is straight and forward and left/right is correct.</small></div>
  <div class="row"><button onclick="cmd(T.ACAL,2)">2. Deadband + speed range</button> <small>Needs 60 cm free. Sets min PWM from the measured breakaway and reports the real speed range.</small></div>
  <div class="row"><button onclick="cmd(T.ACAL,3,1)">3a. Start hand-rotation</button> turns per wheel <input id="revs" type="number" value="5" style="width:50px" step="any">
   <button onclick="cmd(T.ACAL,3,2,num('revs'))">3b. Finish</button> <span id="cap" class="ack"></span>
   <small>Lift the robot, mark each tyre, turn EACH wheel exactly N full turns by hand, press Finish. This gives counts per revolution (the robot cannot measure that itself).</small></div>
  <div class="row"><button onclick="cmd(T.ACAL,4)">4. Track width from gyro</button> <small>Needs a calibrated gyro and steps 1 to 3 done. Spins 360&deg; each way.</small></div>
  <div class="row"><button onclick="cmd(T.SAVE)">Save to robot</button> <button onclick="if(confirm('Erase the saved settings on the robot?'))cmd(T.RSTCFG)">Erase saved</button> <span id="ack13" class="ack"></span>
   <small>Calibration steps save automatically. Gains, geometry and limits you Apply by hand are kept only after Save.</small></div>
  <div id="calres" class="ack" style="font-family:monospace"></div>
  <div class="row"><b>Proposals from your measured trials</b> <button onclick="propose()">Compute</button><div id="prop" class="ack"></div></div>
 </div>
 <div class="card"><h3>Geometry, limits, heading source</h3>
  <div class="row">counts/rev <input id="cpr" type="number" step="any" value="700"> wheel &#8960; m <input id="wd" type="number" step="any" value="0.034">
   track m <input id="tw" type="number" step="any" value="0.1"> <button onclick="applyGeom()">Apply</button> <span id="ack10" class="ack"></span></div>
  <div class="row">min PWM <input id="mn" type="number" value="70"> max PWM <input id="mx" type="number" value="150">
   <button onclick="applyLim()">Apply</button> <span id="ack11" class="ack"></span><br>
   <label><input type="checkbox" id="fL"> invert L motor</label> <label><input type="checkbox" id="fR"> invert R motor</label>
   <label><input type="checkbox" id="fEL"> flip L encoder</label> <label><input type="checkbox" id="fER"> flip R encoder</label>
   <label><input type="checkbox" id="fG"> flip gyro sign</label></div>
  <div class="row">Heading source <select id="hs"><option value="1">gyro</option><option value="0">encoders</option></select>
   gyro scale <input id="gs" type="number" step="any" value="1"> <button onclick="applyHead()">Apply</button> <span id="ack12" class="ack"></span></div>
 </div></div>
<div>
 <div class="card"><h3>Live</h3><div id="read" style="font-family:monospace"></div>
  <div class="charts"><canvas id="c1"></canvas><canvas id="c2"></canvas><canvas id="c3"></canvas><canvas id="c4"></canvas></div></div>
 <div class="card"><h3>Trials (type the independently measured result)</h3>
  <div id="stats"></div><div style="overflow-x:auto"><table id="trials"></table></div>
  <small>Error sign: + = overshoot, - = undershoot. "robot est." compares the robot's final estimate with the command. "true" compares your measurement.
  Only trials that finished normally enter the stats. <a href="/download/telemetry.csv">telemetry.csv</a> <a href="/download/trials.csv">trials.csv</a></small></div>
</div></main>
<script>
const $=id=>document.getElementById(id), sleep=ms=>new Promise(r=>setTimeout(r,ms));
const T={STOP:0,TURN:1,DRIVE:2,WHEEL:3,GAINS:4,GEOM:5,LIMITS:6,ZERO:7,HEAD:8,HELLO:9,CALIB:10,REQ:11,SIGN:12,TURNG:13,ACAL:14,SAVE:15,FLIPFB:16,RSTCFG:17,FLIPR:18,SWAPLR:19};
const GL=[{id:0,n:'Wheel speed',u:'err m/s to PWM fraction, both wheels',d:[3,10,0]},
 {id:1,n:'Turn',u:'heading err rad to rate rad/s',d:[4,0.3,0.15]},
 {id:2,n:'Drive distance',u:'dist err m to speed m/s',d:[2,0,0]},
 {id:3,n:'Drive heading-hold',u:'heading err rad to rate rad/s',d:[3,0,0.1]},
 {id:4,n:'Gyro-only turn',u:'heading err rad to PWM fraction, D on gyro rate rad/s',d:[0.8,0,0.05]}];
const ST=['idle','wheel','turn','drive','calibrating'];
let since=0,buf=[],acks={},trials=[],filled={},mode=2,stopSeries=false,lastTrialsJson='';
async function post(u,o){return (await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)})).json();}
function cmd(type,a=0,b=0,c=0,d=0){return post('/api/cmd',{type,a,b,c,d});}
const num=id=>parseFloat($(id).value);
function stopAll(){stopSeries=true;cmd(T.STOP);}
document.addEventListener('keydown',e=>{if(e.code==='Space'||e.code==='Escape'){e.preventDefault();stopAll();}});
function setIp(){post('/api/ip',{ip:$('ip').value});filled={};}
function reload(){filled={};cmd(T.REQ);}
function runTurn(){return cmd(T.TURN,num('turnDeg'),num('turnRate'));}
function runGyroTurn(){return cmd(T.TURNG,num('gDeg'),num('gMax'),num('gLead'));}
function runDrive(){return cmd(T.DRIVE,num('driveDist'),num('driveV'));}
function wheelTest(){cmd(T.WHEEL,num('wL'),num('wR'),num('wT'),$('wOL').checked?1:0);}
function applyGain(id){cmd(T.GAINS,id,num('kp'+id),num('ki'+id),num('kd'+id));}
function applyGeom(){cmd(T.GEOM,num('cpr'),num('wd'),num('tw'));}
function applyLim(){const f=($('fL').checked?1:0)|($('fR').checked?2:0)|($('fEL').checked?4:0)|($('fER').checked?8:0)|($('fG').checked?16:0);cmd(T.LIMITS,num('mn'),num('mx'),f);}
function applyHead(){cmd(T.HEAD,parseFloat($('hs').value),num('gs'));}
async function series(kind){
 const n=parseInt($('reps').value),alt=$('alt').checked;stopSeries=false;
 for(let i=0;i<n&&!stopSeries;i++){
  const sg=(alt&&i%2)?-1:1,before=trials.length;
  if(kind==='turn')await cmd(T.TURN,sg*num('turnDeg'),num('turnRate'));else if(kind==='gturn')await cmd(T.TURNG,sg*num('gDeg'),num('gMax'),num('gLead'));else await cmd(T.DRIVE,sg*num('driveDist'),num('driveV'));
  const t0=Date.now();while(trials.length===before&&!stopSeries&&Date.now()-t0<25000)await sleep(100);
  await sleep(1500);}
}
$('gains').innerHTML=GL.map(g=>`<div class="row"><b>${g.n}</b> <small>${g.u}</small>
 kp <input id="kp${g.id}" type="number" step="any" value="${g.d[0]}"> ki <input id="ki${g.id}" type="number" step="any" value="${g.d[1]}">
 kd <input id="kd${g.id}" type="number" step="any" value="${g.d[2]}"> <button onclick="applyGain(${g.id})">Apply</button>
 <span id="ack${g.id}" class="ack"></span></div>`).join('');
const same=(a,b)=>Math.abs(a-b)<1e-3*Math.max(1,Math.abs(a));
function ackText(id,vals,inputs){const a=acks[id];if(!a)return '';
 return 'robot: '+a.map(v=>+v.toPrecision(5)).join(' / ')+' '+(inputs.every((v,i)=>same(v,a[i]))?'<span class="ok">matches</span>':'<span class="bad">differs</span>');}
function syncInputs(){
 for(const g of GL){const a=acks[g.id];
  if(a&&!filled['g'+g.id]){$('kp'+g.id).value=a[0];$('ki'+g.id).value=a[1];$('kd'+g.id).value=a[2];filled['g'+g.id]=1;}
  if(a)$('ack'+g.id).innerHTML=ackText(g.id,0,[num('kp'+g.id),num('ki'+g.id),num('kd'+g.id)]);}
 let a=acks[10];if(a){if(!filled.geo){$('cpr').value=a[0];$('wd').value=a[1];$('tw').value=a[2];filled.geo=1;}
  $('ack10').innerHTML=ackText(10,0,[num('cpr'),num('wd'),num('tw')]);}
 a=acks[11];if(a){if(!filled.lim){$('mn').value=a[0];$('mx').value=a[1];const f=a[2];
   $('fL').checked=!!(f&1);$('fR').checked=!!(f&2);$('fEL').checked=!!(f&4);$('fER').checked=!!(f&8);$('fG').checked=!!(f&16);filled.lim=1;}
  $('ack11').textContent='robot: '+a[0]+' / '+a[1]+' flags '+a[2];}
 a=acks[12];if(a){if(!filled.hd){$('hs').value=a[0];$('gs').value=a[1];filled.hd=1;}
  $('ack12').textContent='robot: src '+a[0]+' scale '+(+a[1].toPrecision(5))+' gyro bias '+(+a[2].toFixed(3))+' dps';}
}
function plot(id,ts,series,title){
 const c=$(id);c.width=c.clientWidth;c.height=c.clientHeight;const g=c.getContext('2d'),W=c.width,H=c.height;
 g.fillStyle='#fff';g.fillRect(0,0,W,H);
 const vals=series.flatMap(s=>s.d).filter(Number.isFinite);if(!ts.length||!vals.length)return;
 let lo=Math.min(...vals),hi=Math.max(...vals);if(hi-lo<1e-9){lo-=1;hi+=1;}const p=(hi-lo)*.1;lo-=p;hi+=p;
 const t1=ts[ts.length-1],t0=t1-10,X=t=>(t-t0)/10*W,Y=v=>H-6-(v-lo)/(hi-lo)*(H-26);
 g.strokeStyle='#ccc';g.beginPath();if(lo<0&&hi>0){g.moveTo(0,Y(0));g.lineTo(W,Y(0));}g.stroke();
 for(const s of series){g.strokeStyle=s.c;g.setLineDash(s.dash||[]);g.beginPath();let st=false;
  for(let i=0;i<ts.length;i++){if(ts[i]<t0||!Number.isFinite(s.d[i]))continue;const x=X(ts[i]),y=Y(s.d[i]);st?g.lineTo(x,y):g.moveTo(x,y);st=true;}g.stroke();}
 g.setLineDash([]);g.font='11px sans-serif';g.fillStyle='#333';g.fillText(title+'  ['+lo.toFixed(2)+' .. '+hi.toFixed(2)+']',6,12);
 let lx=6;for(const s of series){g.fillStyle=s.c;g.fillText(s.n,lx,H-8>26?26:26);lx+=g.measureText(s.n).width+14;}
}
function draw(){
 const ts=buf.map(s=>s.t),R=180/Math.PI,k=mode===3?1000:R;
 plot('c1',ts,[{n:mode===3?'dist err mm':'heading err deg',c:'#d32f2f',d:buf.map(s=>s.err*k)},
  {n:'hold err deg',c:'#7b1fa2',d:buf.map(s=>mode===3?s.aux*R:NaN)}],'Error (target - achieved)');
 plot('c2',ts,[{n:'L meas',c:'#1565c0',d:buf.map(s=>s.vl)},{n:'L tgt',c:'#1565c0',dash:[4,3],d:buf.map(s=>s.vlt)},
  {n:'R meas',c:'#ef6c00',d:buf.map(s=>s.vr)},{n:'R tgt',c:'#ef6c00',dash:[4,3],d:buf.map(s=>s.vrt)}],'Wheel speed m/s');
 const g0=buf.length?buf[0].tg:0,e0=buf.length?buf[0].te:0;
 plot('c3',ts,[{n:'gyro',c:'#2e7d32',d:buf.map(s=>(s.tg-g0)*R)},{n:'encoders',c:'#6d4c41',d:buf.map(s=>(s.te-e0)*R)}],'Heading deg: gyro vs encoders');
 plot('c4',ts,[{n:'L pwm',c:'#1565c0',d:buf.map(s=>s.pl)},{n:'R pwm',c:'#ef6c00',d:buf.map(s=>s.pr)}],'PWM (signed)');
}

const CALS=[[20,'breakaway PWM straight L/R, new min PWM'],[21,'breakaway PWM spin L/R, max PWM'],[22,'wheel speed m/s at min PWM, at max PWM, spin at max'],
 [24,'flags, raw gyro spin deg, encoder spin deg'],[26,'counts/rev L, R, right scale'],[27,'track m, ratio ccw, ratio cw']];
const CODE={0:'running',1:'OK',2:'FAILED (see serial monitor)',3:'aborted'};
let lastCal='';
const g5=v=>+(+v).toPrecision(5);
function renderCal(){
 const a=acks[23];let h='';
 if(a){h+=`step ${a[0]}: <b>${CODE[a[1]]||a[1]}</b> (aux ${g5(a[2])})<br>`;
  const key=JSON.stringify(a);if(key!==lastCal){lastCal=key;if(a[1]!==0)filled={};}}
 for(const [k,n] of CALS)if(acks[k])h+=`${n}: ${acks[k].map(g5).join(' / ')}<br>`;
 $('calres').innerHTML=h;
 const s=buf[buf.length-1];
 $('cap').textContent=(s&&s.st==4&&acks[25])?`live counts L ${acks[25][0]} R ${acks[25][1]}`:'';
 if(acks[13])$('ack13').innerHTML=acks[13][1]?'<span class="ok">saved in flash</span> right scale '+g5(acks[13][0]):'<span class="bad">not saved</span> right scale '+g5(acks[13][0]);
}
function applyProp(kind,val){
 const g=acks[10];if(!g)return;
 cmd(T.GEOM,kind==='cpr'?val:g[0],g[1],kind==='trk'?val:g[2],0);setTimeout(()=>cmd(T.SAVE),300);
}
function propose(){
 const g=acks[10];if(!g){$('prop').textContent='robot values not loaded yet';return;}
 const mn=a=>a.reduce((p,q)=>p+q,0)/a.length,sd=a=>{const m=mn(a);return a.length>1?Math.sqrt(a.reduce((p,q)=>p+(q-m)**2,0)/(a.length-1)):0;};
 let h='';
 const d=trials.filter(t=>t.kind==='drive'&&t.result==='ok'&&t.measured!==null&&t.measured!==''&&Math.abs(t.est)>50&&Math.abs(t.measured)>10);
 const bad=d.filter(t=>t.est*t.measured<0);
 if(bad.length)h+=`<span class="bad">Trial ${bad.map(t=>t.id).join(', ')}: the robot moved the OPPOSITE way to its own estimate. That is polarity, not scale: run step 1 / Flip front/back first.</span><br>`;
 else if(d.length){const r=d.map(t=>t.est/t.measured),m=mn(r),s=sd(r),n=g[0]*m;
  h+=`Distance scale: n=${d.length}, robot est / measured = ${m.toFixed(3)} (sd ${s.toFixed(3)}) &rarr; counts/rev ${g5(g[0])} &rarr; <b>${g5(n)}</b> <button onclick="applyProp('cpr',${n})">Apply + save</button>`;
  if(s>0.05*m)h+=' <span class="bad">spread above 5%: fix the scatter before trusting the mean</span>';h+='<br>';}
 else h+='Distance scale: needs finished drive trials with your tape measurement typed in (at least 0.1 m).<br>';
 const t2=trials.filter(t=>t.kind==='turn'&&t.result==='ok'&&t.gyro_ok&&t.dtg!==null&&Math.abs(t.dtg)>=30);
 if(t2.some(t=>t.dte*t.dtg<=0))h+='<span class="bad">Gyro and encoder headings disagree in sign on some turn: run step 1.</span><br>';
 else if(t2.length){const r=t2.map(t=>t.dte/t.dtg),m=mn(r),s=sd(r),n=g[2]*m;
  h+=`Track width: n=${t2.length}, encoder/gyro angle = ${m.toFixed(3)} (sd ${s.toFixed(3)}) &rarr; track ${g5(g[2])} &rarr; <b>${g5(n)}</b> m <button onclick="applyProp('trk',${n})">Apply + save</button> <small>assumes the gyro scale is right and the distance scale is done first</small><br>`;}
 else h+='Track width: needs finished turn trials of 30&deg; or more with a calibrated gyro.<br>';
 $('prop').innerHTML=h;
}
const f1=(v,d=2)=>v==null?'':(+v).toFixed(d);
function renderTrials(){
 const j=JSON.stringify(trials);if(j===lastTrialsJson)return;lastTrialsJson=j;
 let h='<tr><th>#</th><th>kind</th><th>cmd</th><th>robot est. err</th><th>gyro-enc</th><th>overshoot</th><th>settle s</th><th>hold err deg</th><th>result</th><th>measured</th><th>true err</th><th>gains</th></tr>';
 for(const t of trials)h+=`<tr><td>${t.id}</td><td>${t.kind}</td><td>${t.cmd} ${t.unit}</td><td>${f1(t.est_err)}</td><td>${f1(t.gyro_minus_enc)}</td><td>${f1(t.overshoot)}</td><td>${f1(t.settle,1)}</td><td>${f1(t.hold)}</td><td>${t.result}</td>
  <td><input size="6" value="${t.measured??''}" onchange="post('/api/measured',{id:${t.id},value:this.value})"></td><td>${f1(t.true_err)}</td><td style="text-align:left">${t.gains||''}</td></tr>`;
 $('trials').innerHTML=h;
}
function renderStats(st){
 const f=(o,u)=>o?`n=${o.n} mean ${o.mean.toFixed(2)} sd ${o.std.toFixed(2)} max|e| ${o.maxabs.toFixed(2)} ${u}`:'no data';
 $('stats').innerHTML=`<div>Turn: robot est. [${f(st.turn.est,'deg')}] &nbsp; true [${f(st.turn.true,'deg')}]</div>
  <div>Drive: robot est. [${f(st.drive.est,'mm')}] &nbsp; true [${f(st.drive.true,'mm')}]</div>`;
}
async function poll(){
 try{
  const r=await (await fetch('/api/state?since='+since)).json();
  since=r.n;for(const s of r.samples){buf.push(s);if(s.st==2||s.st==3)mode=s.st;}
  if(buf.length>600)buf=buf.slice(-600);
  acks=r.acks;trials=r.trials;
  if(document.activeElement!==$('ip'))$('ip').value=r.robot_ip||'';
  const c=r.conn;$('conn').innerHTML=r.robot_ip?(c.age!=null&&c.age<1.5?`<span style="color:#8f8">connected</span> ${c.rate} pkt/s, lost ${c.lost}`:'<span style="color:#fa6">no telemetry</span>'):'set the robot IP';
  const s=buf[buf.length-1];
  if(s){const fl=s.fl,R=180/Math.PI;
   $('live').textContent=`IMU ${fl&1?'ok':'MISSING'} | gyro cal ${fl&16?'yes':'NO'} | heading from ${fl&2?'gyro':'encoders'}`;
   $('read').innerHTML=`state <b>${ST[s.st]||s.st}</b> | heading ${(s.th*R).toFixed(2)}&deg; (gyro ${(s.tg*R).toFixed(2)}, enc ${(s.te*R).toFixed(2)}) | x ${s.x.toFixed(3)} y ${s.y.toFixed(3)} m<br>
    err ${mode===3?(s.err*1000).toFixed(1)+' mm':(s.err*R).toFixed(2)+'&deg;'} | vL ${s.vl.toFixed(3)} vR ${s.vr.toFixed(3)} m/s | last result ${s.fl&32?'FAULT (see serial monitor)':(s.fl&8?'done':(s.fl&4?'TIMEOUT':'-'))}`;}
  syncInputs();renderCal();draw();renderTrials();renderStats(r.stats);
 }catch(e){$('conn').textContent='tuner server unreachable';}
 setTimeout(poll,100);
}
poll();
</script></body></html>"""


def main():
    global TUNER
    ap = argparse.ArgumentParser()
    ap.add_argument('--robot-ip', default=None)
    ap.add_argument('--robot-id', type=int, default=1)
    ap.add_argument('--telem-port', type=int, default=6005)
    ap.add_argument('--http-port', type=int, default=8000)
    a = ap.parse_args()
    TUNER = Tuner(a)
    threading.Thread(target=TUNER.rx_loop, daemon=True).start()
    threading.Thread(target=TUNER.hello_loop, daemon=True).start()
    srv = ThreadingHTTPServer(('127.0.0.1', a.http_port), H)
    print(f'Open http://127.0.0.1:{a.http_port}   (telemetry UDP {a.telem_port}, robot cmd port {5005 + a.robot_id})')
    print('Logs:', TUNER.tel_path)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""Backend: 3 Ultraschall (RCWL-1655) + MPU-6050 IMU + IM69D130 Stereo-Mikrofon
mit Richtungsbestimmung per 3 kHz Goertzel-Phasendifferenz (d=4.5cm, eindeutig)."""
import time, threading, subprocess, math, array, collections, json
from flask import Flask, jsonify, Response
from gpiozero import DistanceSensor
import smbus

TAU = 2*math.pi
C_SOUND = 343.0          # m/s
FULL = 2**31

# ---------- Ultraschall ----------
MAX_M = 6.0
MAX_CM = int(MAX_M * 100)
SAFE_CM = MAX_CM - 10
US_DELAY = 0.10

PINS = {"sensor1":{"trigger":23,"echo":24},"sensor2":{"trigger":17,"echo":27},"sensor3":{"trigger":22,"echo":5}}
sensors = {n: DistanceSensor(echo=p["echo"], trigger=p["trigger"], max_distance=MAX_M) for n,p in PINS.items()}

for _ in range(3):
    for s in sensors.values():
        try: s.distance
        except: pass

us_lock = threading.Lock()
def read_us():
    data={}
    with us_lock:
        for n,s in sensors.items():
            try:
                d = s.distance
                cm = round(d*100, 1) if d is not None and math.isfinite(d) else None
            except Exception:
                cm = None
            data[n] = {"distance_cm":cm,"in_range":cm is not None and cm < SAFE_CM}
            time.sleep(US_DELAY)
    return data

# ---------- IMU ----------
IMU_ADDR=0x68; imu_bus=smbus.SMBus(1); imu_lock=threading.Lock(); imu_ok=False
def imu_init():
    global imu_ok
    try:
        imu_bus.write_byte_data(IMU_ADDR,0x6B,0); imu_ok=True
    except OSError:
        imu_ok=False
    return imu_ok
imu_init()
if not imu_ok:
    print("WARN: MPU-6050 IMU at 0x68 not responding - IMU disabled, rest of backend runs.")
def _r16(reg):
    hi=imu_bus.read_byte_data(IMU_ADDR,reg); lo=imu_bus.read_byte_data(IMU_ADDR,reg+1); v=(hi<<8)|lo
    return v-65536 if v>32767 else v
def read_imu():
    global imu_ok
    none_resp={"ok":False,"accel_g":None,"gyro_dps":None,"temperature_c":None}
    with imu_lock:
        if not imu_ok and not imu_init():
            return none_resp
        try:
            ax,ay,az=_r16(0x3B)/16384.0,_r16(0x3D)/16384.0,_r16(0x3F)/16384.0
            temp=_r16(0x41)/340.0+36.53; gx,gy,gz=_r16(0x43)/131.0,_r16(0x45)/131.0,_r16(0x47)/131.0
        except OSError:
            imu_ok=False; return none_resp
    return {"ok":True,"accel_g":{"x":round(ax,3),"y":round(ay,3),"z":round(az,3)},
            "gyro_dps":{"x":round(gx,2),"y":round(gy,2),"z":round(gz,2)},"temperature_c":round(temp,1)}

# ---------- Mikrofon + Richtung ----------
DIR_FREQ = 3000.0        # Hz, selbst erzeugter Ton
MIC_DIST = 0.045         # m, gemessener Abstand
SIGN = 1                 # +1: positiv = rechts. Bei vertauscht -> auf -1 setzen.
CAL_FILE = "/home/pi/mic_cal.json"
CAL_PHASE = 0.0
try:
    CAL_PHASE = float(json.load(open(CAL_FILE)).get("cal_phase", 0.0))
except Exception:
    pass

mic_state={"ok":False,"level_pct":0.0,"dbfs":None,"rms_l":0.0,"rms_r":0.0,"peak_pct":0.0,"peak_hold_pct":0.0,"clip":False}
dir_state={"ok":False,"tone_present":False,"angle_deg":0.0,"phase_deg":0.0,"phase_raw":0.0,"tone_pct":0.0,"freq":DIR_FREQ}
mic_lock=threading.Lock()

def goertzel(samples, fs, f):
    w=TAU*f/fs; cw=math.cos(w); sw=math.sin(w); coeff=2*cw; s1=0.0; s2=0.0
    for x in samples:
        s=x+coeff*s1-s2; s2=s1; s1=s
    re=s1-s2*cw; im=s2*sw
    return re, im, math.sqrt(re*re+im*im)

def wrap(p):
    p=(p+math.pi)%TAU-math.pi
    return p

def mic_worker():
    cmd=["arecord","-D","plughw:1","-c","2","-r","48000","-f","S32_LE","-t","raw","-q"]
    FS=48000; CHUNK=4800; nbytes=CHUNK*2*4
    hold=collections.deque()
    while True:
        try:
            p=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
            p.stdout.read(nbytes*5)
            while True:
                buf=p.stdout.read(nbytes)
                if len(buf)<nbytes: break
                a=array.array("i"); a.frombytes(buf); L=a[0::2]; R=a[1::2]
                rl=(sum(x*x for x in L)/len(L))**0.5; rr=(sum(x*x for x in R)/len(R))**0.5
                rms=max(rl,rr); peak=max(abs(x) for x in a)
                dbfs=20*math.log10(rms/FULL) if rms>0 else -120.0
                level=max(0.0,min(100.0,(dbfs+60.0)/60.0*100.0)); clip=peak>=FULL*0.999
                now=time.time(); hold.append((now,level))
                while hold and now-hold[0][0]>3.0: hold.popleft()
                peak_hold=max(v for _,v in hold) if hold else level
                # --- 3 kHz Goertzel auf beiden Kanaelen ---
                reL,imL,magL=goertzel(L,FS,DIR_FREQ); reR,imR,magR=goertzel(R,FS,DIR_FREQ)
                reC=reL*reR+imL*imR; imC=imL*reR-reL*imR
                phase_raw=math.atan2(imC,reC)
                dphi=wrap(phase_raw-CAL_PHASE)
                sinv=C_SOUND*dphi/(TAU*DIR_FREQ*MIC_DIST); sinv=max(-1.0,min(1.0,sinv))
                angle=math.degrees(math.asin(sinv))*SIGN
                ampL=2*magL/CHUNK; ampR=2*magR/CHUNK; tone=(ampL+ampR)/2
                tone_pct=100*tone/FULL; present=tone_pct>0.03
                with mic_lock:
                    mic_state.update(ok=True,level_pct=round(level,1),dbfs=round(dbfs,1),
                        rms_l=round(100*rl/FULL,3),rms_r=round(100*rr/FULL,3),
                        peak_pct=round(100*peak/FULL,1),peak_hold_pct=round(peak_hold,1),clip=clip)
                    dir_state.update(ok=True,tone_present=present,angle_deg=round(angle,1),
                        phase_deg=round(math.degrees(dphi),1),phase_raw=phase_raw,
                        tone_pct=round(tone_pct,3),freq=DIR_FREQ)
        except Exception:
            with mic_lock: mic_state["ok"]=False; dir_state["ok"]=False
        time.sleep(1)

def read_mic():
    with mic_lock: return dict(mic_state)
def read_dir():
    with mic_lock: return dict(dir_state)
threading.Thread(target=mic_worker,daemon=True).start()

# ---------- Flask ----------
app=Flask(__name__)
@app.route("/api/sensors")
def api_sensors(): return jsonify({"timestamp":time.time(),"sensors":read_us()})
@app.route("/api/imu")
def api_imu(): return jsonify({"timestamp":time.time(),"imu":read_imu()})
@app.route("/api/mic")
def api_mic(): return jsonify({"timestamp":time.time(),"mic":read_mic(),"direction":read_dir()})
@app.route("/api/direction")
def api_direction(): return jsonify({"timestamp":time.time(),"direction":read_dir()})
@app.route("/api/all")
def api_all(): return jsonify({"timestamp":time.time(),"sensors":read_us(),"imu":read_imu(),"mic":read_mic(),"direction":read_dir()})
@app.route("/api/calibrate")
def api_calibrate():
    global CAL_PHASE
    with mic_lock: CAL_PHASE=dir_state.get("phase_raw",0.0)
    try: json.dump({"cal_phase":CAL_PHASE},open(CAL_FILE,"w"))
    except Exception: pass
    return jsonify({"calibrated":True,"cal_phase_deg":round(math.degrees(CAL_PHASE),1)})
@app.route("/health")
def health(): return jsonify({"status":"ok"})
@app.route("/")
def index(): return Response(PAGE,mimetype="text/html")

PAGE = """<!doctype html><html><head><meta charset=utf-8>
<title>Sensor-Backend</title>
<style>body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:2em}
h2{color:#aaa;margin-top:1.5em}
.card{display:inline-block;margin:.6em;padding:1.2em 1.6em;background:#222;border-radius:12px;min-width:150px;vertical-align:top}
.val{font-size:1.8em;font-weight:bold;transition:color .2s}.oor{color:#e66}.ok{color:#6e6}.num{color:#6af}
small{color:#888}#status{color:#888;margin-top:1em}#status.err{color:#e66}
#micclip{color:#e66;font-weight:bold;height:1.2em}
#micchart{display:block;margin:.5em auto;background:#1a1a1a;border-radius:8px}
#dirarc{display:block;margin:.4em auto;background:#1a1a1a;border-radius:8px}
button{margin-top:.5em;background:#333;color:#eee;border:1px solid #555;border-radius:6px;padding:.4em .8em;cursor:pointer}
button:hover{background:#444}</style></head><body>
<h1>Sensoren</h1>
<h2>Ultraschall (cm)</h2><div id=us></div>
<h2>IMU (MPU-6050)</h2><div id=imu></div>
<h2>Mikrofon (IM69D130)</h2><div id=micwrap></div>
<h2>Richtung (3 kHz Ton)</h2><div id=dirwrap></div>
<div id=status>verbinde...</div>
<small>JSON: <a style=color:#6af href=/api/all>/api/all</a></small>
<script>
const $=function(id){return document.getElementById(id);};
$("us").innerHTML="<div class=card><div>sensor1</div><div class=val id=s1>--</div></div><div class=card><div>sensor2</div><div class=val id=s2>--</div></div><div class=card><div>sensor3</div><div class=val id=s3>--</div></div>";
$("imu").innerHTML="<div class=card><div>Beschl. (g)</div><div class=val id=acc>--</div></div><div class=card><div>Gyro (deg/s)</div><div class=val id=gyr>--</div></div><div class=card><div>Temp (C)</div><div class=val id=tmp>--</div></div>";
$("micwrap").innerHTML="<div class=card style=min-width:380px><div>Pegel (Verlauf)</div><div class=val id=miclvl>--</div><canvas id=micchart width=360 height=90></canvas><div><small id=micdb></small> &middot; <small id=micpeak></small></div><div id=micclip></div></div>";
$("dirwrap").innerHTML="<div class=card style=min-width:380px><div class=val id=dirang>--</div><canvas id=dirarc width=360 height=120></canvas><div><small id=dirconf></small></div><button id=calbtn>Kalibrieren (Quelle frontal)</button></div>";
const LV=[],LVMAX=200; const cv=$("micchart"),cx=cv.getContext("2d");
function drawChart(ph){const W=cv.width,H=cv.height;cx.clearRect(0,0,W,H);cx.strokeStyle="#333";cx.lineWidth=1;
 for(let i=0;i<=4;i++){const y=H*i/4;cx.beginPath();cx.moveTo(0,y);cx.lineTo(W,y);cx.stroke();}
 cx.strokeStyle="#6af";cx.lineWidth=2;cx.beginPath();
 for(let i=0;i<LV.length;i++){const x=W*i/(LVMAX-1),y=H-(LV[i]/100*H);if(i===0)cx.moveTo(x,y);else cx.lineTo(x,y);}cx.stroke();
 if(ph!=null){const y=H-(ph/100*H);cx.strokeStyle="#e66";cx.setLineDash([5,4]);cx.beginPath();cx.moveTo(0,y);cx.lineTo(W,y);cx.stroke();cx.setLineDash([]);}}
const dc=$("dirarc"),dx=dc.getContext("2d");
function drawDir(angle,present){const W=dc.width,H=dc.height,cx0=W/2,cy0=H-12,r=90;
 dx.clearRect(0,0,W,H);
 dx.strokeStyle="#444";dx.lineWidth=2;dx.beginPath();dx.arc(cx0,cy0,r,Math.PI,0);dx.stroke();
 dx.fillStyle="#666";dx.font="11px sans-serif";dx.textAlign="center";
 [["-90",-90],["-45",-45],["0",0],["45",45],["90",90]].forEach(function(t){var a=t[1]*Math.PI/180;var ux=Math.sin(a),uy=-Math.cos(a);dx.fillText(t[0],cx0+ux*(r+12),cy0+uy*(r+12)+3);
   dx.strokeStyle="#333";dx.beginPath();dx.moveTo(cx0+ux*(r-6),cy0+uy*(r-6));dx.lineTo(cx0+ux*r,cy0+uy*r);dx.stroke();});
 var a=angle*Math.PI/180,ux=Math.sin(a),uy=-Math.cos(a);
 dx.strokeStyle=present?"#6e6":"#555";dx.lineWidth=3;dx.beginPath();dx.moveTo(cx0,cy0);dx.lineTo(cx0+ux*r,cy0+uy*r);dx.stroke();
 dx.fillStyle=present?"#6e6":"#555";dx.beginPath();dx.arc(cx0,cy0,5,0,Math.PI*2);dx.fill();}
async function micTick(){
 try{
  const r=await(await fetch("/api/mic",{cache:"no-store"})).json();const m=r.mic,d=r.direction;
  if(m.ok){$("miclvl").textContent=m.level_pct+" %";$("miclvl").className="val num";
   $("micdb").textContent=m.dbfs+" dBFS";$("micpeak").textContent="Peak(3s): "+m.peak_hold_pct+" %";
   $("micclip").textContent=m.clip?"CLIPPING!":"";LV.push(m.level_pct);while(LV.length>LVMAX)LV.shift();drawChart(m.peak_hold_pct);}
  else{$("miclvl").textContent="kein Signal";$("miclvl").className="val oor";}
  if(d.tone_present){$("dirang").textContent=(d.angle_deg>0?"+":"")+d.angle_deg+"°";$("dirang").className="val ok";
   $("dirconf").textContent="3 kHz erkannt · Phase "+d.phase_deg+"° · Pegel "+d.tone_pct+"%";drawDir(d.angle_deg,true);}
  else{$("dirang").textContent="kein 3 kHz Ton";$("dirang").className="val oor";
   $("dirconf").textContent="Ton-Pegel "+d.tone_pct+"% (Schwelle 0.03%)";drawDir(0,false);}
 }catch(e){}}
async function tick(){
 try{const d=await(await fetch("/api/all",{cache:"no-store"})).json();const map={sensor1:"s1",sensor2:"s2",sensor3:"s3"};
  for(const k in map){const v=d.sensors[k],el=$(map[k]);el.textContent=v.in_range?v.distance_cm+" cm":"ausser Reichw.";el.className="val "+(v.in_range?"ok":"oor");}
  if(d.imu&&d.imu.ok){const a=d.imu.accel_g,g=d.imu.gyro_dps;
   $("acc").textContent=a.x+" / "+a.y+" / "+a.z;$("acc").className="val num";
   $("gyr").textContent=g.x+" / "+g.y+" / "+g.z;$("gyr").className="val num";
   $("tmp").textContent=d.imu.temperature_c;$("tmp").className="val num";}
  else{$("acc").textContent="nicht verbunden";$("acc").className="val oor";
   $("gyr").textContent="--";$("gyr").className="val oor";
   $("tmp").textContent="--";$("tmp").className="val oor";}
  $("status").textContent="aktualisiert "+new Date(d.timestamp*1000).toLocaleTimeString();$("status").className="";
 }catch(e){$("status").textContent="keine Verbindung zum Backend";$("status").className="err";}}
$("calbtn").addEventListener("click",async function(){this.textContent="kalibriert...";await fetch("/api/calibrate");const self=this;setTimeout(function(){self.textContent="Kalibrieren (Quelle frontal)";},1200);});
tick();setInterval(tick,1000);micTick();setInterval(micTick,250);
</script></body></html>"""

if __name__=="__main__":
    app.run(host="0.0.0.0",port=8000,threaded=True)

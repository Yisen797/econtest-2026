/**
 * 摆杆平衡 V4 — S3纯PID控制器 (视频已迁到K230 WiFi)
 *
 * 按键: GPIO2(小球稳中心)  GPIO38(第三题流程O→+5→0→-5→0)
 * 串口: GPIO18(RX) 460800 收K230位置$B,mm
 * Web: http://192.168.4.1 PID调参(视频在K230那边看)
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#define SERVO_PIN   4
#define BUZZ_PIN    14
#define BTN_BAL     47
#define BTN_SEQ     38
#define OMVRX       18
#define OMVTX       17
#define OMV_RX2     16
#define MAX_FRAME   (30*1024)

#define PID_HZ      50
#define PID_DT      (1.0f/PID_HZ)
#define PID_DT_MS   (1000/PID_HZ)
#define DEF_KP      40.0f
#define DEF_KD      18.0f
#define DEF_KI      0.5f
#define IMAX        300.0f
#define OUTMAX      300.0f

WebServer server(80);
WiFiClient mjpegClient;
Preferences prefs;

uint8_t *fb=nullptr, *sb=nullptr;
size_t slen=0; bool frdy=false;
bool jpeg_inf=false; uint32_t jpeg_bi=0; uint8_t jpeg_pv=0;
unsigned long jpeg_lb=0, jpeg_fc=0;

float ball_cm=0.0; bool ball_valid=false;
int servo_us=1500;

float pid_kp=DEF_KP, pid_kd=DEF_KD, pid_ki=DEF_KI;
float pid_setpoint=0.0, pid_integral=0.0, pid_last_err=0.0, pid_output=0.0;
bool pid_enabled=true;
bool pid_slew_reset=false;
bool seq_kick_active=false; int seq_kick_dir=0; unsigned long seq_kick_ms=0;
#define KICK_DURATION 200
#define KICK_STRENGTH 10
float deadband=0.0f;
#define LP_ALPHA 0.3f

enum SeqState { SEQ_IDLE=0, SEQ_HOME_L, SEQ_HOME_R, SEQ_HOLD_0, SEQ_TO_P5, SEQ_HOLD_P5, SEQ_TO_Z0, SEQ_HOLD_Z0, SEQ_TO_N5, SEQ_HOLD_N5, SEQ_TO_Z1, SEQ_HOLD_Z1, SEQ_DONE };
SeqState seq_state=SEQ_IDLE;
unsigned long seq_enter_ms=0, seq_hold_ms=2000;

/* ---- 舵机 ---- */
void servoWrite(int us){
    if(us<500)us=500; if(us>2500)us=2500;
    ledcWrite(0,us*4096/20000); servo_us=us;
}
void servoToCenter(){
    servoWrite(prefs.getInt("home",1265));
    pid_integral=0;
}

/* ---- 蜂鸣器 ---- */
void beep(int ms){ (void)ms; /* 无蜂鸣器, 禁用 */ }

/* ---- 丢球保持 ---- */
void pidControl(){
    int home=prefs.getInt("home",1265);
    if(!pid_enabled){pid_integral=0;pid_last_err=0;return;}

    static float last_bp=0.0; static unsigned long lost_since=0; static bool was_valid=false;

    if(ball_valid){
        last_bp=ball_cm; was_valid=true; lost_since=0;
    }else if(was_valid){
        if(lost_since==0) lost_since=millis();
        unsigned long lost_ms=millis()-lost_since;
        if(lost_ms<500){} else if(lost_ms<1500){ float r=(float)(lost_ms-500)/1000.0f; last_bp=last_bp*(1.0f-r); }
        else{ pid_integral=0; pid_last_err=0; pid_output=0; servoWrite(home); return; }
    }else{
        /* 序列运行中丢球: 持续向SP方向缓慢推, 把球拉回视野 */
        if(seq_state!=SEQ_IDLE && seq_state!=SEQ_DONE){
            static int blind_us=-1; static unsigned long blind_ms=0;
            static SeqState last_blind_state=SEQ_IDLE;
            if(blind_us<0 || seq_state!=last_blind_state){
                blind_us=servo_us; blind_ms=millis(); last_blind_state=seq_state;
            }
            int dir=(pid_setpoint>=0)?1:-1;  /* SP>=0往右扫找球 */
            if(millis()-blind_ms<5000){
                blind_us+=dir*20;  /* 快速扫: 20us/周期 = 1000us/s */
                if(blind_us<700)blind_us=700;
                if(blind_us>2100)blind_us=2100;
                servoWrite(blind_us);
            }
            pid_output=dir*200;
            return;
        }
        pid_integral=0;pid_last_err=0;return;
    }

    static float bp_filt=0.0; static bool bp_first=true;
    float bp_use=ball_valid?ball_cm:last_bp;
    if(bp_first){bp_filt=bp_use;bp_first=false;}
    bp_filt=bp_filt*(1.0f-LP_ALPHA)+bp_use*LP_ALPHA;

    float err=pid_setpoint-bp_filt;
    if(err>-deadband && err<deadband){ err=0; }

    pid_integral+=err*PID_DT;
    if(pid_integral>IMAX)pid_integral=IMAX; if(pid_integral<-IMAX)pid_integral=-IMAX;
    float deriv=(err-pid_last_err)/PID_DT; pid_last_err=err;
    pid_output=pid_kp*err+pid_ki*pid_integral+pid_kd*deriv;
    if(pid_output>OUTMAX)pid_output=OUTMAX; if(pid_output<-OUTMAX)pid_output=-OUTMAX;

    /* 轻柔限幅, PID平滑 */
    { static int cur_us=-1;
      if(cur_us<0 || pid_slew_reset){cur_us=servo_us; pid_slew_reset=false;}
      int target=home+(int)pid_output;
      int diff=target-cur_us;
      if(diff>20)diff=20; else if(diff<-20)diff=-20;
      cur_us+=diff;
      /* TO_N5时限幅: 防止冲过-5卡左边缘 */
      if(seq_state==SEQ_TO_N5 && cur_us<1250) cur_us=1250;
      servoWrite(cur_us);
      /* 稳在±0.3cm持续1秒→锁死 */
      { static int lock_cnt=0; static bool locked=false;
        if(locked) return; /* 已锁, 不再调整 */
        if(ball_valid && ball_cm>-0.6f && ball_cm<0.6f) lock_cnt++;
        else lock_cnt=0;
        if(lock_cnt>30){
          locked=true; pid_enabled=false; lock_cnt=0;
          prefs.putInt("home",servo_us);
          Serial.printf("[STOP] locked at us=%d bp=%.1f\n",servo_us,ball_cm);
        }
      }
    }
}

/* ---- 自动流程 ---- */
void seqUpdate(){
    if(seq_state==SEQ_IDLE||seq_state==SEQ_DONE) return;
    unsigned long e=millis()-seq_enter_ms; SeqState n=seq_state;
    static int loop_cnt=0;
    switch(seq_state){
        case SEQ_HOME_L:  servoWrite(800);  /* 推到最左, 1.5s */
                          if(e>1500)n=SEQ_HOME_R; break;
        case SEQ_HOME_R:  servoWrite(1550); /* 右倾, 球到-4触发PID */
                          if(ball_valid && ball_cm>=-4.0f){ n=SEQ_HOLD_0; }
                          else if(e>4000)n=SEQ_HOLD_0; break;
        case SEQ_HOLD_0:  /* 刹车后开PID稳零 */
                          { static int brake_ms=0;
                            if(brake_ms==0){brake_ms=millis(); servoWrite(1350);}
                            if(millis()-brake_ms>150){ pid_enabled=true; pid_setpoint=0.0f; pid_slew_reset=true; n=SEQ_DONE; }
                          } break;
        /* 第三题: GPIO38, 0→+4→(减速1s)→-5 */
        case SEQ_TO_P5:   pid_setpoint=4.0f;
                          if(e>3000)n=SEQ_TO_Z0; break; /* 3s到+4 */
        case SEQ_TO_Z0:   pid_setpoint=0.0f;
                          if(e>1000)n=SEQ_TO_N5; break; /* 1s减速 */
        case SEQ_TO_N5:   pid_setpoint=-5.0f;
                          if(e>6000)n=SEQ_DONE; break; /* 6s到-5 */
        default: break;
    }
    if(n!=seq_state){ seq_state=n; seq_enter_ms=millis();
        pid_integral=0; pid_last_err=0; pid_slew_reset=true;
        if(seq_state==SEQ_DONE){ pid_setpoint=0.0f; Serial.println("[SEQ] DONE"); }
    }
}

/* ---- 串口: 只读位置(视频已迁WiFi) ---- */
void readSerial(){
    static uint8_t pv=0; static unsigned long lb=0;
    while(Serial2.available()>0){
        uint8_t b=Serial2.read(); lb=millis();
        if(b=='$'){ char buf[8]; int i=0; unsigned long t=millis();
            while(i<7&&millis()-t<10){ if(Serial2.available()){ buf[i]=Serial2.read(); if(buf[i]=='\n'){buf[i]=0;break;} i++; } }
            if(buf[0]=='B'&&buf[1]==','){ int mm=atoi(buf+2); if(mm==999)ball_valid=false; else{ball_cm=mm/10.0f;ball_valid=true;} }
        }
        pv=b;
        if(millis()-lb>200) break;  // 每轮最多读200ms
    }
}

/* ---- OpenMV视频 (Serial1 GPIO16) ---- */
void readOMV(){
    if(!fb||!sb) return; /* 缓冲未分配则跳过 */
    int a; while((a=Serial1.available())>0){
        if(a>512)a=512;
        while(a-->0){
            uint8_t b=Serial1.read(); jpeg_lb=millis();
            if(!jpeg_inf){
                if(jpeg_pv==0xFF && b==0xD8){ jpeg_inf=true; jpeg_bi=0; fb[jpeg_bi++]=0xFF; fb[jpeg_bi++]=0xD8; }
            }else{
                if(jpeg_bi<MAX_FRAME) fb[jpeg_bi++]=b;
                if(jpeg_pv==0xFF && b==0xD9){ jpeg_inf=false;
                    if(jpeg_bi>=1024){ memcpy(sb,fb,jpeg_bi); slen=jpeg_bi; frdy=true; jpeg_fc++; }
                    jpeg_bi=0;
                }
            }
            jpeg_pv=b;
        }
        if(jpeg_inf && millis()-jpeg_lb>500){ jpeg_inf=false; jpeg_bi=0; }
    }
}
void pushMJ(){
    if(!mjpegClient||!mjpegClient.connected()||!frdy) return;
    size_t len=slen; frdy=false;
    mjpegClient.print("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "+String(len)+"\r\n\r\n");
    mjpegClient.write(sb,len); mjpegClient.print("\r\n");
}
void hS(){ WiFiClient c=server.client();
    c.print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"); mjpegClient=c; }

/* ---- 按键 ---- */
void readBtns(){
    static int db[2]={0,0}; static bool last[2]={true,true};
    int pins[2]={BTN_BAL,BTN_SEQ};
    for(int i=0;i<2;i++){
        bool raw=digitalRead(pins[i]);
        if(raw!=last[i]){last[i]=raw;db[i]=0;
            if(!raw){
                if(i==0){ /* GPIO47: 归零 */
                    seq_state=SEQ_HOME_L; seq_enter_ms=millis();
                    pid_enabled=false; pid_integral=0; pid_last_err=0; pid_slew_reset=true;
                    Serial.println("[BTN] GPIO47 HOME"); }
                else{ /* GPIO38: 第三题 */
                    seq_state=SEQ_TO_P5; seq_enter_ms=millis();
                    pid_enabled=true; pid_setpoint=4.0f; pid_integral=0; pid_last_err=0; pid_slew_reset=true;
                    Serial.println("[BTN] GPIO38 SEQ"); }
            }
        }else db[i]=0;
    }
}

/* ---- Web ---- */
void hD(){
    float err=pid_setpoint-ball_cm;
    String j="{\"fc\":"+String(jpeg_fc)+",\"us\":"+String(servo_us)+",\"bp\":"+String(ball_cm,1)+",\"bv\":"+String(ball_valid?1:0);
    j+=",\"err\":"+String(err,1)+",\"out\":"+String(pid_output,1);
    j+=",\"kp\":"+String(pid_kp,1)+",\"kd\":"+String(pid_kd,1)+",\"ki\":"+String(pid_ki,2);
    j+=",\"sp\":"+String(pid_setpoint,1)+",\"db\":"+String(deadband,2);
    j+=",\"pe\":"+String(pid_enabled?1:0);
    j+=",\"sname\":\""+String(seq_state==SEQ_IDLE?"待机":seq_state==SEQ_DONE?"完成":"流程中")+"\"}";
    server.send(200,"application/json",j);
}
void hSetPid(){
    if(server.hasArg("kp"))pid_kp=server.arg("kp").toFloat();
    if(server.hasArg("kd"))pid_kd=server.arg("kd").toFloat();
    if(server.hasArg("ki"))pid_ki=server.arg("ki").toFloat();
    if(server.hasArg("sp"))pid_setpoint=server.arg("sp").toFloat();
    prefs.putFloat("kp",pid_kp);prefs.putFloat("kd",pid_kd);prefs.putFloat("ki",pid_ki);
    server.send(200,"application/json","{\"ok\":1}");
}
void hSetDB(){ if(server.hasArg("db"))deadband=server.arg("db").toFloat(); server.send(200,"application/json","{\"ok\":1}"); }
void hHome(){ servoToCenter(); pid_setpoint=0; pid_integral=0; pid_last_err=0; seq_state=SEQ_IDLE; server.send(200,"application/json","{\"ok\":1}"); }
void hSetHome(){ prefs.putInt("home",servo_us); server.send(200,"application/json","{\"ok\":1,\"us\":"+String(servo_us)+"}"); }

void hR(){ server.send(200,"text/html",R"raw(<!DOCTYPE html><html lang="zh"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>车载平衡滚球</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:system-ui;background:#1a1a2e;color:#eee;padding:8px;max-width:500px;margin:0 auto}
h1{text-align:center;color:#e94560;margin-bottom:6px;font-size:16px}
.card{background:#16213e;border-radius:12px;padding:10px;margin-bottom:6px}
h2{font-size:13px;color:#e94560;margin-bottom:4px}
#cv{border-radius:8px;width:100%;background:#000}
.info{text-align:center;color:#888;font-size:11px;margin-top:2px}
.rec-item{background:#0a0a1a;border-radius:8px;padding:8px;margin-top:4px}
.rec-item video{width:100%;border-radius:4px}
.rec-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px}
.rec-head span{font-size:11px;color:#888}
.rec-head button{font-size:11px;padding:2px 8px;border:none;border-radius:4px;background:#e94560;color:#fff}
</style></head><body>
<h1>车载平衡滚球</h1>
<div class="card"><h2>实时视频</h2>
<canvas id="cv" width="320" height="240" style="width:100%;border-radius:8px;background:#000"></canvas>
<img id="cam" src="/stream" style="display:none" onload="copyFrame()">
<div style="display:flex;gap:6px;margin-top:4px;align-items:center">
<button id="recBtn" onclick="toggleRec()" style="flex:1;padding:8px;border:none;border-radius:6px;background:#e94560;color:#fff;font-weight:700;font-size:14px">开始录制</button>
<span id="recStatus" class="info" style="color:#e94560;min-width:60px;text-align:right"></span></div>
<div class="info">帧:<span id="fc">0</span> 球位:<span id="bp" style="color:#e94560;font-weight:700">?</span></div></div>
<div id="recList"></div>
<script>
setInterval(function(){fetch('/data').then(function(r){return r.json()}).then(function(d){
document.getElementById("fc").textContent=d.fc;document.getElementById("bp").textContent=d.bv?d.bp.toFixed(1)+"cm":"?";
});},500);
var cv=document.getElementById("cv"),ctx=cv.getContext("2d"),cam=document.getElementById("cam");
function copyFrame(){
  ctx.drawImage(cam,0,0,320,240);
  // 刻度尺 + 中线
  var cx=149,w=320,h=240,pxPerCm=15.7; // 中轴左移1cm, 标尺-8=实际-5cm
  ctx.strokeStyle="rgba(0,0,0,0.8)"; ctx.lineWidth=2;
  ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,h); ctx.stroke(); // 中线
  ctx.fillStyle="#000"; ctx.font="bold 11px monospace";
  ctx.fillText("0",cx-6,14);
  ctx.strokeStyle="rgba(0,0,0,0.6)"; ctx.lineWidth=1;
  for(var cm=-12;cm<=12;cm++){
    var x=cx+cm*pxPerCm; if(x<0||x>w) continue;
    var th=(cm%5==0)?12:6;
    ctx.beginPath(); ctx.moveTo(x,h-th); ctx.lineTo(x,h); ctx.stroke();
    if(cm%5==0&&cm!=0){ ctx.fillText((cm>0?"+":"")+cm,x-14,h-16); }
  }
}
var rec,chunks=[],recs=[];
function toggleRec(){
  if(rec&&rec.state=="recording"){rec.stop();}else{
    chunks=[];var s=cv.captureStream(15);rec=new MediaRecorder(s,{mimeType:"video/webm"});
    rec.ondataavailable=function(e){if(e.data.size)chunks.push(e.data)};
    rec.onstop=function(){var b=new Blob(chunks,{type:"video/webm"});var u=URL.createObjectURL(b);
      var n=new Date().toLocaleTimeString();recs.push({url:u,time:n});renderList();
      document.getElementById("recBtn").textContent="开始录制";document.getElementById("recBtn").style.background="#e94560";
      document.getElementById("recStatus").textContent="";};
    rec.start();document.getElementById("recBtn").textContent="停止录制";
    document.getElementById("recBtn").style.background="#ff0";document.getElementById("recBtn").style.color="#000";
    document.getElementById("recStatus").textContent="● 录制中";
  }
}
function renderList(){var h="";for(var i=recs.length-1;i>=0;i--){h+='<div class="rec-item"><div class="rec-head"><span>录像 #'+(i+1)+' '+recs[i].time+'</span>';h+='<button onclick="delRec('+i+')">删除</button></div>';h+='<video controls src="'+recs[i].url+'" style="width:100%"></video></div>';}document.getElementById("recList").innerHTML=h;}
function delRec(i){recs.splice(i,1);renderList()}
setInterval(copyFrame,66);
</script></body></html>)raw"); }

void setup(){
    Serial.begin(115200); delay(300);
    Serial.println("\n=== 摆杆平衡 V4 S3纯PID ===");

    WiFi.mode(WIFI_AP); WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.setSleep(false);
    WiFi.softAP("ESP32-S3-VIDEO","12345678",11);
    Serial.printf("AP: %s\n",WiFi.softAPIP().toString().c_str());

    ledcSetup(0,50,12); ledcAttachPin(SERVO_PIN,0);
    prefs.begin("bal",false);
    pid_kp=prefs.getFloat("kp",DEF_KP);pid_kd=prefs.getFloat("kd",DEF_KD);pid_ki=prefs.getFloat("ki",DEF_KI);
    if(pid_kp<0.01f)pid_kp=DEF_KP; if(pid_kd<0.005f)pid_kd=DEF_KD;
    servoToCenter();

    fb=(uint8_t*)ps_malloc(MAX_FRAME); sb=(uint8_t*)ps_malloc(MAX_FRAME);

    Serial2.setRxBufferSize(65536); Serial2.begin(460800,SERIAL_8N1,OMVRX,OMVTX);
    Serial1.setRxBufferSize(32768); Serial1.begin(460800,SERIAL_8N1,OMV_RX2,-1);

    /* 蜂鸣器: 不配置引脚, GPIO14靠近Flash SPI易串扰 */
    pinMode(BTN_BAL,INPUT_PULLUP); pinMode(BTN_SEQ,INPUT_PULLUP);

    server.on("/",hR); server.on("/data",hD); server.on("/stream",hS);
    server.on("/setpid",hSetPid); server.on("/setdb",hSetDB);
    server.on("/home",hHome); server.on("/sethome",hSetHome);
    server.begin();
    Serial.println("Web PID: http://"+WiFi.softAPIP().toString());
}

void loop(){
    readOMV();
    readSerial();
    readBtns();
    static unsigned long last_pid=0;
    if(millis()-last_pid>=PID_DT_MS){last_pid=millis();seqUpdate();pidControl();}
    server.handleClient(); pushMJ();
    static unsigned long last_dbg=0;
    if(millis()-last_dbg>100){
        last_dbg=millis();
        Serial.printf("[S3] bp=%.1f ok=%d us=%d out=%.1f kp=%.1f kd=%.1f ki=%.2f sp=%.1f\n",
            ball_cm,ball_valid?1:0,servo_us,pid_output, pid_kp,pid_kd,pid_ki,pid_setpoint);
    }

    /* 串口PID调参命令: $PID,kp,kd,ki\n  例: $PID,35,20,0.3 */
    while(Serial.available()>0){
        char c=Serial.read();
        if(c=='$'){
            char buf[32]; int i=0; unsigned long t=millis();
            while(i<31&&millis()-t<20){ if(Serial.available()){ buf[i]=Serial.read(); if(buf[i]=='\n'){buf[i]=0;break;} i++; } }
            buf[i]=0;
            if(strncmp(buf,"PID,",4)==0){
                char *p=buf+4; float nk[3]={pid_kp,pid_kd,pid_ki}; int n=0;
                char *tok=strtok(p,","); while(tok&&n<3){ nk[n++]=atof(tok); tok=strtok(NULL,","); }
                pid_kp=nk[0]; pid_kd=nk[1]; pid_ki=nk[2];
                prefs.putFloat("kp",pid_kp);prefs.putFloat("kd",pid_kd);prefs.putFloat("ki",pid_ki);
                pid_integral=0; pid_last_err=0; pid_enabled=true; pid_slew_reset=true;
                Serial.printf("[PID] SET kp=%.1f kd=%.1f ki=%.2f\n",pid_kp,pid_kd,pid_ki);
            }else if(strncmp(buf,"SP,",3)==0){
                pid_setpoint=atof(buf+3); pid_integral=0; pid_last_err=0;
                Serial.printf("[SP] SET sp=%.1f\n",pid_setpoint);
            }else if(strncmp(buf,"SERVO,",6)==0){
                int us=atoi(buf+6); servoWrite(us);
                pid_enabled=false; pid_integral=0; pid_last_err=0;
                Serial.printf("[SERVO] SET us=%d (PID off)\n",us);
            }else if(strncmp(buf,"SEQ",3)==0){
                seq_state=SEQ_HOME_L; seq_enter_ms=millis();
                pid_enabled=false; pid_integral=0; pid_last_err=0;
                Serial.println("[SEQ] HOMING...");
            }else if(strncmp(buf,"HOME,",5)==0){
                int us=atoi(buf+5); prefs.putInt("home",us);
                Serial.printf("[HOME] SAVED us=%d\n",us);
            }
        }
    }
    delay(2);
}

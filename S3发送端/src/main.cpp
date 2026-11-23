/**
 * ESP32-S3 视频站 — AP + OpenMV + Web 录像
 * ==========================================
 * OpenMV P4(TX)→S3 GPIO16(RX), GND共地
 * S3开AP, 电脑连WiFi打开网页即可看视频+录像+回放
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#define MAX_FRAME   (40 * 1024)
#define UART_BAUD   460800

WebServer server(80);
WiFiClient mjpegClient;

uint8_t *fb = nullptr, *sb = nullptr;
size_t slen = 0; bool frdy = false;
bool inf = false; uint32_t bi = 0; uint8_t pv = 0x00;
unsigned long fc = 0, lb = 0;
float ball_pos = -1.0;  /* 球位置 (cm) */

void readOMV() {
    int a; while ((a = Serial2.available()) > 0) while (a-- > 0) {
        uint8_t b = Serial2.read(); lb = millis();
        if (!inf) {
            if (pv == 0xFF && b == 0xD8) {
                inf = true; bi = 0; fb[bi++] = 0xFF; fb[bi++] = 0xD8;
            } else if (b == 'P' && !inf) {
                /* 球位置: P12.3\n */
                char buf[8]; int i=0;
                unsigned long t=millis();
                while(i<7 && millis()-t<10){
                    if(Serial2.available()){ buf[i]=Serial2.read(); if(buf[i]=='\n'){buf[i]=0;break;} i++; }
                }
                ball_pos = atof(buf);
            }
        } else {
            if (bi < MAX_FRAME) fb[bi++] = b;
            if (pv == 0xFF && b == 0xD9) { inf = false;
                if (bi >= 1024) { memcpy(sb, fb, bi); slen = bi; frdy = true; fc++; } bi = 0; }
        }
        pv = b;
    }
    if (inf && millis() - lb > 500) { inf = false; bi = 0; }
}

void pushMJ() {
    if (!mjpegClient || !mjpegClient.connected() || !frdy) return;
    size_t len = slen; frdy = false;
    String h = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(len) + "\r\n\r\n";
    mjpegClient.print(h); mjpegClient.write(sb, len); mjpegClient.print("\r\n");
}

void hS() { WiFiClient c = server.client();
    c.print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n"); mjpegClient = c; }
void hD() { server.send(200, "application/json",
    "{\"fc\":" + String(fc) + ",\"bp\":" + String(ball_pos,1) + "}"); }
void hR() { const char h[] = R"raw(<!DOCTYPE html><html lang="zh"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>车载平衡滚球</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#eee;padding:8px;max-width:500px;margin:0 auto}
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
setInterval(function(){
  fetch('/data').then(function(r){return r.json()}).then(function(d){
    document.getElementById("fc").textContent=d.fc;document.getElementById("bp").textContent=d.bp>0?d.bp.toFixed(1)+"cm":"?";
  });
},500);
var cv=document.getElementById("cv"),ctx=cv.getContext("2d"),cam=document.getElementById("cam");
function copyFrame(){ctx.drawImage(cam,0,0,320,240)}
var rec,chunks=[],recs=[];
function toggleRec(){
  if(rec&&rec.state=="recording"){
    rec.stop();
  }else{
    chunks=[];
    var s=cv.captureStream(15);
    rec=new MediaRecorder(s,{mimeType:"video/webm"});
    rec.ondataavailable=function(e){if(e.data.size)chunks.push(e.data)};
    rec.onstop=function(){
      var b=new Blob(chunks,{type:"video/webm"});
      var u=URL.createObjectURL(b);
      var n=new Date().toLocaleTimeString();
      recs.push({url:u,time:n});
      renderList();
      document.getElementById("recBtn").textContent="开始录制";
      document.getElementById("recBtn").style.background="#e94560";
      document.getElementById("recStatus").textContent="";
      document.getElementById("recBtn").style.color="#fff";
    };
    rec.start();
    document.getElementById("recBtn").textContent="停止录制";
    document.getElementById("recBtn").style.background="#ff0";
    document.getElementById("recBtn").style.color="#000";
    document.getElementById("recStatus").textContent="● 录制中";
  }
}
function renderList(){
  var h="";
  for(var i=recs.length-1;i>=0;i--){
    h+='<div class="rec-item"><div class="rec-head"><span>录像 #'+(i+1)+' '+recs[i].time+'</span>';
    h+='<button onclick="delRec('+i+')">删除</button></div>';
    h+='<video controls src="'+recs[i].url+'" style="width:100%"></video></div>';
  }
  document.getElementById("recList").innerHTML=h;
}
function delRec(i){recs.splice(i,1);renderList()}
setInterval(copyFrame,66);
</script></body></html>)raw";
    server.send(200, "text/html; charset=utf-8", h); }

void setup() {
    Serial.begin(115200); delay(300);
    Serial.println("\n=== S3 视频站 ===");

    fb = (uint8_t*)ps_malloc(MAX_FRAME);
    sb = (uint8_t*)ps_malloc(MAX_FRAME);
    if (!fb || !sb) { Serial.println("MEM FAIL"); while (1); }
    Serial.printf("缓冲: %dKB\n", MAX_FRAME/1024);

    WiFi.mode(WIFI_AP);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    WiFi.softAP("ESP32-S3-VIDEO", "12345678", 11);
    Serial.printf("AP: %s CH11\n", WiFi.softAPIP().toString().c_str());

    Serial2.setRxBufferSize(32768);
    Serial2.begin(UART_BAUD, SERIAL_8N1, 16, 17);
    Serial.printf("OpenMV UART: %d波特\n", UART_BAUD);

    server.on("/", hR);
    server.on("/data", hD);
    server.on("/stream", hS);
    server.begin();
    Serial.println("Web: http://" + WiFi.softAPIP().toString());
}

void loop() {
    readOMV();
    server.handleClient();
    pushMJ();
    delay(2);
}

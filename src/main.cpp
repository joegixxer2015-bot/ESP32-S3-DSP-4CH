#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <math.h>
#include <Update.h>
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "AudioTools.h"
void setDefaultPreset();void updateAllCoeffs();void applyAll();void loadSettings();void saveSettings();void saveUserPreset(int);void loadUserPreset(int);void initWifiAndEspNow();void setupWeb();void setupI2S();void flushTank();
#define I2S_BCK_PIN 15
#define I2S_LRCK_PIN 16
#define I2S_DATA1_PIN 17
#define I2S_IN_BCK 4
#define I2S_IN_LRCK 5
#define I2S_IN_DATA 6
AudioInfo info(44100,2,16);
Preferences prefs;
WebServer server(80);
typedef struct{int controlMode;int value;bool isMuted;} Msg;
Msg incomingData;
bool apActive=true;
unsigned long apLastSeen=0;
const unsigned long AP_TIMEOUT=120000;
bool isUpdating=false;
class Biquad{
public:
enum Type{PEAKING,LOW_SHELF,HIGH_SHELF,LOW_PASS,HIGH_PASS};
void set(Type t,float freq,float q,float gainDb,float sr){
float A=powf(10.0f,gainDb/40.0f);float w=2.0f*(float)M_PI*freq/sr;float sn=sinf(w),cs=cosf(w);float alpha=sn/(2.0f*q);
float b0_,b1_,b2_,a0_,a1_,a2_;
if(t==LOW_PASS){b0_=(1-cs)/2;b1_=1-cs;b2_=(1-cs)/2;a0_=1+alpha;a1_=-2*cs;a2_=1-alpha;}
else if(t==HIGH_PASS){b0_=(1+cs)/2;b1_=-(1+cs);b2_=(1+cs)/2;a0_=1+alpha;a1_=-2*cs;a2_=1-alpha;}
else if(t==PEAKING){b0_=1+alpha*A;b1_=-2*cs;b2_=1-alpha*A;a0_=1+alpha/A;a1_=-2*cs;a2_=1-alpha/A;}
else if(t==LOW_SHELF){float sA=2*sqrtf(A)*alpha;b0_=A*((A+1)-(A-1)*cs+sA);b1_=2*A*((A-1)-(A+1)*cs);b2_=A*((A+1)-(A-1)*cs-sA);a0_=(A+1)+(A-1)*cs+sA;a1_=-2*((A-1)+(A+1)*cs);a2_=(A+1)-(A-1)*cs-sA;}
else{float sA=2*sqrtf(A)*alpha;b0_=A*((A+1)+(A-1)*cs+sA);b1_=-2*A*((A-1)+(A+1)*cs);b2_=A*((A+1)+(A-1)*cs-sA);a0_=(A+1)-(A-1)*cs+sA;a1_=2*((A-1)-(A+1)*cs);a2_=(A+1)-(A-1)*cs-sA;}
float inv=1.0f/a0_;b0=b0_*inv;b1=b1_*inv;b2=b2_*inv;a1=a1_*inv;a2=a2_*inv;}
float process(float x){float y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2;x2=x1;x1=x;y2=y1;y1=y;if(y>32767)y=32767;if(y<-32768)y=-32768;return y;}
void reset(){x1=x2=y1=y2=0;}
private:float b0=1,b1=0,b2=0,a1=0,a2=0,x1=0,x2=0,y1=0,y2=0;
};
struct StereoBiquad{Biquad L,R;void set(Biquad::Type t,float f,float q,float g,float sr){L.set(t,f,q,g,sr);R.set(t,f,q,g,sr);}void process(float &l,float &r){l=L.process(l);r=R.process(r);}};
struct Preset{float webEQ[5];float chA_hp,chA_lp;int chA_slope;float chA_delay;int chA_mode;float chB_hp,chB_lp;int chB_slope;float chB_delay;int chB_mode;};
Preset current;Preset defaultPreset;int selectedUser=1;
StereoBiquad webEQ[5];StereoBiquad remoteEQ[3];StereoBiquad chB_HP1,chB_HP2,chB_LP1,chB_LP2;
volatile float targetBass=0,targetMid=0,targetTreble=0,targetVolume=0.8f;volatile bool targetMuted=false;
float curBass=0,curMid=0,curTreble=0;volatile bool eqDirty=false,volDirty=false,nvsDirty=false;
unsigned long lastNvsChange=0;bool pendingNvs=false;
I2SStream i2sStreamIn; I2SStream i2sStreamOut;
unsigned long lastAudioTime=0;bool gapMuted=false;float fadeVolume=0.8f;unsigned long fadeStartTime=0;bool isFadingUp=false;
void setDefaultPreset(){
for(int i=0;i<5;i++)defaultPreset.webEQ[i]=0;
defaultPreset.chA_hp=30;defaultPreset.chA_lp=500;defaultPreset.chA_slope=24;defaultPreset.chA_delay=0;defaultPreset.chA_mode=1;
defaultPreset.chB_hp=500;defaultPreset.chB_lp=20000;defaultPreset.chB_slope=24;defaultPreset.chB_delay=0.15;defaultPreset.chB_mode=0;
current=defaultPreset;}
void updateAllCoeffs(){
float sr=(float)info.sample_rate;float wf[5]={60,250,1000,4000,10000};
for(int i=0;i<5;i++)webEQ[i].set(Biquad::PEAKING,wf[i],1.0f,current.webEQ[i],sr);
remoteEQ[0].set(Biquad::LOW_SHELF,125,0.7f,curBass,sr);remoteEQ[1].set(Biquad::PEAKING,1000,0.9f,curMid,sr);remoteEQ[2].set(Biquad::HIGH_SHELF,6500,0.75f,curTreble,sr);
chB_HP1.set(Biquad::HIGH_PASS,current.chB_hp,0.707f,0,sr);chB_HP2.set(Biquad::HIGH_PASS,current.chB_hp,0.707f,0,sr);
chB_LP1.set(Biquad::LOW_PASS,current.chB_lp,0.707f,0,sr);chB_LP2.set(Biquad::LOW_PASS,current.chB_lp,0.707f,0,sr);}
void applyAll(){updateAllCoeffs();}
void loadSettings(){
prefs.begin("dsp_tuner",true);
targetVolume=constrain(prefs.getFloat("vol",0.8f),0,1);targetBass=constrain(prefs.getFloat("bass",0),-15,15);
targetMid=constrain(prefs.getFloat("mid",0),-12,12);targetTreble=constrain(prefs.getFloat("treble",0),-12,12);
targetMuted=prefs.getBool("muted",false);
size_t n=prefs.getBytes("user1",&current,sizeof(Preset));if(n!=sizeof(Preset))current=defaultPreset;
selectedUser=prefs.getInt("selUser",1);prefs.end();
curBass=targetBass;curMid=targetMid;curTreble=targetTreble;fadeVolume=targetMuted?0:targetVolume;}
void saveSettings(){
prefs.begin("dsp_tuner",false);prefs.putFloat("vol",targetVolume);prefs.putFloat("bass",targetBass);prefs.putFloat("mid",targetMid);prefs.putFloat("treble",targetTreble);prefs.putBool("muted",targetMuted);prefs.putInt("selUser",selectedUser);prefs.putBytes((String("user")+selectedUser).c_str(),&current,sizeof(Preset));prefs.end();}
void saveUserPreset(int idx){selectedUser=idx;prefs.begin("dsp_tuner",false);prefs.putBytes((String("user")+idx).c_str(),&current,sizeof(Preset));prefs.putInt("selUser",idx);prefs.end();}
void loadUserPreset(int idx){selectedUser=idx;prefs.begin("dsp_tuner",true);size_t n=prefs.getBytes((String("user")+idx).c_str(),&current,sizeof(Preset));prefs.end();if(n!=sizeof(Preset))return;applyAll();}
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info,const uint8_t *data,int len)
#else
void OnDataRecv(const uint8_t *mac,const uint8_t *data,int len)
#endif
{
if(len<(int)sizeof(incomingData)){Serial.printf("ESP-NOW BAD LEN %d expect %d\n",len,(int)sizeof(incomingData));return;}
memcpy(&incomingData,data,sizeof(incomingData));
Serial.printf("ESP-NOW RX mode=%d val=%d muted=%d | BEFORE vol=%.2f bass=%.1f mid=%.1f treble=%.1f\n",incomingData.controlMode,incomingData.value,(int)incomingData.isMuted,targetVolume,targetBass,targetMid,targetTreble);
if(incomingData.controlMode<0||incomingData.controlMode>3){Serial.println("ESP-NOW IGNORE bad mode");return;}
if(incomingData.isMuted!=targetMuted){targetMuted=incomingData.isMuted;volDirty=true;nvsDirty=true;Serial.printf("ESP-NOW MUTE CHANGED -> %d\n",(int)targetMuted);}
if(incomingData.isMuted)return;
int m=incomingData.controlMode;
if(m==0){float v=constrain((float)incomingData.value/100.0f,0,1);if(fabsf(v-targetVolume)>0.001){targetVolume=v;volDirty=true;nvsDirty=true;Serial.printf("ESP-NOW VOL -> %.2f\n",targetVolume);}}
if(m==1){float v=constrain((float)incomingData.value,-15,15);if(fabsf(v-targetBass)>0.01){targetBass=v;eqDirty=true;nvsDirty=true;Serial.printf("ESP-NOW BASS -> %.1f\n",targetBass);}}
if(m==2){float v=constrain((float)incomingData.value,-12,12);if(fabsf(v-targetMid)>0.01){targetMid=v;eqDirty=true;nvsDirty=true;Serial.printf("ESP-NOW MID -> %.1f\n",targetMid);}}
if(m==3){float v=constrain((float)incomingData.value,-12,12);if(fabsf(v-targetTreble)>0.01){targetTreble=v;eqDirty=true;nvsDirty=true;Serial.printf("ESP-NOW TREBLE -> %.1f\n",targetTreble);}}
Serial.printf("ESP-NOW AFTER vol=%.2f bass=%.1f mid=%.1f treble=%.1f muted=%d\n",targetVolume,targetBass,targetMid,targetTreble,(int)targetMuted);
}
void initWifiAndEspNow(){
WiFi.persistent(false);WiFi.mode(WIFI_AP_STA);WiFi.softAP("DSP-TUNER","12345678",1);WiFi.disconnect();
esp_wifi_set_promiscuous(true);esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);esp_wifi_set_promiscuous(false);
if(esp_now_init()==ESP_OK){esp_now_register_recv_cb(OnDataRecv);Serial.println("ESP-NOW Ready Ch1");}else Serial.println("ESP-NOW Fail");
apActive=true;apLastSeen=millis();Serial.println("AP ON - 5 min timeout if no client");}
void checkApTimeout(){
if(!apActive||isUpdating)return;
unsigned long now=millis();
int clients=WiFi.softAPgetStationNum();
if(clients>0){apLastSeen=now;return;}
if(now-apLastSeen>=AP_TIMEOUT){
WiFi.softAPdisconnect(true);
apActive=false;
esp_wifi_set_promiscuous(true);esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);esp_wifi_set_promiscuous(false);
Serial.println("AP OFF - no client 5 min, restart S3 to enable again. ESP-NOW still works.");
}}
const char* HTML_PAGE = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>DSP TUNER</title><style>*{box-sizing:border-box}body{margin:0;font-family:Arial,sans-serif;background:#fff;color:#111;padding:16px;max-width:480px;margin:auto}h1{text-align:center;font-size:28px;margin:8px 0 16px}.label{font-weight:bold;margin:12px 0 6px}.box{background:#e8e8e8;border-radius:10px;padding:12px;margin-bottom:12px}.row{display:flex;align-items:center;justify-content:space-between;gap:8px;margin:8px 0}.row label{flex:1}.row input{width:90px;padding:8px;font-size:16px}.btn{padding:10px 14px;border:1px solid #ccc;background:#fff;border-radius:8px;font-size:16px;cursor:pointer}.btn.sel{background:#111;color:#fff}.btn-green{background:#22c55e;color:#fff;border-color:#22c55e}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.full{width:100%;padding:12px;font-size:18px;border-radius:10px}</style></head><body><h1>DSP TUNER</h1><div class="label">CROSSOVER</div><div class="box"><b>CHANNEL A</b><div class="row"><label>Filter 1 Hi-Pass</label><input id="a_hp" type="number" value="30"><span>Hz</span></div><div class="row"><label>Filter 2 Low-Pass</label><input id="a_lp" type="number" value="500"><span>Hz</span></div><div class="row"><label>Delay</label><input id="a_dly" type="number" step="0.01" value="0"><span>ms</span></div><div class="row"><label>Mode</label><button class="btn" onclick="setMode('A','STEREO',this)">STEREO</button><button class="btn sel" onclick="setMode('A','MONO',this)">MONO</button></div></div><div class="box"><b>CHANNEL B</b><div class="row"><label>Filter 1 Hi-Pass</label><input id="b_hp" type="number" value="500"><span>Hz</span></div><div class="row"><label>Filter 2 Low-Pass</label><input id="b_lp" type="number" value="20000"><span>Hz</span></div><div class="row"><label>Delay</label><input id="b_dly" type="number" step="0.01" value="0.15"><span>ms</span></div><div class="row"><label>Mode</label><button class="btn sel" onclick="setMode('B','STEREO',this)">STEREO</button><button class="btn" onclick="setMode('B','MONO',this)">MONO</button></div></div><button class="btn full" onclick="applyCrossover()">APPLY</button><div class="label">EQ - 5 BANDS</div><div class="box"><div class="row"><label>60 Hz</label><input id="eq0" type="number" value="0"><span>dB</span></div><div class="row"><label>250 Hz</label><input id="eq1" type="number" value="0"><span>dB</span></div><div class="row"><label>1 kHz</label><input id="eq2" type="number" value="0"><span>dB</span></div><div class="row"><label>4 kHz</label><input id="eq3" type="number" value="0"><span>dB</span></div><div class="row"><label>10 kHz</label><input id="eq4" type="number" value="0"><span>dB</span></div></div><div class="label">USER PRESET</div><div class="grid"><button class="btn" onclick="loadUser(1,this)">USER 1</button><button class="btn" onclick="loadUser(2,this)">USER 2</button><button class="btn" onclick="loadUser(3,this)">USER 3</button><button class="btn" onclick="loadUser(4,this)">USER 4</button></div><br><div style="display:flex;gap:10px"><button class="btn full" onclick="doDefault()">DEFAULT</button><button class="btn full btn-green" onclick="doSave()">SAVE</button></div><button class="btn full" onclick="location.href='/update'" style="margin-top:10px">UPDATE</button><p id="msg" style="text-align:center"></p><script>let modeA='MONO',modeB='STEREO',selUser=1;document.querySelectorAll('input[type=number]').forEach(el=>{el.addEventListener('focus',()=>el.select());el.addEventListener('blur',()=>{if(el.value==='')el.value='0';});});function setMode(ch,m,btn){if(ch==='A')modeA=m;else modeB=m;btn.parentElement.querySelectorAll('.btn').forEach(b=>b.classList.remove('sel'));btn.classList.add('sel');}function collect(){return{a_hp:+document.getElementById('a_hp').value,a_lp:+document.getElementById('a_lp').value,a_dly:+document.getElementById('a_dly').value,b_hp:+document.getElementById('b_hp').value,b_lp:+document.getElementById('b_lp').value,b_dly:+document.getElementById('b_dly').value,eq:[+document.getElementById('eq0').value,+document.getElementById('eq1').value,+document.getElementById('eq2').value,+document.getElementById('eq3').value,+document.getElementById('eq4').value],modeA,modeB,user:selUser};}async function applyCrossover(){let d=collect();let r=await fetch('/api/apply',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});document.getElementById('msg').textContent=await r.text();}async function doSave(){let d=collect();let r=await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});document.getElementById('msg').textContent=await r.text();}async function doDefault(){let r=await fetch('/api/default',{method:'POST'});document.getElementById('msg').textContent=await r.text();location.reload();}async function loadUser(n,btn){selUser=n;document.querySelectorAll('.grid .btn').forEach(b=>b.classList.remove('sel'));btn.classList.add('sel');let r=await fetch('/api/load?user='+n);let j=await r.json();document.getElementById('a_hp').value=j.a_hp;document.getElementById('a_lp').value=j.a_lp;document.getElementById('b_hp').value=j.b_hp;document.getElementById('b_lp').value=j.b_lp;document.getElementById('eq0').value=j.eq[0];document.getElementById('eq1').value=j.eq[1];document.getElementById('eq2').value=j.eq[2];document.getElementById('eq3').value=j.eq[3];document.getElementById('eq4').value=j.eq[4];}</script></body></html>
)HTML";
const char* UPDATE_PAGE = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>UPDATE</title><style>body{margin:0;font-family:Arial,sans-serif;background:#fff;color:#111;padding:16px;max-width:480px;margin:auto}h1{text-align:center}.box{background:#e8e8e8;border-radius:10px;padding:12px;margin:12px 0}.btn{width:100%;padding:12px;font-size:18px;border-radius:10px;border:1px solid #ccc;background:#fff;cursor:pointer}.btn-green{background:#22c55e;color:#fff;border-color:#22c55e}#bar{width:100%;background:#eee;border-radius:8px;overflow:hidden;margin:10px 0;display:none}#fill{height:22px;width:0%;background:#22c55e}</style></head><body><h1>UPDATE</h1><div class="box"><p>1. โหลดไฟล์ firmware.bin จาก GitHub มาก่อน</p><p>2. กด Choose File แล้วเลือกไฟล์นั้น</p><p>3. กด UPLOAD แล้วรอจนครบ 100% ห้ามปิดหน้า</p></div><form id="f"><input type="file" id="file" name="update" accept=".bin" required style="width:100%;padding:10px"><br><br><button class="btn btn-green" type="submit">UPLOAD</button></form><div id="bar"><div id="fill"></div></div><p id="msg" style="text-align:center"></p><button class="btn" onclick="location.href='/'" style="margin-top:10px">BACK</button><script>document.getElementById('f').onsubmit=function(e){e.preventDefault();var file=document.getElementById('file').files[0];if(!file){alert('เลือกไฟล์ก่อน');return;}var fd=new FormData();fd.append('update',file);var xhr=new XMLHttpRequest();xhr.open('POST','/doUpdate',true);document.getElementById('bar').style.display='block';xhr.upload.onprogress=function(ev){if(ev.lengthComputable){var p=Math.round(ev.loaded/ev.total*100);document.getElementById('fill').style.width=p+'%';document.getElementById('msg').textContent='กำลังอัพเดท... '+p+'%';}};xhr.onload=function(){document.getElementById('msg').textContent=xhr.responseText;if(xhr.status==200){setTimeout(function(){location.href='/';},3000);}};xhr.onerror=function(){document.getElementById('msg').textContent='อัพเดทไม่สำเร็จ ลองใหม่';};xhr.send(fd);}</script></body></html>
)HTML";
float getVal(String &body,const char* key){int idx=body.indexOf(key);if(idx<0)return 0;int c=body.indexOf(':',idx);return body.substring(c+1).toFloat();}
void parseBody(String body){
if(body.length()<10)return;
current.chA_hp=constrain(getVal(body,"\"a_hp\""),20,1000);current.chA_lp=constrain(getVal(body,"\"a_lp\""),100,5000);
current.chB_hp=constrain(getVal(body,"\"b_hp\""),100,5000);current.chB_lp=constrain(getVal(body,"\"b_lp\""),1000,20000);
current.chA_delay=constrain(getVal(body,"\"a_dly\""),0,5);current.chB_delay=constrain(getVal(body,"\"b_dly\""),0,5);
int eqIdx=body.indexOf("\"eq\"");if(eqIdx>=0){int br=body.indexOf('[',eqIdx);int br2=body.indexOf(']',br);String arr=body.substring(br+1,br2);for(int i=0;i<5;i++){int cm=arr.indexOf(',');String tok=(cm>=0)?arr.substring(0,cm):arr;current.webEQ[i]=constrain(tok.toFloat(),-12,12);if(cm<0)break;arr=arr.substring(cm+1);}}
if(body.indexOf("\"modeA\":\"MONO\"")>=0)current.chA_mode=1;else if(body.indexOf("\"modeA\"")>=0)current.chA_mode=0;
if(body.indexOf("\"modeB\":\"MONO\"")>=0)current.chB_mode=1;else if(body.indexOf("\"modeB\"")>=0)current.chB_mode=0;
int uIdx=body.indexOf("\"user\"");if(uIdx>=0){int c=body.indexOf(':',uIdx);selectedUser=constrain((int)body.substring(c+1).toFloat(),1,4);}
applyAll();}
void setupWeb(){
server.on("/",[](){server.send(200,"text/html",HTML_PAGE);});
server.on("/update",[](){isUpdating=true;apLastSeen=millis();server.send(200,"text/html",UPDATE_PAGE);});
server.on("/doUpdate",HTTP_POST,[](){
bool ok=!Update.hasError();
server.send(200,"text/plain",ok?"อัพเดทสำเร็จ กำลังรีสตาร์ท...":"อัพเดทไม่สำเร็จ ลองใหม่");
delay(500);if(ok)ESP.restart();},[](){
HTTPUpload& up=server.upload();
if(up.status==UPLOAD_FILE_START){isUpdating=true;Serial.printf("UPDATE START %s\n",up.filename.c_str());if(!Update.begin(UPDATE_SIZE_UNKNOWN)){Serial.println("Update begin fail");}}
else if(up.status==UPLOAD_FILE_WRITE){if(Update.write(up.buf,up.currentSize)!=up.currentSize){Serial.println("Update write fail");}}
else if(up.status==UPLOAD_FILE_END){if(Update.end(true)){Serial.println("UPDATE DONE");}else{Serial.println("UPDATE END fail");}}});
server.on("/api/apply",HTTP_POST,[](){String b=server.arg("plain");parseBody(b);server.send(200,"text/plain","Applied - not saved yet");});
server.on("/api/save",HTTP_POST,[](){String b=server.arg("plain");parseBody(b);saveUserPreset(selectedUser);saveSettings();server.send(200,"text/plain",String("Saved to USER ")+selectedUser);});
server.on("/api/default",HTTP_POST,[](){current=defaultPreset;applyAll();saveUserPreset(selectedUser);saveSettings();server.send(200,"text/plain","Reset to DEFAULT Flat");});
server.on("/api/load",HTTP_GET,[](){int u=server.arg("user").toInt();if(u<1||u>4)u=1;loadUserPreset(u);String j=String("{\"a_hp\":")+current.chA_hp+",\"a_lp\":"+current.chA_lp+",\"b_hp\":"+current.chB_hp+",\"b_lp\":"+current.chB_lp+",\"eq\":["+current.webEQ[0]+","+current.webEQ[1]+","+current.webEQ[2]+","+current.webEQ[3]+","+current.webEQ[4]+"]}";server.send(200,"application/json",j);});
server.begin();}
void setupI2S(){
auto configIn=i2sStreamIn.defaultConfig(RX_MODE);configIn.copyFrom(info);configIn.rx_tx_mode=RX_MODE;configIn.port_no=0;configIn.pin_bck=I2S_IN_BCK;configIn.pin_ws=I2S_IN_LRCK;configIn.pin_data=I2S_IN_DATA;configIn.is_master=false;configIn.buffer_count=16;configIn.buffer_size=512;i2sStreamIn.begin(configIn);
auto configOut=i2sStreamOut.defaultConfig(TX_MODE);configOut.copyFrom(info);configOut.rx_tx_mode=TX_MODE;configOut.port_no=1;configOut.pin_bck=I2S_BCK_PIN;configOut.pin_ws=I2S_LRCK_PIN;configOut.pin_data=I2S_DATA1_PIN;configOut.buffer_count=12;configOut.buffer_size=512;i2sStreamOut.begin(configOut);}
void flushTank(){uint8_t tmp[512];while(i2sStreamIn.available()>0){int n=i2sStreamIn.readBytes(tmp,sizeof(tmp));if(n<=0)break;}}
void processAudio(){
uint8_t buf[1024];int n=i2sStreamIn.available();if(n<=0)return;if(n>1024)n=1024;n=n/4*4;int read=i2sStreamIn.readBytes(buf,n);if(read<=0)return;
int16_t* s=(int16_t*)buf;int frames=read/4;
for(int i=0;i<frames;i++){
float l=(float)s[i*2];float r=(float)s[i*2+1];
for(int e=0;e<3;e++)remoteEQ[e].process(l,r);
for(int e=0;e<5;e++)webEQ[e].process(l,r);
chB_HP1.process(l,r);chB_LP1.process(l,r);
if(current.chB_slope>=24){chB_HP2.process(l,r);chB_LP2.process(l,r);}
float vol=targetMuted?0:fadeVolume;
l*=vol;r*=vol;
if(l>32767)l=32767;if(l<-32768)l=-32768;if(r>32767)r=32767;if(r<-32768)r=-32768;
s[i*2]=(int16_t)l;s[i*2+1]=(int16_t)r;}
i2sStreamOut.write(buf,read);}
void setup(){Serial.begin(115200);delay(1000);setDefaultPreset();loadSettings();setupI2S();updateAllCoeffs();initWifiAndEspNow();setupWeb();lastAudioTime=millis();Serial.println("S3 DSP Ready");}
void loop(){
server.handleClient();checkApTimeout();int avail=i2sStreamIn.available();unsigned long now=millis();
if(avail>200){lastAudioTime=now;if(gapMuted){gapMuted=false;isFadingUp=true;fadeStartTime=now;fadeVolume=0;}}
if(!gapMuted&&(now-lastAudioTime>100)){delay(10);flushTank();gapMuted=true;isFadingUp=false;}
if(isFadingUp){unsigned long el=now-fadeStartTime;if(el>=30){fadeVolume=targetMuted?0:targetVolume;isFadingUp=false;}else{float p=(float)el/30.0f;fadeVolume=(targetMuted?0:targetVolume)*p;}}
if(!gapMuted)processAudio();else delay(1);
if(volDirty){volDirty=false;}
bool needUpdate=false;
if(fabsf(targetBass-curBass)>0.01){curBass+=(targetBass>curBass?0.05f:-0.05f);needUpdate=true;}
if(fabsf(targetMid-curMid)>0.01){curMid+=(targetMid>curMid?0.05f:-0.05f);needUpdate=true;}
if(fabsf(targetTreble-curTreble)>0.01){curTreble+=(targetTreble>curTreble?0.05f:-0.05f);needUpdate=true;}
if(needUpdate||eqDirty){eqDirty=false;float sr=(float)info.sample_rate;remoteEQ[0].set(Biquad::LOW_SHELF,125,0.7f,curBass,sr);remoteEQ[1].set(Biquad::PEAKING,1000,0.9f,curMid,sr);remoteEQ[2].set(Biquad::HIGH_SHELF,6500,0.75f,curTreble,sr);}
if(nvsDirty){nvsDirty=false;lastNvsChange=now;pendingNvs=true;}
if(pendingNvs&&(now-lastNvsChange>=4500)){pendingNvs=false;saveSettings();}
}

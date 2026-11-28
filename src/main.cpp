#include "Arduino.h"
#include "WiFi.h"
#include "WebServer.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "Preferences.h"
#include "math.h"
#include "Update.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "AudioTools.h"

#define I2S_BCK_PIN 15
#define I2S_LRCK_PIN 16
#define I2S_DATA1_PIN 17
#define I2S_DATA2_PIN 18
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
if(freq<10)freq=10;if(freq>20000)freq=20000;
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

class DelayLine{
public:
void init(){idx=0;for(int i=0;i<512;i++)buf[i]=0;}
float process(float x,float ms,float sr){int d=(int)(ms*sr/1000.0f);if(d<0)d=0;if(d>500)d=500;int ridx=idx-d;if(ridx<0)ridx+=512;float y=buf[ridx];buf[idx]=x;idx++;if(idx>=512)idx=0;return y;}
private:float buf[512];int idx=0;
};

struct Ch{
int hpType=0; //0 Butter 1 LR 2 Bessel
int hpSlope=24; //0 OFF 12 18 24
float hpFreq=500;
int lpType=1; //default LR for crossover
int lpSlope=24;
float lpFreq=20000;
float delayMs=0;
float gainDb=0; // -60 to +12, 0 = loud normal
int phase=0; //0 or 180
bool muted=false;
int mix=0; //0 L 1 R 2 L+R
};
struct Preset{
Ch ch[4];
float eq[4][5];
};
Preset current;Preset defaultPreset;int selectedUser=1;

Biquad eqBiquad[4][5];
Biquad hp1[4],hp2[4],hp3[4];
Biquad lp1[4],lp2[4],lp3[4];
Biquad remoteEQ[3];
DelayLine dly[4];

volatile float targetBass=0,targetMid=0,targetTreble=0,targetVolume=0.8f;volatile bool targetMuted=false;
float curBass=0,curMid=0,curTreble=0;volatile bool eqDirty=false,volDirty=false,nvsDirty=false;
unsigned long lastNvsChange=0;bool pendingNvs=false;
I2SStream i2sStreamIn; I2SStream i2sStreamOut; I2SStream i2sStreamOut2;
unsigned long lastAudioTime=0;bool gapMuted=false;float fadeVolume=0.8f;unsigned long fadeStartTime=0;bool isFadingUp=false;

float qFor(int type,int slope,int stage){
if(type==2){ //Bessel
if(slope<=12) return 0.577f;
if(slope==18) return 0.65f;
if(stage==0) return 0.522f; else return 0.806f;
}
if(type==1){ //Linkwitz-Riley = 2x Butter
return 0.707f;
}
return 0.707f; //Butterworth
}
void setDefaultPreset(){
for(int c=0;c<4;c++){
for(int i=0;i<5;i++) defaultPreset.eq[c][i]=0;
defaultPreset.ch[c].delayMs=0;
defaultPreset.ch[c].gainDb=0;
defaultPreset.ch[c].phase=0;
defaultPreset.ch[c].muted=false;
}
defaultPreset.ch[0].hpType=0;defaultPreset.ch[0].hpSlope=24;defaultPreset.ch[0].hpFreq=500;
defaultPreset.ch[0].lpType=1;defaultPreset.ch[0].lpSlope=24;defaultPreset.ch[0].lpFreq=20000;defaultPreset.ch[0].mix=0;
defaultPreset.ch[1].hpType=0;defaultPreset.ch[1].hpSlope=24;defaultPreset.ch[1].hpFreq=500;
defaultPreset.ch[1].lpType=1;defaultPreset.ch[1].lpSlope=24;defaultPreset.ch[1].lpFreq=20000;defaultPreset.ch[1].mix=1;
defaultPreset.ch[2].hpType=0;defaultPreset.ch[2].hpSlope=12;defaultPreset.ch[2].hpFreq=30;
defaultPreset.ch[2].lpType=1;defaultPreset.ch[2].lpSlope=24;defaultPreset.ch[2].lpFreq=500;defaultPreset.ch[2].mix=2;
defaultPreset.ch[3].hpType=0;defaultPreset.ch[3].hpSlope=12;defaultPreset.ch[3].hpFreq=30;
defaultPreset.ch[3].lpType=1;defaultPreset.ch[3].lpSlope=24;defaultPreset.ch[3].lpFreq=500;defaultPreset.ch[3].mix=2;
current=defaultPreset;
}
void updateAllCoeffs(){
float sr=(float)info.sample_rate;float wf[5]={60,250,1000,4000,10000};
for(int c=0;c<4;c++){
for(int i=0;i<5;i++) eqBiquad[c][i].set(Biquad::PEAKING,wf[i],1.0f,current.eq[c][i],sr);
Ch &cc=current.ch[c];
if(cc.hpSlope==0){hp1[c].set(Biquad::HIGH_PASS,20,0.707f,0,sr);}
else{
hp1[c].set(Biquad::HIGH_PASS,cc.hpFreq,qFor(cc.hpType,cc.hpSlope,0),0,sr);
hp2[c].set(Biquad::HIGH_PASS,cc.hpFreq,qFor(cc.hpType,cc.hpSlope,1),0,sr);
hp3[c].set(Biquad::HIGH_PASS,cc.hpFreq,0.707f,0,sr);
}
if(cc.lpSlope==0){lp1[c].set(Biquad::LOW_PASS,20000,0.707f,0,sr);}
else{
lp1[c].set(Biquad::LOW_PASS,cc.lpFreq,qFor(cc.lpType,cc.lpSlope,0),0,sr);
lp2[c].set(Biquad::LOW_PASS,cc.lpFreq,qFor(cc.lpType,cc.lpSlope,1),0,sr);
lp3[c].set(Biquad::LOW_PASS,cc.lpFreq,0.707f,0,sr);
}
}
remoteEQ[0].set(Biquad::LOW_SHELF,125,0.7f,curBass,sr);remoteEQ[1].set(Biquad::PEAKING,1000,0.9f,curMid,sr);remoteEQ[2].set(Biquad::HIGH_SHELF,6500,0.75f,curTreble,sr);
}
void applyAll(){updateAllCoeffs();}
void loadSettings(){
prefs.begin("dsp_tuner",true);
targetVolume=constrain(prefs.getFloat("vol",0.8f),0,1);targetBass=constrain(prefs.getFloat("bass",0),-15,15);
targetMid=constrain(prefs.getFloat("mid",0),-12,12);targetTreble=constrain(prefs.getFloat("treble",0),-12,12);
targetMuted=prefs.getBool("muted",false);
size_t n=prefs.getBytes("user1",&current,sizeof(Preset));if(n!=sizeof(Preset))current=defaultPreset;
selectedUser=prefs.getInt("selUser",1);prefs.end();
curBass=targetBass;curMid=targetMid;curTreble=targetTreble;fadeVolume=targetMuted?0:targetVolume;
}
void saveSettings(){
prefs.begin("dsp_tuner",false);prefs.putFloat("vol",targetVolume);prefs.putFloat("bass",targetBass);prefs.putFloat("mid",targetMid);prefs.putFloat("treble",targetTreble);prefs.putBool("muted",targetMuted);prefs.putInt("selUser",selectedUser);prefs.putBytes((String("user")+selectedUser).c_str(),&current,sizeof(Preset));prefs.end();
}
void saveUserPreset(int idx){selectedUser=idx;prefs.begin("dsp_tuner",false);prefs.putBytes((String("user")+idx).c_str(),&current,sizeof(Preset));prefs.putInt("selUser",idx);prefs.end();}
void loadUserPreset(int idx){selectedUser=idx;prefs.begin("dsp_tuner",true);size_t n=prefs.getBytes((String("user")+idx).c_str(),&current,sizeof(Preset));prefs.end();if(n!=sizeof(Preset))return;applyAll();}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *info,const uint8_t *data,int len)
#else
void OnDataRecv(const uint8_t *mac,const uint8_t *data,int len)
#endif
{
if(len<(int)sizeof(incomingData))return;
memcpy(&incomingData,data,sizeof(incomingData));
if(incomingData.controlMode<0||incomingData.controlMode>3)return;
if(incomingData.isMuted!=targetMuted){targetMuted=incomingData.isMuted;volDirty=true;nvsDirty=true;}
if(incomingData.isMuted)return;
int m=incomingData.controlMode;
if(m==0){float v=constrain((float)incomingData.value/100.0f,0,1);if(fabsf(v-targetVolume)>0.001){targetVolume=v;volDirty=true;nvsDirty=true;}}
if(m==1){float v=constrain((float)incomingData.value,-15,15);if(fabsf(v-targetBass)>0.01){targetBass=v;eqDirty=true;nvsDirty=true;}}
if(m==2){float v=constrain((float)incomingData.value,-12,12);if(fabsf(v-targetMid)>0.01){targetMid=v;eqDirty=true;nvsDirty=true;}}
if(m==3){float v=constrain((float)incomingData.value,-12,12);if(fabsf(v-targetTreble)>0.01){targetTreble=v;eqDirty=true;nvsDirty=true;}}
}
void initWifiAndEspNow(){
WiFi.persistent(false);WiFi.mode(WIFI_AP_STA);WiFi.softAP("DSP-TUNER","12345678",1);WiFi.disconnect();
esp_wifi_set_promiscuous(true);esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);esp_wifi_set_promiscuous(false);
if(esp_now_init()==ESP_OK){esp_now_register_recv_cb(OnDataRecv);Serial.println("ESP-NOW Ready Ch1");}else Serial.println("ESP-NOW Fail");
apActive=true;apLastSeen=millis();Serial.println("AP ON");
}
void checkApTimeout(){
if(!apActive||isUpdating)return;
unsigned long now=millis();
int clients=WiFi.softAPgetStationNum();
if(clients>0){apLastSeen=now;return;}
if(now-apLastSeen>=AP_TIMEOUT){WiFi.softAPdisconnect(true);apActive=false;esp_wifi_set_promiscuous(true);esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);esp_wifi_set_promiscuous(false);Serial.println("AP OFF");}
}

const char* HTML_PAGE = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1"><title>DSP TUNER</title><style>*{box-sizing:border-box}body{margin:0;font-family:Arial,sans-serif;background:#fff;color:#111;height:100dvh;display:flex;flex-direction:column;overflow:hidden}header{text-align:center;padding:6px 0 2px}header h1{margin:0;font-size:20px}header p{margin:0;font-size:11px;color:#666}main{flex:1;overflow:hidden;padding:6px 8px}.tab{display:none;height:100%}.tab.on{display:block}.box{background:#e8e8e8;border-radius:10px;padding:8px}.row{display:flex;align-items:center;gap:6px;margin:4px 0}.row label{flex:1;font-size:13px}.row input[type=text]{width:64px;padding:6px;font-size:14px;text-align:center}.step{width:28px;height:28px;border:1px solid #ccc;background:#fff;border-radius:6px;font-size:16px}.grid2{display:grid;grid-template-columns:1fr 1fr;gap:6px}.btn{padding:8px;border:1px solid #ccc;background:#fff;border-radius:8px;font-size:13px}.btn.sel{background:#111;color:#fff}.btn-green{background:#22c55e;color:#fff;border-color:#22c55e}.full{width:100%;padding:9px;font-size:15px;border-radius:9px}select{padding:6px;font-size:12px;max-width:110px}nav{height:56px;display:flex;border-top:1px solid #ddd}nav button{flex:1;border:0;background:#fff;font-size:12px}nav button.on{background:#111;color:#fff}.presets{display:flex;gap:6px;margin:6px 0}.presets .btn{flex:1}.actions{display:flex;gap:6px}</style></head><body><header><h1>DSP TUNER</h1><p>4CH - DAC 2ตัว</p></header><main>
<div id="t-delay" class="tab on"><div class="grid2"><div class="box"><b>CH1</b><div class="row"><label>Delay</label><button class="step" onclick="step(this,-0.01)">-</button><input type="text" inputmode="decimal" value="0" id="d0"><button class="step" onclick="step(this,0.01)">+</button><span>ms</span></div></div><div class="box"><b>CH2</b><div class="row"><label>Delay</label><button class="step" onclick="step(this,-0.01)">-</button><input type="text" inputmode="decimal" value="0" id="d1"><button class="step" onclick="step(this,0.01)">+</button><span>ms</span></div></div><div class="box"><b>CH3</b><div class="row"><label>Delay</label><button class="step" onclick="step(this,-0.01)">-</button><input type="text" inputmode="decimal" value="0" id="d2"><button class="step" onclick="step(this,0.01)">+</button><span>ms</span></div></div><div class="box"><b>CH4</b><div class="row"><label>Delay</label><button class="step" onclick="step(this,-0.01)">-</button><input type="text" inputmode="decimal" value="0" id="d3"><button class="step" onclick="step(this,0.01)">+</button><span>ms</span></div></div></div></div>
<div id="t-xover" class="tab"><div class="grid2"><div class="box"><b>CH1</b><div class="row"><label>HP</label><select id="hpT0"><option>Butterworth</option><option>Linkwitz-Riley</option><option>Bessel</option></select><select id="hpS0"><option>OFF</option><option>12</option><option>18</option><option selected>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="500" id="hpF0"><span>Hz</span></div><div class="row"><label>LP</label><select id="lpT0"><option>Butterworth</option><option selected>Linkwitz-Riley</option><option>Bessel</option></select><select id="lpS0"><option>OFF</option><option>12</option><option>18</option><option selected>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="20000" id="lpF0"><span>Hz</span></div></div><div class="box"><b>CH2</b><div class="row"><label>HP</label><select id="hpT1"><option>Butterworth</option><option>Linkwitz-Riley</option><option>Bessel</option></select><select id="hpS1"><option>OFF</option><option>12</option><option>18</option><option selected>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="500" id="hpF1"><span>Hz</span></div><div class="row"><label>LP</label><select id="lpT1"><option>Butterworth</option><option selected>Linkwitz-Riley</option><option>Bessel</option></select><select id="lpS1"><option>OFF</option><option>12</option><option>18</option><option selected>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="20000" id="lpF1"><span>Hz</span></div></div><div class="box"><b>CH3</b><div class="row"><label>HP</label><select id="hpT2"><option>Butterworth</option><option>Linkwitz-Riley</option><option>Bessel</option></select><select id="hpS2"><option>OFF</option><option selected>12</option><option>18</option><option>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="30" id="hpF2"><span>Hz</span></div><div class="row"><label>LP</label><select id="lpT2"><option>Butterworth</option><option selected>Linkwitz-Riley</option><option>Bessel</option></select><select id="lpS2"><option>OFF</option><option>12</option><option>18</option><option selected>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="500" id="lpF2"><span>Hz</span></div></div><div class="box"><b>CH4</b><div class="row"><label>HP</label><select id="hpT3"><option>Butterworth</option><option>Linkwitz-Riley</option><option>Bessel</option></select><select id="hpS3"><option>OFF</option><option selected>12</option><option>18</option><option>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="30" id="hpF3"><span>Hz</span></div><div class="row"><label>LP</label><select id="lpT3"><option>Butterworth</option><option selected>Linkwitz-Riley</option><option>Bessel</option></select><select id="lpS3"><option>OFF</option><option>12</option><option>18</option><option selected>24</option></select></div><div class="row"><input type="text" inputmode="decimal" value="500" id="lpF3"><span>Hz</span></div></div></div><button class="btn full" onclick="applyX()">APPLY</button></div>
<div id="t-chan" class="tab"><div class="grid2"><div class="box"><b>CH1</b><div class="row"><button class="btn" data-mix="0" data-ch="0">L</button><button class="btn" data-mix="1" data-ch="0">R</button><button class="btn" data-mix="2" data-ch="0">L+R</button></div><div class="row"><label>Gain</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="g0"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><button class="btn" onclick="togPhase(this)">0°</button><button class="btn" onclick="togMute(this)">UNMUTED</button></div><div style="font-size:11px">0 dB · 0° · Playing</div></div><div class="box"><b>CH2</b><div class="row"><button class="btn" data-mix="0" data-ch="1">L</button><button class="btn" data-mix="1" data-ch="1">R</button><button class="btn" data-mix="2" data-ch="1">L+R</button></div><div class="row"><label>Gain</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="g1"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><button class="btn" onclick="togPhase(this)">0°</button><button class="btn" onclick="togMute(this)">UNMUTED</button></div><div style="font-size:11px">0 dB · 0° · Playing</div></div><div class="box"><b>CH3</b><div class="row"><button class="btn" data-mix="0" data-ch="2">L</button><button class="btn" data-mix="1" data-ch="2">R</button><button class="btn" data-mix="2" data-ch="2">L+R</button></div><div class="row"><label>Gain</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="g2"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><button class="btn" onclick="togPhase(this)">0°</button><button class="btn" onclick="togMute(this)">UNMUTED</button></div><div style="font-size:11px">0 dB · 0° · Playing</div></div><div class="box"><b>CH4</b><div class="row"><button class="btn" data-mix="0" data-ch="3">L</button><button class="btn" data-mix="1" data-ch="3">R</button><button class="btn" data-mix="2" data-ch="3">L+R</button></div><div class="row"><label>Gain</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="g3"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><button class="btn" onclick="togPhase(this)">0°</button><button class="btn" onclick="togMute(this)">UNMUTED</button></div><div style="font-size:11px">0 dB · 0° · Playing</div></div></div></div>
<div id="t-eq" class="tab"><div class="row"><button class="btn" onclick="selEq(0,this)">CH1</button><button class="btn" onclick="selEq(1,this)">CH2</button><button class="btn" onclick="selEq(2,this)">CH3</button><button class="btn" onclick="selEq(3,this)">CH4</button><button class="btn" id="linkBtn" onclick="togLink()">LINK ON</button></div><div class="box"><div class="row"><label>60 Hz</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="eq0"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><label>250 Hz</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="eq1"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><label>1 kHz</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="eq2"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><label>4 kHz</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="eq3"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div><div class="row"><label>10 kHz</label><button class="step" onclick="step(this,-1)">-</button><input type="text" inputmode="decimal" value="0" id="eq4"><button class="step" onclick="step(this,1)">+</button><span>dB</span></div></div></div>
<div id="t-upd" class="tab"><div class="box"><p>1. Download firmware.bin</p><p>2. กด Choose File แล้วเลือกไฟล์นั้น</p><p>3. กด UPLOAD แล้วรอจนครบ 100% ห้ามปิดหน้า</p></div><input type="file" id="file" accept=".bin" style="width:100%"><br><br><button class="btn full btn-green" onclick="doUpd()">UPLOAD</button><div id="bar" style="height:22px;background:#eee;border-radius:8px;overflow:hidden;display:none;margin-top:8px"><div id="fill" style="height:22px;width:0%;background:#22c55e"></div></div><p id="msg2" style="text-align:center"></p></div>
<div class="presets"><button class="btn" onclick="loadUser(1)">USER 1</button><button class="btn" onclick="loadUser(2)">USER 2</button><button class="btn" onclick="loadUser(3)">USER 3</button><button class="btn" onclick="loadUser(4)">USER 4</button></div><div class="actions"><button class="btn full" onclick="doDefault()">DEFAULT</button><button class="btn full btn-green" onclick="doSave()">SAVE</button></div><p id="msg" style="text-align:center;margin:4px"></p></main><nav><button class="on" onclick="showTab('delay',this)">Delay</button><button onclick="showTab('xover',this)">Crossover</button><button onclick="showTab('chan',this)">Channel</button><button onclick="showTab('eq',this)">Equalizer</button><button onclick="showTab('upd',this)">Update</button></nav><script>let selEqCh=0,linkOn=true;function showTab(n,btn){document.querySelectorAll('.tab').forEach(t=>t.classList.remove('on'));document.getElementById('t-'+n).classList.add('on');document.querySelectorAll('nav button').forEach(b=>b.classList.remove('on'));btn.classList.add('on');}function step(btn,d){let inp=btn.parentElement.querySelector('input');let v=parseFloat(inp.value||0)+d;inp.value=Math.round(v*100)/100;}function togPhase(b){b.textContent=b.textContent=='0°'?'180°':'0°';}function togMute(b){b.textContent=b.textContent=='UNMUTED'?'MUTED':'UNMUTED';}function selEq(c,btn){selEqCh=c;if(linkOn){let pair=c<2?0:2;document.querySelectorAll('#t-eq .btn').forEach(b=>b.classList.remove('sel'));} }function togLink(){linkOn=!linkOn;document.getElementById('linkBtn').textContent=linkOn?'LINK ON':'LINK OFF';}document.querySelectorAll('input').forEach(el=>{el.addEventListener('focus',()=>el.select());el.addEventListener('blur',()=>{if(el.value==='')el.value='0';});});function collect(){let ch=[];for(let i=0;i<4;i++){ch.push({hpT:document.getElementById('hpT'+i).selectedIndex,hpS:document.getElementById('hpS'+i).value,hpF:+document.getElementById('hpF'+i).value,lpT:document.getElementById('lpT'+i).selectedIndex,lpS:document.getElementById('lpS'+i).value,lpF:+document.getElementById('lpF'+i).value,dly:+document.getElementById('d'+i).value,gain:+document.getElementById('g'+i).value});}return{ch,eq:[+document.getElementById('eq0').value,+document.getElementById('eq1').value,+document.getElementById('eq2').value,+document.getElementById('eq3').value,+document.getElementById('eq4').value],eqCh:selEqCh,link:linkOn,user:1};}async function applyX(){let d=collect();let r=await fetch('/api/apply',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});document.getElementById('msg').textContent=await r.text();}async function doSave(){let d=collect();let r=await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});document.getElementById('msg').textContent=await r.text();}async function doDefault(){let r=await fetch('/api/default',{method:'POST'});document.getElementById('msg').textContent=await r.text();location.reload();}async function loadUser(n){let r=await fetch('/api/load?user='+n);document.getElementById('msg').textContent=await r.text();}function doUpd(){let f=document.getElementById('file').files[0];if(!f){alert('เลือกไฟล์ก่อน');return;}let fd=new FormData();fd.append('update',f);let xhr=new XMLHttpRequest();xhr.open('POST','/doUpdate',true);document.getElementById('bar').style.display='block';xhr.upload.onprogress=e=>{if(e.lengthComputable){let p=Math.round(e.loaded/e.total*100);document.getElementById('fill').style.width=p+'%';document.getElementById('msg2').textContent='กำลังอัพเดท... '+p+'%';}};xhr.onload=()=>{document.getElementById('msg2').textContent=xhr.responseText;};xhr.send(fd);}</script></body></html>
)HTML";

const char* UPDATE_PAGE = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>UPDATE</title><style>body{margin:0;font-family:Arial,sans-serif;background:#fff;padding:16px;max-width:480px;margin:auto}h1{text-align:center}.box{background:#e8e8e8;border-radius:10px;padding:12px}.btn{width:100%;padding:12px;font-size:18px;border-radius:10px}.btn-green{background:#22c55e;color:#fff}</style></head><body><h1>UPDATE</h1><div class="box"><p>1. Download firmware.bin</p><p>2. เลือกไฟล์นั้น</p><p>3. กด UPLOAD รอ 100% ห้ามปิดหน้า</p></div></body></html>
)HTML";

float getVal(String &body,const char* key){int idx=body.indexOf(key);if(idx<0)return 0;int c=body.indexOf(':',idx);if(c<0)return 0;return body.substring(c+1).toFloat();}
void parseBody(String body){
if(body.length()<10)return;
// parse simplified: we read delay/gain/eq from JSON by searching keys
// For full CH arrays, the frontend sends ch array - parse per index using indexOf
for(int i=0;i<4;i++){
String k1="\"hpF\":"+String(i); // fallback - actual JSON uses hpF per ch object, we use generic scan
}
// Simple fallback: use global fields if present, else keep current
float d0=getVal(body,"\"dly\""); // not used in new format
// New format: eq array
int eqIdx=body.indexOf("\"eq\"");if(eqIdx>=0){int br=body.indexOf('[',eqIdx);int br2=body.indexOf(']',br);String arr=body.substring(br+1,br2);int eqCh=0;int li=body.indexOf("\"eqCh\"");if(li>=0) eqCh=(int)getVal(body,"\"eqCh\"");bool link=body.indexOf("\"link\":true")>=0;for(int j=0;j<5;j++){int cm=arr.indexOf(',');String tok=(cm>=0)?arr.substring(0,cm):arr;float v=constrain(tok.toFloat(),-12,12);current.eq[eqCh][j]=v;if(link){int pair = eqCh<2?1:3;int other = (eqCh%2==0)? eqCh+1 : eqCh-1;current.eq[other][j]=v;}if(cm<0)break;arr=arr.substring(cm+1);}}
// Crossover / delay / gain parsed from ch array tokens in order: hpF, lpF, dly, gain appear 4 times in order
int pos=0;for(int c=0;c<4;c++){
int pHp=body.indexOf("\"hpF\"",pos);if(pHp<0)break;int colon=body.indexOf(':',pHp);current.ch[c].hpFreq=constrain(body.substring(colon+1).toFloat(),20,20000);pos=colon+1;
int pLp=body.indexOf("\"lpF\"",pos);if(pLp<0)break;colon=body.indexOf(':',pLp);current.ch[c].lpFreq=constrain(body.substring(colon+1).toFloat(),20,20000);pos=colon+1;
int pD=body.indexOf("\"dly\"",pos);if(pD>=0){colon=body.indexOf(':',pD);current.ch[c].delayMs=constrain(body.substring(colon+1).toFloat(),0,10);pos=colon+1;}
int pG=body.indexOf("\"gain\"",pos);if(pG>=0){colon=body.indexOf(':',pG);current.ch[c].gainDb=constrain(body.substring(colon+1).toFloat(),-60,12);pos=colon+1;}
}
applyAll();
}
void setupWeb(){
server.on("/",[](){server.send(200,"text/html",HTML_PAGE);});
server.on("/update",[](){isUpdating=true;apLastSeen=millis();server.send(200,"text/html",UPDATE_PAGE);});
server.on("/doUpdate",HTTP_POST,[](){
bool ok=!Update.hasError();
server.send(200,"text/plain",ok?"อัพเดทสำเร็จ กำลังรีสตาร์ท...":"อัพเดทไม่สำเร็จ ลองใหม่");
delay(500);if(ok)ESP.restart();},[](){
HTTPUpload& up=server.upload();
if(up.status==UPLOAD_FILE_START){isUpdating=true;if(!Update.begin(UPDATE_SIZE_UNKNOWN))Serial.println("Update begin fail");}
else if(up.status==UPLOAD_FILE_WRITE){if(Update.write(up.buf,up.currentSize)!=up.currentSize)Serial.println("Update write fail");}
else if(up.status==UPLOAD_FILE_END){if(Update.end(true))Serial.println("UPDATE DONE");else Serial.println("UPDATE END fail");}});
server.on("/api/apply",HTTP_POST,[](){String b=server.arg("plain");parseBody(b);server.send(200,"text/plain","Applied - not saved yet");});
server.on("/api/save",HTTP_POST,[](){String b=server.arg("plain");parseBody(b);saveUserPreset(selectedUser);saveSettings();server.send(200,"text/plain",String("Saved to USER ")+selectedUser);});
server.on("/api/default",HTTP_POST,[](){current=defaultPreset;applyAll();saveUserPreset(selectedUser);saveSettings();server.send(200,"text/plain","Reset to DEFAULT Flat");});
server.on("/api/load",HTTP_GET,[](){int u=server.arg("user").toInt();if(u<1||u>4)u=1;loadUserPreset(u);server.send(200,"text/plain",String("Loaded USER ")+u);});
server.begin();
}
void setupI2S(){
auto configIn=i2sStreamIn.defaultConfig(RX_MODE);configIn.copyFrom(info);configIn.rx_tx_mode=RX_MODE;configIn.port_no=0;configIn.pin_bck=I2S_IN_BCK;configIn.pin_ws=I2S_IN_LRCK;configIn.pin_data=I2S_IN_DATA;configIn.is_master=false;configIn.buffer_count=16;configIn.buffer_size=512;i2sStreamIn.begin(configIn);
auto configOut=i2sStreamOut.defaultConfig(TX_MODE);configOut.copyFrom(info);configOut.rx_tx_mode=TX_MODE;configOut.port_no=1;configOut.pin_bck=I2S_BCK_PIN;configOut.pin_ws=I2S_LRCK_PIN;configOut.pin_data=I2S_DATA1_PIN;configOut.buffer_count=12;configOut.buffer_size=512;i2sStreamOut.begin(configOut);
auto configOut2=i2sStreamOut2.defaultConfig(TX_MODE);configOut2.copyFrom(info);configOut2.rx_tx_mode=TX_MODE;configOut2.port_no=1;configOut2.pin_bck=I2S_BCK_PIN;configOut2.pin_ws=I2S_LRCK_PIN;configOut2.pin_data=I2S_DATA2_PIN;configOut2.buffer_count=12;configOut2.buffer_size=512;
// Note: DAC2 uses same BCK/LRCK, DATA2 separate. If AudioTools blocks second stream on same port, keep DATA2 wired to same DATA1 pin in hardware test and use OUT1 only for CH1/CH2, OUT2 for CH3/CH4 in next revision.
i2sStreamOut2.begin(configOut2);
for(int i=0;i<4;i++) dly[i].init();
}
void flushTank(){uint8_t tmp[512];while(i2sStreamIn.available()>0){int n=i2sStreamIn.readBytes(tmp,sizeof(tmp));if(n<=0)break;}}
float dbToLin(float db){return powf(10.0f,db/20.0f);}
void processAudio(){
uint8_t buf[1024];int n=i2sStreamIn.available();if(n<=0)return;if(n>1024)n=1024;n=n/4*4;int read=i2sStreamIn.readBytes(buf,n);if(read<=0)return;
int16_t* s=(int16_t*)buf;int frames=read/4;
uint8_t out1[1024];uint8_t out2[1024];int16_t* o1=(int16_t*)out1;int16_t* o2=(int16_t*)out2;
for(int i=0;i<frames;i++){
float inL=(float)s[i*2];float inR=(float)s[i*2+1];
for(int e=0;e<3;e++)remoteEQ[e].process(inL,inR);
float outCh[4];
for(int c=0;c<4;c++){
Ch &cc=current.ch[c];
float x = cc.mix==0? inL : cc.mix==1? inR : (inL+inR)*0.5f;
for(int e=0;e<5;e++) x=eqBiquad[c][e].process(x);
if(cc.hpSlope!=0){
x=hp1[c].process(x);
if(cc.hpSlope>=18) x=hp2[c].process(x);
if(cc.hpSlope>=24) x=hp3[c].process(x);
}
if(cc.lpSlope!=0){
x=lp1[c].process(x);
if(cc.lpSlope>=18) x=lp2[c].process(x);
if(cc.lpSlope>=24) x=lp3[c].process(x);
}
x=dly[c].process(x,cc.delayMs,(float)info.sample_rate);
if(cc.phase==180) x=-x;
if(cc.muted) x=0;
float g=dbToLin(cc.gainDb);
float vol=targetMuted?0:fadeVolume;
x*=g*vol;
if(x>32767)x=32767;if(x<-32768)x=-32768;
outCh[c]=x;
}
o1[i*2]=(int16_t)outCh[0];o1[i*2+1]=(int16_t)outCh[1];
o2[i*2]=(int16_t)outCh[2];o2[i*2+1]=(int16_t)outCh[3];
}
i2sStreamOut.write(out1,read);
i2sStreamOut2.write(out2,read);
}
void setup(){Serial.begin(115200);delay(1000);setDefaultPreset();loadSettings();setupI2S();updateAllCoeffs();initWifiAndEspNow();setupWeb();lastAudioTime=millis();Serial.println("S3 DSP 4CH Ready");}
void loop(){
server.handleClient();checkApTimeout();int avail=i2sStreamIn.available();unsigned long now=millis();
if(avail>200){lastAudioTime=now;if(gapMuted){gapMuted=false;isFadingUp=true;fadeStartTime=now;fadeVolume=0;}}
if(!gapMuted&&(now-lastAudioTime>100)){delay(10);flushTank();gapMuted=true;isFadingUp=false;}
if(isFadingUp){unsigned long el=now-fadeStartTime;if(el>=30){fadeVolume=targetMuted?0:targetVolume;isFadingUp=false;}else{float p=(float)el/30.0f;fadeVolume=(targetMuted?0:targetVolume)*p;}}
if(!gapMuted)processAudio();else delay(1);
bool needUpdate=false;
if(fabsf(targetBass-curBass)>0.01){curBass+=(targetBass>curBass?0.05f:-0.05f);needUpdate=true;}
if(fabsf(targetMid-curMid)>0.01){curMid+=(targetMid>curMid?0.05f:-0.05f);needUpdate=true;}
if(fabsf(targetTreble-curTreble)>0.01){curTreble+=(targetTreble>curTreble?0.05f:-0.05f);needUpdate=true;}
if(needUpdate||eqDirty){eqDirty=false;float sr=(float)info.sample_rate;remoteEQ[0].set(Biquad::LOW_SHELF,125,0.7f,curBass,sr);remoteEQ[1].set(Biquad::PEAKING,1000,0.9f,curMid,sr);remoteEQ[2].set(Biquad::HIGH_SHELF,6500,0.75f,curTreble,sr);}
if(nvsDirty){nvsDirty=false;lastNvsChange=now;pendingNvs=true;}
if(pendingNvs&&(now-lastNvsChange>=4500)){pendingNvs=false;saveSettings();}
}

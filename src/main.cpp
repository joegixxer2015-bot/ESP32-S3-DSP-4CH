#include <Arduino.h>
#include <AudioTools.h>
#include <AudioTools/AudioLibs/AudioDSP.h>
#include <esp_now.h>
#include <WiFi.h>

// กำหนดพิน I2S Input (ปรับแก้ให้ตรงกับฮาร์ดแวร์จริงของคุณ)
#define I2S_IN_BCLK 4
#define I2S_IN_LRC  5
#define I2S_IN_DIN  6

// กำหนดพิน I2S Output (ปรับแก้ให้ตรงกับฮาร์ดแวร์จริงของคุณ)
#define I2S_OUT_BCK 15
#define I2S_OUT_LRC 16
#define I2S_OUT_DOUT1 17

I2SStream i2sStreamIn;
I2SStream i2sStreamOut;

// ประกาศตัวกรอง Biquad Filter สำหรับ 3 ย่าน
BiquadFilter bassFilter;
BiquadFilter midFilter;
BiquadFilter trebleFilter;

// ตัวแปรควบคุมเสียง
int currentBass = 0;
int currentMid = 0;
int currentTreble = 0;
bool isMuted = false;
int currentVolume = 80;

// โครงสร้างรับข้อมูลจาก ESP-NOW (ให้ตรงกับฝั่งรีโมต C3)
typedef struct struct_message {
    int controlMode;  
    int value;        
    bool isMuted;     
} struct_message;

// ฟังก์ชันรับค่าจากรีโมต C3 ผ่าน ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    struct_message myData;
    memcpy(&myData, incomingData, sizeof(myData));

    isMuted = myData.isMuted;

    // อัปเดตค่า EQ ตามโหมดที่ส่งมา
    if (myData.controlMode == 0) {
        currentVolume = myData.value;
        Serial.printf("Volume Updated: %d%%\n", currentVolume);
    } else if (myData.controlMode == 1) {
        currentBass = myData.value;
        bassFilter.setGain(currentBass);
        Serial.printf("Bass Updated: %d dB\n", currentBass);
    } else if (myData.controlMode == 2) {
        currentMid = myData.value;
        midFilter.setGain(currentMid);
        Serial.printf("Mid Updated: %d dB\n", currentMid);
    } else if (myData.controlMode == 3) {
        currentTreble = myData.value;
        trebleFilter.setGain(currentTreble);
        Serial.printf("Treble Updated: %d dB\n", currentTreble);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // 1. ตั้งค่า Wi-Fi และ ESP-NOW ฝั่งรับ (S3)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    // 2. ตั้งค่าฟิลเตอร์เริ่มต้น (Bass, Mid, Treble)
    bassFilter.setLowShelf(250, 0, 1.0);     // ย่านเบสตัดที่ 250Hz, Gain เริ่มต้น 0dB
    midFilter.setPeak(1000, 0, 1.0);         // ย่านกลางที่ 1000Hz, Gain เริ่มต้น 0dB
    trebleFilter.setHighShelf(4000, 0, 1.0); // ย่านแหลมที่ 4000Hz, Gain เริ่มต้น 0dB

    // 3. เริ่มต้น I2S Input และ Output Config
    auto configIn = i2sStreamIn.defaultConfig(RX_MODE);
    configIn.pin_bck = I2S_IN_BCLK;
    configIn.pin_ws  = I2S_IN_LRC;
    configIn.pin_data = I2S_IN_DIN;
    i2sStreamIn.begin(configIn);

    auto configOut = i2sStreamOut.defaultConfig(TX_MODE);
    configOut.pin_bck = I2S_OUT_BCK;
    configOut.pin_ws  = I2S_OUT_LRC;
    configOut.pin_data = I2S_OUT_DOUT1;
    i2sStreamOut.begin(configOut);

    Serial.println("ESP32-S3 Audio DSP & EQ Ready for Flash!");
}

void loop() {
    // ถ้าระบบไม่ได้ Mute ให้ประมวลผลสัญญาณเสียงผ่านฟิลเตอร์ EQ
    if (!isMuted) {
        if (i2sStreamIn.available()) {
            int16_t sampleLeft, sampleRight;
            
            // อ่านข้อมูลเสียงเข้ามา (แบบตัวอย่างเสียง / Sample)
            size_t bytesRead = i2sStreamIn.readBytes((uint8_t*)&sampleLeft, sizeof(int16_t));
            
            if (bytesRead > 0) {
                // นำสัญญาณเสียงวิ่งผ่าน Biquad Filter ทีละย่าน (Bass -> Mid -> Treble)
                float processed = bassFilter.filter((float)sampleLeft);
                processed = midFilter.filter(processed);
                processed = trebleFilter.filter(processed);
                
                // คำนวณระดับความดัง Volume รวม
                int16_t finalSample = (int16_t)(processed * (currentVolume / 100.0));

                // ส่งสัญญาณเสียงที่ผ่านการปรับแต่งแล้วออกไปยัง I2S Output (DAC)
                i2sStreamOut.write((uint8_t*)&finalSample, sizeof(int16_t));
                i2sStreamOut.write((uint8_t*)&finalSample, sizeof(int16_t)); // ส่งออกคู่สเตอริโอ (L/R)
            }
        }
    } else {
        // ถ้าสถานะเป็น Mute ให้ส่งค่า 0 (เงียบ) ออกไปที่ Output
        int16_t muteSample = 0;
        i2sStreamOut.write((uint8_t*)&muteSample, sizeof(int16_t));
        i2sStreamOut.write((uint8_t*)&muteSample, sizeof(int16_t));
    }
    
    // พักการประมวลผลเล็กน้อยเพื่อไม่ให้ CPU ทำงานหนักเกินไป
    delay(1);
}

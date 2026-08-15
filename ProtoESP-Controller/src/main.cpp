//Make sure you have everything connected by the schematic in the repository and set these defines correctly!

#define MICpin ADC_CHANNEL_0 //Microphone, pin 1
#define T_in 2 //Output from Touch Sensor (for KY-032, Capac)
#define T_en 42 //Enable pin to Touch Sensor (for KY-032)
#define DATA_PIN_EARS 5  //Ears(Blush) (from outer to inner, POV-right cheek, (if blush: from top to bottom, right cheek nearest to ear first))
#define DATA_PIN_VISOR 7 //Face matrices 1-8 (right cheek, left segment of eye first)
#define DATA_PIN_VISOR2 6 //Face matrices 9-17 (second data pin to reduce chain length and improve signal integrity)
#define VISOR_SPLIT 8 //Number of matrices on the first data pin (DATA_PIN_VISOR); remaining go to DATA_PIN_VISOR2
#define I2C_SDA 8 //SDA for Gyro, OLED, INA219, ToF
#define I2C_SCL 9 //SCL for Gyro, OLED, INA219, ToF
#define MAX_CLK 12 //Clock for MAX72xx matrixes if used
#define MAX_MOSI 11 //Data for MAX72xx matrixes if used1
#define MAX_CS 10 //ChipSelect for MAX72xx matrixes if used
#define animBtn 4 //Pulling this pin LOW cycles through animations
#define fanPWM 13 //PWM pin to control 4pin fan

#define visorType "WS2812" // What displays are you using? (WS2812 or MAX72XX so far)
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW //flip up-down: ::DR1CR0RR1_HW , flip left-right: ::PAROLA_HW , flip both: ::ICSTATION_HW
#define MATRIXESNUM 17 // How many matrices for visor? 11
#define FADESTEPS 4 //how many steps when fading between frames? (0=disabled; only for WS2812 displays)
#define FADE_INTERVAL_MS 20 //ms between each fade step

bool earPresent = true; // Are you using ear leds?
#define earLedsNum 74 // How many? (74 or 32 rn)

bool blushPresent = true; // Are you using blush leds?
#define blushLedsNum 8 // How many? (might crash under 8)
bool useRGBblush = false; //Swaps red-green for RGB strip

bool INApresent = true; //Are you using INA219?

#define boopMode "VL53L1X" //"KY-032" for KY-032, "Capac" for capacitive sensor/boop when HIGH, "APDS9960" for ADPS9960, "VL53L1X" for VL53L1X, leave empty for none

#define revertTilt 8000 //The maximum time that animation caused by tilt gets shown (used as if tilt bugs out etc)

#define oldMatrixFix false //fix for Legacy WS2812B-2020 matrix

#define oledAddr 60 //define oled on address 0x3c

#define LOW_BATT_THRESHOLD 7.0f      //2S LiPo low-battery warning threshold (V) — ~3.5V/cell (compared against compensated/unloaded-equivalent voltage)
#define BATT_INT_RESISTANCE_OHMS 0.052f //Internal resistance of your battery pack (Ω) — measured: INA=5A total → 2.5A/cell; (3.75-3.62)/2.5=0.052Ω/cell → /2(2P)×2(2S) = 0.052Ω pack total
#define LOW_BATT_CONSECUTIVE_SECS 5  //Compensated voltage must stay below threshold for this many seconds before warning triggers (prevents false positives from inrush spikes)

//--------------------------------//No touching after this!

#include <Arduino.h>

#include "esp_adc/adc_oneshot.h"
adc_oneshot_unit_handle_t adc_handle;

#define earTypeSize 7
#define visTypeSize 2
String earTypes[earTypeSize] = {"custom","rainbow","white_noise","corner_sabers","custom_glow","none","fire"}; //available ear type animations
String visorTypes[visTypeSize] = {"custom","all_rainbow"}; //available visor type animations
String vTAcro[visTypeSize] = {"cust","rnbw"}; //OLED acronyms for visor type animations

#include <ezButton.h>
ezButton hwBtn(animBtn);

#include <StreamUtils.h>
#include <sstream>
#define FASTLED_ESP8266_RAW_PIN_ORDER
// FASTLED_RMT_MAX_CHANNELS=3 and FASTLED_RMT_MEM_BLOCKS=1 are set in platformio.ini build_flags
// so they apply to both main.cpp and the FastLED library compilation.
// 3 controllers (visor pin1, visor pin2, ears) all start simultaneously → doneOnChannel()
// never calls startNext() from ISR context (rmt_set_gpio is not IRAM-safe on ESP32-S3).
#include <FastLED.h>
#define ARDUINOJSON_USE_DOUBLE 0
#include <ArduinoJson.h>

#define CONFIG_LITTLEFS_SPIFFS_COMPAT 1
#include <LittleFS.h>

#include "fileOp.h" //CRC + Config variables store/save/load/default
Config cfg;

#include "Misc.h" //Misc/helping functions

#include "oled.h"
SSDOLED oled;

#include "SparkFunLSM6DS3.h"
#include <Wire.h>
LSM6DS3 myIMU;

#include <Adafruit_INA219.h> //edited library in this sketch (replace 0.1R with 0.03R resistor on the board)
Adafruit_INA219 ina219;

#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds;

#include "Adafruit_VL53L1X.h"
Adafruit_VL53L1X vl53;

//--------------------------------//realtime logger
#define LOG_BUFFER_SIZE (50 * 1024)  // 50 KB

char *logBuffer = nullptr;
size_t logIndex = 0;

void logPrint(const char *str) {
  Serial.println(str);
  if (!logBuffer) return;
  size_t len = strlen(str);
  size_t needed = len + 1; //newline
  if (logIndex + needed >= LOG_BUFFER_SIZE) {
    logIndex = 0;
    logBuffer[0] = '\0';
  }
  memcpy(logBuffer + logIndex, str, len);
  logIndex += len;
  logBuffer[logIndex++] = '\n';
  logBuffer[logIndex] = '\0';
}

inline void logPrint(const __FlashStringHelper *str) {
  logPrint((const char*)str);
}

void logPrint(const String &str) {
    logPrint(str.c_str());
}

//--------------------------------//web / wifi
#include "WiFi.h"
#include "esp_wifi.h"
#include "ESPAsyncWebServer.h"
#include <ElegantOTA.h>

AsyncWebServer server(80);

//--------------------------------//Config vars
bool instantReload = false, oledInitDone = false, tiltInitDone = false, getfilesProper = true, ToFInitDone = false, flashlightMode = false;
uint8_t currentEarsFrame = 0, currentVisorFrame = 0, numOfSegm, numAnimBlush, totalAnims;
uint16_t visorLedsNum = MATRIXESNUM*64;
String currentAnim = "", animToLoad = "", availAnims[50], getfilesCache;
float micDC = 800;
int16_t lastToFDistance = -1; //cached last VL53L1X reading for /tof endpoint

//--------------------------------//getting stored anims names and count
void getFilesFunc() {
  getfilesCache = "";
  totalAnims = 0;
  File root = LittleFS.open("/anims");
  File file = root.openNextFile();
  while(file){
    availAnims[totalAnims] = String(file.name());
    getfilesCache += availAnims[totalAnims] + ";";
    totalAnims++;
    file = root.openNextFile();
  }
  getfilesProper = false;
}

//--------------------------------//Structs for anims in psram
struct FramesEars {
  int timespan;
  long ledColor[earLedsNum];
};

struct AnimNowEars {
  int type;
  int numOfFrames;
  FramesEars* frames = nullptr;
};

struct FramesVisor {
  int timespan;
  uint8_t numSegm; //segment count stored per-frame to avoid global race with WiFi callbacks
  uint64_t leds[MATRIXESNUM];
  long ledsBlush[blushLedsNum];
  long fColor[MATRIXESNUM];
  long ppColor[MATRIXESNUM][64];
};

struct AnimNowVisor {
  int type;
  int numOfFrames;
  FramesVisor* frames = nullptr;
  bool isMouth[MATRIXESNUM];
};

AnimNowEars* earsNow;
AnimNowVisor* visorNow;

//--------------------------------//MAX LEDs
#include <MD_MAX72xx.h>
#include <SPI.h>

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, MAX_MOSI, MAX_CLK, MAX_CS, MATRIXESNUM);

//--------------------------------//WS2812 LEDs
CRGB earLeds[earLedsNum];
CRGB blushLeds[blushLedsNum];
CRGB visorLeds[MATRIXESNUM*64];
CRGB visorLedsNEW[MATRIXESNUM*64];
CRGB c2Leds[earLedsNum+blushLedsNum];

CLEDController *ledController[3]; // [0]=visor pin1 (matrices 1-VISOR_SPLIT), [1]=ears/blush, [2]=visor pin2 (matrices VISOR_SPLIT+1-MATRIXESNUM)

CRGB pixelBuffer[18];
CRGB visorPixelBuffer[10];
uint8_t noiseData[earLedsNum];
uint8_t fireHeat[earLedsNum]; // heat buffer for fire animation

DEFINE_GRADIENT_PALETTE( blackWhite_gp ) {
  0,   100,  0, 0,
  120,   0,  0, 0,
  255, 255,  255, 255
};
CRGBPalette16 blackWhite = blackWhite_gp;

const std::vector<std::vector<int>> lookupDiag1 = 
{{14,15,16},
 {13,27,28,1},
 {12,26,36,17,2},
 {25,35,29,18},
 {11,34,37,30,3},
 {24,33,31,19},
 {10,23,32,20,4},
 {9,22,21,5},
 {8,7,6}};

const std::vector<std::vector<int>> lookupDiag2 = 
{{2,3,4},
 {1,18,19,5},
 {16,17,30,20,6},
 {28,29,31,21},
 {15,36,37,32,7},
 {27,35,33,22},
 {14,26,34,23,8},
 {13,25,24,9},
 {12,11,10}};

//--------------------------------//Load functions
bool loadAnim(String anim, String temp) {
  JsonDocument doc;
  DeserializationError error;

  if (currentAnim != anim || anim == "POSTAnimLoad") {
    if(anim == "POSTAnimLoad") {
      logPrint(F("[I] POST load"));
      error = deserializeJson(doc, temp);
    } else {
      delay(25);
      File file = LittleFS.open("/anims/"+anim, "r");
      if (!file) {
        logPrint(F("[E] There was an error opening the animation file!"));
        file.close();
        return false;
      }
      logPrint(F("[I] Animation file opened!"));
      ReadBufferingStream bufferedFile{file, 64};
      error = deserializeJson(doc, bufferedFile);
      file.close();
    }
    
    if(error){
      logPrint("[E] Failed to deserialize animation file! : " + String(error.c_str()));
      return false;
    }

    // Free previously allocated frames
    if (earsNow->frames) { free(earsNow->frames); earsNow->frames = nullptr; }
    if (visorNow->frames) { free(visorNow->frames); visorNow->frames = nullptr; }

    currentAnim = anim;

    //Ears anim type
    for(int o=0;o<earTypeSize;o++) {
      if(doc["ears"]["type"].as<String>() == earTypes[o]) {
        earsNow->type = o;
        break;
      }
    }
    //Ears anim load
    earsNow->numOfFrames = doc["ears"]["frames"].size();
    earsNow->frames = (FramesEars*) ps_malloc(sizeof(FramesEars) * earsNow->numOfFrames);
    for(int x = 0; x < earsNow->numOfFrames; x++) {
      earsNow->frames[x].timespan = doc["ears"]["frames"][x]["timespan"].as<int>();
      for(int y = 0; y < doc["ears"]["frames"][x]["leds"].size(); y++) {
        earsNow->frames[x].ledColor[y] = strtol(doc["ears"]["frames"][x]["leds"][y].as<String>().c_str(), NULL, 16);
      }
    }
    //Visor anim type
    for(int o=0;o<visTypeSize;o++) {
      if(doc["visor"]["type"].as<String>() == visorTypes[o]) {
        visorNow->type = o;
        break;
      }
    }
    //isMouth
    for(int y = 0; y < doc["visor"]["isMouth"].size(); y++) {
      visorNow->isMouth[y] = doc["visor"]["isMouth"][y].as<bool>();
    }
    //Visor anim load
    visorNow->numOfFrames = doc["visor"]["frames"].size();
    visorNow->frames = (FramesVisor*) ps_malloc(sizeof(FramesVisor) * visorNow->numOfFrames);
    for(int x = 0; x < visorNow->numOfFrames; x++) {
      visorNow->frames[x].timespan = doc["visor"]["frames"][x]["timespan"].as<int>();
      numAnimBlush = (uint8_t)doc["visor"]["frames"][x]["ledsBlush"].size();
      for(int y = 0; y < numAnimBlush; y++) {
        if(y<blushLedsNum) {
          visorNow->frames[x].ledsBlush[y] = strtol(doc["visor"]["frames"][x]["ledsBlush"][y].as<String>().c_str(), NULL, 16);
        }
      }
      visorNow->frames[x].numSegm = (uint8_t)doc["visor"]["frames"][x]["leds"].size();
      for(int y = 0; y < visorNow->frames[x].numSegm; y++) {
        visorNow->frames[x].fColor[y] = strtol(doc["visor"]["frames"][x]["fColor"][y].as<String>().c_str(), NULL, 16); //should return 0 if not present
        visorNow->frames[x].leds[y] = strtoull(doc["visor"]["frames"][x]["leds"][y].as<String>().c_str(), NULL, 16); //string to uint64
      }
      for(int i = 0; i < MATRIXESNUM; i++) { //wipe ppColor data
        for(int o = 0; o < 64; o++) {
          visorNow->frames[x].ppColor[i][o] = 0;
        }
      }
      int numOfpp = doc["visor"]["frames"][x]["ppColor"].size();
      for(int y = 0; y < numOfpp; y++) {
        int numOfppData = doc["visor"]["frames"][x]["ppColor"][y]["data"].size();
        for(int z = 0; z < numOfppData; z++) { //ppColor[matrix][pixel] = color
          visorNow->frames[x].ppColor[doc["visor"]["frames"][x]["ppColor"][y]["mIndex"].as<int>()][doc["visor"]["frames"][x]["ppColor"][y]["data"][z][0].as<int>()] = strtol(doc["visor"]["frames"][x]["ppColor"][y]["data"][z][1].as<String>().c_str(), NULL, 16);
        }
      }
    }

    instantReload = true;
    currentVisorFrame = 0;
    currentEarsFrame = 0;

    flashlightMode = (anim == "flashlight.json");
    if(flashlightMode) {
      logPrint(F("[I] Flashlight mode: brightness set to 255"));
    }

    if(cfg.oledEna && oledInitDone) {
      oled.writeAnim(anim.substring(0,anim.length()-5));
      oled.writeRGB(vTAcro[visorNow->type]);
    }
    return true;
  }
  return false;
}

//--------------------------------//BLE
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 3
#define CONFIG_BT_NIMBLE_ROLE_CENTRAL_DISABLED
#define CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL 1
#include "NimBLEDevice.h"

BLEServer *pServer = NULL;
BLECharacteristic * pCharacteristic;
BLEAdvertising* pAdvertising;

class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
      String temp = String(pCharacteristic->getValue().c_str());
      if(temp.charAt(0) == 'g') { //legacy remote reasons
        pCharacteristic->setValue("i"+String(totalAnims));
        pCharacteristic->notify();
      } else if (temp.charAt(0) == '?') {
        String animtemp;
        for(int i = 0; i < totalAnims; i++) {
          animtemp += availAnims[i].substring(0, availAnims[i].length() - 5);
          animtemp += ";";
        }
        pCharacteristic->setValue(animtemp);
        pCharacteristic->notify(true);
      } else if (temp.charAt(0) == ';') { //command
        if (temp.indexOf("rgb") > 0 && visorType == "WS2812") {
          visorNow->type++;
          if(visorNow->type == visTypeSize)
            visorNow->type = 0;
          if(cfg.oledEna && oledInitDone)
            oled.writeRGB(vTAcro[visorNow->type]);
        } else if (temp.indexOf("set") > 0) {
          temp.remove(0,4);
          if(cfg.oledEna && oledInitDone)
            oled.writeSet(temp.toInt()+1);
        }
      } else if (temp.toInt() > 0 && temp.toInt() <= totalAnims){ //legacy remote reasons
        animToLoad = availAnims[temp.toInt()-1];
      } else {
        for(int i = 0; i < totalAnims; i++) {
          if(temp == availAnims[i].substring(0, availAnims[i].length() - 5)) {
            animToLoad = availAnims[i];
          }
        }
      }
      logPrint("[I] BT Recv.: "+temp);
    };
} chrCallbacks;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
      // If there is still room for more connections, keep advertising so additional remotes can connect
      if (pServer->getConnectedCount() < CONFIG_BT_NIMBLE_MAX_CONNECTIONS) {
          NimBLEDevice::startAdvertising();
      }
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      NimBLEDevice::startAdvertising();
  }
} serverCallbacks;

bool startBLE() {
  std::string stdStr(cfg.wifiName.c_str(), cfg.wifiName.length());
  BLEDevice::init(stdStr);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  BLEService *pService = pServer->createService("ffe0");

  pCharacteristic = pService->createCharacteristic("ffe1",
      NIMBLE_PROPERTY::BROADCAST | NIMBLE_PROPERTY::READ  |
      NIMBLE_PROPERTY::NOTIFY    | NIMBLE_PROPERTY::WRITE |
      NIMBLE_PROPERTY::INDICATE
  );
  pCharacteristic->setValue(totalAnims);
  pCharacteristic->setCallbacks(&chrCallbacks);

  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName(stdStr);
  pAdvertising->addServiceUUID(BLEUUID(pService->getUUID()));
  pAdvertising->enableScanResponse(true);
  if(!pAdvertising->start(0)) {
    return false;
  }
  return true;
}

//--------------------------------//WiFi server setup
void startWiFiWeb() {
  WiFi.setSleep(false);          // Arduino wrapper: disable modem sleep
  esp_wifi_set_ps(WIFI_PS_NONE); // IDF-level guarantee: modem stays on, no burst interrupts that corrupt RMT/WS2812B signal
  WiFi.softAP(cfg.wifiName, cfg.wifiPass);

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/getfiles", HTTP_GET, [](AsyncWebServerRequest *request){ //returns current anim + all avaible anims
    if(getfilesProper) {
      getFilesFunc();
    }
    request->send(200, "text/plain", currentAnim+";"+getfilesCache);
  });

  server.on("/saveconfig", HTTP_GET, [](AsyncWebServerRequest *request){ //saves config
    //features enable
    cfg.getBool(request, "boopEna", cfg.boopEna);
    cfg.getBool(request, "speechEna", cfg.speechEna);
    cfg.getBool(request, "tiltEna", cfg.tiltEna);
    cfg.getBool(request, "bleEna", cfg.bleEna);
    cfg.getBool(request, "oledEna", cfg.oledEna);
    cfg.getBool(request, "lowBattSwitch", cfg.lowBattSwitch);
    //brightness
    cfg.getInt(request, "bEar", cfg.bEar);
    cfg.getInt(request, "bVisor", cfg.bVisor);
    if(cfg.getInt(request, "bOled", cfg.bOled)) {
        if(cfg.oledEna && oledInitDone)
            oled.oledBright(cfg.bOled);
    }
    //animation settings
    cfg.getInt(request, "rbSpeed", cfg.rbSpeed);
    cfg.getInt(request, "rbWidth", cfg.rbWidth);
    cfg.getInt(request, "spMin", cfg.spMin);
    cfg.getInt(request, "spMax", cfg.spMax);
    cfg.getInt(request, "spTrig", cfg.spTrig);
    //tilt animations
    cfg.getString(request, "aTilt", cfg.aTilt);
    cfg.getString(request, "aUp", cfg.aUp);
    cfg.getString(request, "aBoop", cfg.aBoop);
    //tilt neutral
    cfg.getFloat(request, "neutralX", cfg.neutralX);
    cfg.getFloat(request, "neutralY", cfg.neutralY);
    cfg.getFloat(request, "neutralZ", cfg.neutralZ);
    //tilt triggers
    cfg.getFloat(request, "tiltX", cfg.tiltX);
    cfg.getFloat(request, "tiltY", cfg.tiltY);
    cfg.getFloat(request, "tiltZ", cfg.tiltZ);
    cfg.getFloat(request, "upX", cfg.upX);
    cfg.getFloat(request, "upY", cfg.upY);
    cfg.getFloat(request, "upZ", cfg.upZ);
    cfg.getFloat(request, "tiltTol", cfg.tiltTol);
    //RGB visor color
    if(cfg.getString(request, "visColor", cfg.visColorStr))
        cfg.visColor = strtol(cfg.visColorStr.c_str() + 1, NULL, 16);
    //wifi
    cfg.getString(request, "wifiName", cfg.wifiName);
    cfg.getString(request, "wifiPass", cfg.wifiPass);
    //boop threshold
    cfg.getInt(request, "boopThresh", cfg.boopThresh);
    //ToF FOV (ROI size, 4-16; 16=~27°, 8=~15°, 4=~7°)
    if (cfg.getInt(request, "toFov", cfg.toFov)) {
      cfg.toFov = constrain(cfg.toFov, 4, 16);
      if (ToFInitDone) vl53.VL53L1X_SetROI(cfg.toFov, cfg.toFov);
    }
    delay(25);
    if(cfg.save()) {
      instantReload = true;
      request->redirect("/saved.html?main");
    } else {
      request->send(200, "text/plain", F("Saving config failed!"));
    }
  });

  server.on("/savefile", HTTP_POST, [](AsyncWebServerRequest *request){ //saves data from POST to file
    if(request->hasParam("file", true) && request->hasParam("content", true)) {
      File file = LittleFS.open("/anims/"+request->getParam("file", true)->value()+".json", "w");
      if (!file) {
        logPrint(F("[E] There was an error opening the file for saving an animation!"));
        file.close();
        request->send(200, "text/plain", F("Error opening file for writing!"));
      } else {
        logPrint(F("[I] File saved!"));
        file.print(request->getParam("content", true)->value());
        file.close();
        getfilesProper = true;
        request->redirect("/saved.html?anim");
      }
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });

  server.on("/deletefile", HTTP_GET, [](AsyncWebServerRequest *request){ //deletes asked file
    if(request->hasParam("file")) {
      LittleFS.remove("/anims/"+request->getParam("file")->value());
      getfilesProper = true;
      request->redirect("/saved.html?main");
    } else {
      request->send(200, "text/plain", F("Parameter 'file' not present!"));
    }
  });

  server.on("/change", HTTP_GET, [](AsyncWebServerRequest *request){ //loads anim from selected avaible anims
    if(request->hasParam("anim")) {
      if(request->getParam("anim")->value() == currentAnim) {
        request->redirect("/saved.html?main");
      } else {
        // Queue the load to happen on the main loop (Core 1) to avoid race with setAllVisor
        animToLoad = request->getParam("anim")->value();
        request->redirect("/saved.html?main");
      }
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });

  server.on("/change", HTTP_POST, [](AsyncWebServerRequest *request){ //loads anim from POST request
    if(request->hasParam("anim", true)) {
      // Queue POST load to main loop; store content in animToLoad with a special prefix
      animToLoad = "POST:" + request->getParam("anim", true)->value();
      request->redirect("/saved.html?main");
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.on("/fanpwm", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("duty")) {
      int duty = request->getParam("duty")->value().toInt();
      if(duty < 256 && duty >= 0) {
        //ledcWrite(0, duty);
        ledcWrite(fanPWM, duty); //Arduino 3.x core
        cfg.fanDuty = duty;
        cfg.save();
        request->send(200, "text/plain", "Set PWM to: " + String(duty));
      } else {
        request->send(200, "text/plain", F("Invalid duty cycle"));
      }
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });
  
  server.on("/rgb", HTTP_GET, [](AsyncWebServerRequest *request){
    if(visorType == "WS2812") {
      visorNow->type++;
      if(visorNow->type == visTypeSize)
        visorNow->type = 0;
      if(cfg.oledEna && oledInitDone)
        oled.writeRGB(vTAcro[visorNow->type]);
        logPrint("[I] Changing visor type to: "+visorTypes[visorNow->type]);
    }
    request->redirect("/saved.html?main");
  });

  server.on("/gyro", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(floor(myIMU.readFloatAccelX()*100)/100)+";"+String(floor(myIMU.readFloatAccelY()*100)/100)+";"+String(floor(myIMU.readFloatAccelZ()*100)/100));
  });

  server.on("/tof", HTTP_GET, [](AsyncWebServerRequest *request){
    String dbg = "boopEna=" + String(cfg.boopEna) +
                 " ToFInitDone=" + String(ToFInitDone) +
                 " lastToFDist=" + String(lastToFDistance) +
                 " dataReady=" + String(vl53.dataReady());
    if(!ToFInitDone) {
      request->send(200, "text/plain", "ToF not initialized! " + dbg);
      return;
    }
    if (boopMode == "APDS9960") {
      request->send(200, "text/plain", String(255 - apds.readProximity()));
    } else if (boopMode == "VL53L1X") {
      if (lastToFDistance != -1) {
        request->send(200, "text/plain", String(lastToFDistance) + " mm | " + dbg);
      } else {
        request->send(200, "text/plain", "Data not ready! " + dbg);
      }
    }
  });
  
  server.on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", logBuffer);
  });

  server.onNotFound([](AsyncWebServerRequest *request){request->send(404, "text/plain", "Not found");});
  ElegantOTA.begin(&server);
  server.begin();

  ElegantOTA.setAutoReboot(true);
}

//--------------------------------//Setup
void setup() {
  Serial.begin(115200);

  pinMode(animBtn, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pinMode(fanPWM, OUTPUT);

  if(boopMode == "KY-032") {
    pinMode(T_in, INPUT_PULLUP);
    pinMode(T_en, OUTPUT);
  } else if ((boopMode == "Capac")) {
    pinMode(T_in, INPUT_PULLDOWN);
  }

  hwBtn.setDebounceTime(50);

  if(!LittleFS.begin(true)) {
    logPrint(F("[E] An Error has occurred while mounting LittleFS! Halting"));
    while(1){};
  }

  if(psramInit() && ESP.getFreePsram() != 0) {
    earsNow  = (AnimNowEars *)  ps_calloc(1, sizeof(AnimNowEars));
    visorNow = (AnimNowVisor *) ps_calloc(1, sizeof(AnimNowVisor));
    logBuffer = (char*) ps_malloc(LOG_BUFFER_SIZE);
    logIndex = 0;
    logBuffer[0] = '\0';
  } else {
    logPrint(F("[E] Could not init PSRAM, either this ESP doesn't have one or is malfunctioning, halting..."));
    while(1){};
  }

  if(!cfg.load()) {
    logPrint(F("[E] An Error has occurred while loading config file! Loading defaults"));
    cfg.setDefault();
  }

  micDC = (float)cfg.spMin;

  adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
  adc_oneshot_chan_cfg_t channel_config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, MICpin, &channel_config));
  ledcAttach(fanPWM, 250, 8); //suport Arduino 3.x
  ledcWrite(fanPWM, cfg.fanDuty); //Arduino 3.x core
  //ledcSetup(0, 25000, 8); //For Arduino 2.x
  //ledcAttachPin(fanPWM, 0); //For Arduino 2.x
  //ledcWrite(0, cfg.fanDuty); //for Arduino 2.x

  if(visorType == "WS2812") {
    ledController[0] = &FastLED.addLeds<WS2812B, DATA_PIN_VISOR,  GRB>(visorLeds,                    VISOR_SPLIT * 64);                  //matrices 1-VISOR_SPLIT
    ledController[2] = &FastLED.addLeds<WS2812B, DATA_PIN_VISOR2, GRB>(&visorLeds[VISOR_SPLIT * 64], (MATRIXESNUM - VISOR_SPLIT) * 64); //matrices VISOR_SPLIT+1 to MATRIXESNUM
  } else if (visorType == "MAX72XX") {
    mx.begin();
  }
  if(earPresent || blushPresent) { //RMT4 2 controller fix
    ledController[1] = &FastLED.addLeds<WS2812B, DATA_PIN_EARS, GRB>(c2Leds, earLedsNum+blushLedsNum);
  }
  /*if(earPresent) {
    ledController[1] = &FastLED.addLeds<WS2812B, DATA_PIN_EARS, GRB>(earLeds, earLedsNum);
  }
  if(blushPresent) {
    ledController[2] = &FastLED.addLeds<WS2812B, DATA_PIN_BLUSH, RGB>(blushLeds, blushLedsNum);
  }*/
  FastLED.setCorrection(TypicalPixelString);
  FastLED.setDither(0);

  startWiFiWeb();

  getFilesFunc();
  if(cfg.bleEna) { //you can disable BLE in config
    if(!startBLE()) {
      logPrint(F("[E] An Error has occurred while starting BLE!"));
    }
  }

  //I2C things
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(400000); //100k = 113ms; 400k = 33ms; (800k = 20ms; 1mhz / 2mhz = 17ms = breaks apds)

  if(cfg.tiltEna) {
    if(myIMU.begin()) {
      logPrint(F("[E] An Error has occurred while initializing LSM!"));
      cfg.tiltEna = false;
    } else {
      tiltInitDone = true;
    }
  }

  if(cfg.oledEna) {
    if(!oled.init(oledAddr,cfg.bOled,INApresent)) {
      logPrint(F("[E] An Error has occurred while initializing SSD1306!"));
      cfg.oledEna = false;
    } else {
      oledInitDone = true;
      oled.speak(false);
      oled.writeSet(1);
    }
  }

  if(INApresent && oledInitDone) {
    if(!ina219.begin()) {
      logPrint(F("[E] An Error has occurred while initializing INA219 chip!"));
      INApresent = false;
    } else {
      ina219.setCalibration_16V_8A();
    }
  }

  if(boopMode == "APDS9960" && cfg.boopEna) {
    if(!apds.begin()){
      cfg.boopEna = false;
      logPrint(F("[E] An Error has occurred while initializing APDS9960 chip!"));
    } else {
      ToFInitDone = true;
      apds.enableProximity(true);
      apds.setProxPulse(APDS9960_PPULSELEN_8US, 8);
    }
  } else if (boopMode == "VL53L1X" && cfg.boopEna) {
    Wire.beginTransmission(0x29);
    if(Wire.endTransmission() != 0) {
      cfg.boopEna = false;
      logPrint(F("[E] An Error has occurred while finding VL53L1X chip!"));
    } else {
      if(!vl53.begin()) {
        cfg.boopEna = false;
        logPrint(F("[E] An Error has occurred while initializing VL53L1X chip!"));
      } else {
        if (!vl53.startRanging()) {
          cfg.boopEna = false;
          logPrint(F("[E] An Error has occurred while starting ranging with VL53L1X chip!"));
        } else {
          ToFInitDone = true;
          vl53.setTimingBudget(50);
          vl53.VL53L1X_SetROI(cfg.toFov, cfg.toFov);
        }
      }
    }
  }
  
  while(millis()<2000) {yield();} //2s delay for the anim to load properly (idk why but it doesnt without this or with 1s)
  loadAnim("default.json","");

  logPrint("[I] Free heap: "+String(ESP.getFreeHeap()));
  logPrint("[I] Free PSRAM: "+String(ESP.getFreePsram()));
}

//--------------------------------//Battery percentage from compensated (unloaded-equivalent) voltage — 2S LiPo curve
// Points: 8.40V=100%, 8.20V=90%, 8.00V=75%, 7.80V=60%, 7.60V=45%, 7.40V=30%, 7.20V=15%, 7.00V=5%, 6.80V=0%
int batteryPercent(float v) {
  if(v >= 8.40f) return 100;
  if(v >= 8.20f) return (int)map((long)(v * 100), 820, 840, 90, 100);
  if(v >= 8.00f) return (int)map((long)(v * 100), 800, 820, 75, 90);
  if(v >= 7.80f) return (int)map((long)(v * 100), 780, 800, 60, 75);
  if(v >= 7.60f) return (int)map((long)(v * 100), 760, 780, 45, 60);
  if(v >= 7.40f) return (int)map((long)(v * 100), 740, 760, 30, 45);
  if(v >= 7.20f) return (int)map((long)(v * 100), 720, 740, 15, 30);
  if(v >= 7.00f) return (int)map((long)(v * 100), 700, 720, 5,  15);
  if(v >= 6.80f) return (int)map((long)(v * 100), 680, 700, 0,  5);
  return 0;
}

//--------------------------------//Loop vars
String oldanim, boopoldanim, preLowBattAnim;
bool FdisplayVisor = false, FdisplayBlush = false, FdisplayEar = false, booping = false, wasTilt = false, boopRea = false, remoteSign = false, speaking = true, animLoading = false, lowBattFlash = false, lowBattActive = false;
float zAx,yAx,finalMicAvg,avgMicArr[10], micAttack = 0.35f, micRelease = 0.2f, env = 0.0f;
int boopRead, startIndex = 1, micVolume, currentMicAvg = 0, btnNum = 0, currFade = 1, apdsprox = 255, lowBattConsecutiveCount = 0;
unsigned long lastMillsEars = 0, lastMillsVisor = 0, lastMillsTilt = 0, laskSpeakCheck = 0, lastMillsBoop = 0, lastFLED = 0, vaStatLast = 0, btnPressTime = 0, tiltChange = 0, check0button = 0, looptime = 0, fadeTime = 0, laskSpeakAnim = 0, lastBoopCheck = 0;

void dynamicSpeak(uint64_t *leds, bool isMouth[MATRIXESNUM], int volume) {
  int mouthIndexes[MATRIXESNUM];
  int mouthCount = 0;
  for (int i = 0; i < MATRIXESNUM; i++) { // collect true (mouth) indexes
    if (isMouth[i]) {
      mouthIndexes[mouthCount++] = i;
    }
  }
  int half = mouthCount / 2;
  for (int i = 0; i < half; i++) {
    int x = map(volume, 0, 100, 0, half * 8) - (i * 8);
    int leftIndex  = mouthIndexes[half - 1 - i];
    int rightIndex = mouthIndexes[half + i];
    if (x > 0) {
      x = constrain(x, 0, 8);
      leds[leftIndex]  = speakMatrix(leds[leftIndex],  x, true);
      leds[rightIndex] = speakMatrix(leds[rightIndex], x, false);
    }
  }
}

void setAllVisor(struct CRGB *ledArray, long ledColor, int visorFrame, bool newFrame = false) {
  if(visorNow->frames == nullptr) return; // guard against null frames during anim load
  if(visorFrame < 0 || visorFrame >= visorNow->numOfFrames) return; // bounds check
  uint64_t tempLeds[MATRIXESNUM];
  memcpy(tempLeds, visorNow->frames[visorFrame].leds, sizeof(tempLeds));
  if(speaking) {
    dynamicSpeak(tempLeds, visorNow->isMouth, micVolume);
  }
  uint8_t segCount = visorNow->frames[visorFrame].numSegm; //use per-frame count, safe from WiFi callback races
  for(int y = 0; y < segCount; y++) {
    for (int i = 0; i < 8; i++) {
      byte row = (tempLeds[y] >> i * 8) & 0xFF;
      for (int j = 0; j < 8; j++) {
        if(visorType == "WS2812") {
          long tempColor = ledColor; //use given color
          if(ledColor == 0) { //if not given a color
            if(visorNow->frames[visorFrame].ppColor[y][(i*8)+j] != 0) { //use ppColor if available
              tempColor = visorNow->frames[visorFrame].ppColor[y][(i*8)+j];
            } else if(visorNow->frames[visorFrame].fColor[y] != 0) { //if not, use fColor if available
              tempColor = visorNow->frames[visorFrame].fColor[y];
            } else { // else config color
              tempColor = cfg.visColor;
            }
          }
          if(oldMatrixFix) {
            ledArray[(y*64)+(i*8)+((i%2!=0)?j:7-j)] = (bitRead(row,j))?tempColor:CRGB::Black; //includes fix for bad rgbmatrix
          } else {
            ledArray[(y*64)+(i*8)+j] = (bitRead(row,j))?tempColor:CRGB::Black;
          }
        } else if (visorType == "MAX72XX") {
          mx.setPoint(i, j+(y*8), bitRead(row, j)); //MAXstuff
        }
      }
    }
  }
  FdisplayVisor = true;
  if(newFrame) { //only reset fade counter when a new animation frame is committed, not on speech updates
    currFade = 1;
    fadeTime = millis();
  }
}

void loop() {
  ElegantOTA.loop();
  //--------------------------------//EAR Leds render
  if(earPresent) {
    if(earsNow->type == 0 && earsNow->frames != nullptr) { //custom
      if(lastMillsEars+earsNow->frames[currentEarsFrame].timespan <= millis() || instantReload) {
        currentEarsFrame++;
        lastMillsEars = millis();
        if(currentEarsFrame == earsNow->numOfFrames) { currentEarsFrame = 0; } //loop back to first frame if last frame
        for(int y = 0; y < earLedsNum; y++) { earLeds[y] = earsNow->frames[currentEarsFrame].ledColor[y]; } //set ear leds
        FdisplayEar = true;
      }
    } else if (earsNow->type == 1) { //rainbow
      fill_rainbow(pixelBuffer, 4, millis()/cfg.rbSpeed, 255/cfg.rbWidth);
      if(earLedsNum == 74) {
        for(int x = 0;x<earLedsNum;x++) {
          if(x<16) {
            earLeds[x] = pixelBuffer[0];
            earLeds[x+37] = pixelBuffer[0];
          } else if(x<28) {
            earLeds[x] = pixelBuffer[1];
            earLeds[x+37] = pixelBuffer[1];
          } else if(x<36) {
            earLeds[x] = pixelBuffer[2];
            earLeds[x+37] = pixelBuffer[2];
          } else if(x==36) {
            earLeds[x] = pixelBuffer[3];
            earLeds[x+37] = pixelBuffer[3];
          }
        }
      } else {
        for(int x = 0;x<earLedsNum;x++) {
          earLeds[x] = pixelBuffer[0];
        }
      }
      FdisplayEar = true;
    } else if (earsNow->type == 2) { //white_noise
      memset(noiseData, 0, earLedsNum);
      fill_raw_noise8(noiseData, earLedsNum, 2, 0, 50, millis()/4);
      for(int x = 0;x<earLedsNum;x++) {
        earLeds[x] = ColorFromPalette(blackWhite, noiseData[x]);
      }
      FdisplayEar = true;
    } else if (earsNow->type == 3 && earLedsNum == 74) { //corner_sabers - only 74 led mode
      if(lastFLED+cfg.rbSpeed < millis()) {
        lastFLED = millis();
        startIndex++;
        int tempIndex = startIndex;
        for(int x = 0;x<18;x++) {
          pixelBuffer[x] = ColorFromPalette(RainbowStripeColors_p, tempIndex, 255, NOBLEND);
          tempIndex+=3;
        }
        for(int x = 0;x<9;x++) {
          for(int y = 0;y<lookupDiag1[x].size();y++) {
            earLeds[lookupDiag2[x][y]-1] = pixelBuffer[x];
            earLeds[lookupDiag1[x][y]+36] = pixelBuffer[x];
          }
        }
        FdisplayEar = true;
      }
    } else if (earsNow->type == 4 && earsNow->frames != nullptr) { //custom_glow
      fill_rainbow(pixelBuffer, 4, millis()/cfg.rbSpeed, 255/cfg.rbWidth);
      for(int y = 0; y < earLedsNum; y++) {
        if(earsNow->frames[0].ledColor[y] == 0) {
          earLeds[y] = 0x000000;
        } else {
          earLeds[y] = pixelBuffer[0];
        }
      }
      FdisplayEar = true;
    } else if (earsNow->type == 5) {} //none
    else if (earsNow->type == 6) { //fire - flames rise upward on concentric rings
      if(instantReload) {
        memset(fireHeat, 0, sizeof(fireHeat)); // clear heat buffer on animation load
      }
      if(millis() - lastFLED > 30) { // ~33fps fire update
        lastFLED = millis();
        if(earLedsNum == 74) {
          // Ring layout per ear (37 LEDs): outer(16) + ring2(12) + ring3(8) + center(1)
          // LED 0 = top of each ring, wired clockwise, outer ring first.
          // We use 8 height rows: row 0 = top of disc, row 7 = bottom of disc.
          // fireHeat[ear*8 + row] stores the heat for each height row (uses first 16 bytes).
          //
          // Row index = round((1 - cos(2*pi*p/ringSize)) / 2 * 7)
          // p=0 (top) → row 0;  p=ringSize/2 (bottom) → row 7
          const int NUM_ROWS = 8;
          static const uint8_t rOff[4]      = { 0, 16, 28, 36};
          static const uint8_t rowOuter[16] = {0,0,1,2,4,5,6,7, 7,7,6,5,4,2,1,0};
          static const uint8_t rowRing2[12] = {0,0,2,4,5,7,7,7, 5,4,2,0};
          static const uint8_t rowRing3[ 8] = {0,1,4,6,7,6,4,1};

          for(int ear = 0; ear < 2; ear++) {
            int base = ear * NUM_ROWS; // index into fireHeat[] for this ear's row heat values

            // Step 1: Cool down each height row
            for(int r = 0; r < NUM_ROWS; r++) {
              uint8_t cool = random8(0, 45);
              fireHeat[base + r] = (fireHeat[base + r] > cool) ? fireHeat[base + r] - cool : 0;
            }

            // Step 2: Heat rises — blend each row upward (from bottom toward top)
            for(int r = 0; r < NUM_ROWS - 2; r++) {
              fireHeat[base + r] = ((uint16_t)fireHeat[base + r] +
                                     (uint16_t)fireHeat[base + r + 1] +
                                     (uint16_t)fireHeat[base + r + 2]) / 3;
            }

            // Step 3: Always keep bottom rows (6–7) hot so the base never goes dark
            fireHeat[base + NUM_ROWS - 1] = qadd8(fireHeat[base + NUM_ROWS - 1], random8(100, 180));
            fireHeat[base + NUM_ROWS - 2] = qadd8(fireHeat[base + NUM_ROWS - 2], random8(80, 160));
            // Cap heat at 200 to prevent HeatColor() from reaching the white range
            for(int r = 0; r < NUM_ROWS; r++) {
              if(fireHeat[base + r] > 200) fireHeat[base + r] = 200;
            }

            // Step 4: Map each LED to its row heat value (with small per-LED noise)
            int earBase = ear * 37;
            for(int p = 0; p < 16; p++) {
              earLeds[earBase + rOff[0] + p] = HeatColor(qsub8(fireHeat[base + rowOuter[p]], random8(0, 25)));
            }
            for(int p = 0; p < 12; p++) {
              earLeds[earBase + rOff[1] + p] = HeatColor(qsub8(fireHeat[base + rowRing2[p]], random8(0, 25)));
            }
            for(int p = 0; p <  8; p++) {
              earLeds[earBase + rOff[2] + p] = HeatColor(qsub8(fireHeat[base + rowRing3[p]], random8(0, 25)));
            }
            // Center LED — sits at the middle of the disc (row NUM_ROWS/2)
            earLeds[earBase + 36] = HeatColor(qsub8(fireHeat[base + NUM_ROWS / 2], random8(0, 25)));
          }
        } else {
          // Fallback for non-74 LED mode: simple linear fire
          int halfSize = earLedsNum / 2;
          for(int ear = 0; ear < 2; ear++) {
            int base = ear * halfSize;
            for(int i = 0; i < halfSize; i++) {
              uint8_t cool = random8(0, ((55 * 10) / halfSize) + 2);
              fireHeat[base + i] = (fireHeat[base + i] > cool) ? fireHeat[base + i] - cool : 0;
            }
            for(int i = halfSize - 1; i >= 2; i--) {
              fireHeat[base + i] = ((uint16_t)fireHeat[base + i - 1] +
                                     (uint16_t)fireHeat[base + i - 2] +
                                     (uint16_t)fireHeat[base + i - 2]) / 3;
            }
            if(random8() < 120) {
              uint8_t y = random8(3);
              fireHeat[base + y] = qadd8(fireHeat[base + y], random8(160, 255));
            }
          }
          for(int i = 0; i < earLedsNum; i++) {
            earLeds[i] = HeatColor(fireHeat[i]);
          }
        }
        FdisplayEar = true;
      }
    }
  }

  //--------------------------------//VISOR+BLUSH Leds render
  if((visorNow->type == 0 || (visorNow->type == 1 && visorType == "MAX72XX")) && visorNow->frames != nullptr) { //custom
    if(lastMillsVisor+visorNow->frames[currentVisorFrame].timespan <= millis() || instantReload) {
      currentVisorFrame++;
      lastMillsVisor = millis();
      if(currentVisorFrame == visorNow->numOfFrames) { currentVisorFrame = 0; }
      setAllVisor(visorLedsNEW,0,currentVisorFrame,true); //set visor leds (newFrame=true resets fade)
      if(blushPresent) {
        for(int x = 0; x<blushLedsNum; x++) { blushLeds[x] = visorNow->frames[currentVisorFrame].ledsBlush[x]; } //set blush leds
        FdisplayBlush = true;
      }
      instantReload = false;
    }
  } else if (visorNow->type == 1 && visorType == "WS2812" && visorNow->frames != nullptr) { //all_rainbow
    if(lastMillsVisor+visorNow->frames[currentVisorFrame].timespan <= millis() || instantReload) {
      currentVisorFrame++;
      lastMillsVisor = millis();
      if(currentVisorFrame == visorNow->numOfFrames) { currentVisorFrame = 0; }
      instantReload = false;
    }
    fill_rainbow(visorPixelBuffer, 1, millis()/cfg.rbSpeed, 128/cfg.rbWidth);
    setAllVisor(visorLeds,((long)visorPixelBuffer[0].r << 16) | ((long)visorPixelBuffer[0].g << 8 ) | (long)visorPixelBuffer[0].b,currentVisorFrame);
    if(blushPresent) {
      for(int x = 0; x<blushLedsNum; x++) { blushLeds[x] = visorNow->frames[currentVisorFrame].ledsBlush[x]; } //set blush leds
      FdisplayBlush = true;
    }
  }

  //--------------------------------//TILT
  if(lastMillsTilt+100<=millis() && cfg.tiltEna) {
    if(isApproxEqual(myIMU.readFloatAccelX(),myIMU.readFloatAccelY(),myIMU.readFloatAccelZ(),cfg.upX,cfg.upY,cfg.upZ,cfg.tiltTol) && !wasTilt) {
      logPrint(F("[I] Tilt: UP!"));
      wasTilt = true;
      oldanim = currentAnim;
      tiltChange = millis();
      loadAnim(cfg.aUp,"");
    } else if (isApproxEqual(myIMU.readFloatAccelX(),myIMU.readFloatAccelY(),myIMU.readFloatAccelZ(),cfg.tiltX,cfg.tiltY,cfg.tiltZ,cfg.tiltTol) && !wasTilt) {
      logPrint(F("[I] Tilt: Side!"));
      wasTilt = true;
      oldanim = currentAnim;
      tiltChange = millis();
      loadAnim(cfg.aTilt,"");
    } else if ((tiltChange+revertTilt<millis() || isApproxEqual(myIMU.readFloatAccelX(),myIMU.readFloatAccelY(),myIMU.readFloatAccelZ(),cfg.neutralX,cfg.neutralY,cfg.neutralZ,cfg.tiltTol)) && wasTilt) {
      logPrint(F("[I] Tilt: Neutral!"));
      wasTilt = false;
      loadAnim(oldanim,"");
    }
    lastMillsTilt = millis();
  } else if (!tiltInitDone && cfg.tiltEna) {
    if(myIMU.begin()) {
      logPrint(F("[E] An Error has occurred while connecting to LSM!"));
      cfg.tiltEna = false;
    } else {
      tiltInitDone = true;
    }
  }

  //looptime = micros();
  //--------------------------------//SPEECH Detection
  if(cfg.speechEna) { //1.045uS
    int nvol = 0, micline = 0, rawInput = 0;
    for (int i = 0; i<32; i++){
      adc_oneshot_read(adc_handle, MICpin, &rawInput);
      micline = abs(rawInput - 512);
      nvol = max(micline, nvol);
    }
    if(currentMicAvg == 9) {
      currentMicAvg = 0;
    } else {
      avgMicArr[currentMicAvg++] = nvol;
    }

    if(laskSpeakCheck+10<=millis()) {
      finalMicAvg = 0;
      for (int i = 0; i<10; i++){
        finalMicAvg+=avgMicArr[i];
      }
      finalMicAvg = finalMicAvg/10;
      //Serial.print(String(finalMicAvg));
      //Serial.print(",");
      float centered = finalMicAvg - micDC; //DC removal
      float mag = fabsf(centered);
      if (mag > env) { // Envelope follower
        env += (mag - env) * micAttack;
      } else {
        env += (mag - env) * micRelease;
      }
      if (env < 25.0f) { // Noise gate
        env = 0.0f;
      }
      if (env < 25.0f) { // Update DC only when quiet
          micDC = micDC * (1.0f - 0.0005f) + finalMicAvg * 0.0005f;
      }
      micVolume = (int)(env * 100.0f / (float)cfg.spMax + 0.5f); // Normalize 0-100
      micVolume = constrain(micVolume, 0, 100);
      //Serial.println(String(micVolume));

      if(micVolume > cfg.spTrig) {
        if(!speaking) {
          speaking = true;
          if(cfg.oledEna && oledInitDone) {
            oled.speak(true);
          }
          logPrint(F("[I] Speak"));
        }
      } else {
        if(speaking) {
          speaking = false;
          if(cfg.oledEna && oledInitDone) {
            oled.speak(false);
          }
          logPrint(F("[I] unSpeak"));
        }
      }

      laskSpeakCheck = millis();
    }

    if(laskSpeakAnim+60<=millis() && speaking) {
      if((visorNow->type == 0 || (visorNow->type == 1 && visorType == "MAX72XX")) && visorNow->frames != nullptr) { //custom
        setAllVisor(visorLedsNEW,0,currentVisorFrame);
        laskSpeakAnim = millis();
      }
    }
  }
  //Serial.println(">SPK1:"+String(micros()-looptime));

  //--------------------------------//Single button anim change
  hwBtn.loop();
  if(hwBtn.isPressed()) { //detect press
    btnPressTime = millis();
  }
  if(hwBtn.isReleased()) {
    if(millis()-btnPressTime < 1500) { //short press
      btnNum++;
      if(btnNum >= totalAnims) {
        btnNum = 0;
      } else {
        animToLoad = availAnims[btnNum];
      }
      logPrint("Changing to "+availAnims[btnNum]+", amount of anims: "+String(totalAnims));
    }
  }

  //looptime = micros();
  //--------------------------------//BOOP Detection; 14-800uS
  if(lastBoopCheck+100<=millis() && cfg.boopEna) {
    if(boopMode == "KY-032") {
      digitalWrite(T_en, HIGH);
      delayMicroseconds(210);
      if(booping == false && !digitalRead(T_in)) {
        delayMicroseconds(395);
        if(!digitalRead(T_in)) {
          logPrint(F("[I] IR BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        }
        digitalWrite(T_en, LOW);
      } else if(booping == true && lastMillsBoop+1000<millis() && digitalRead(T_in)) {
        digitalWrite(T_en, LOW);
        logPrint(F("[I] IR unBOOP"));
        booping = false;
        if(!wasTilt) {
          loadAnim(boopoldanim,"");
        }
      }
    } else if (boopMode == "Capac") {
      if(booping == false && digitalRead(T_in)) {
        delayMicroseconds(395);
        if(digitalRead(T_in)) {
          logPrint(F("[I] Touch BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        }
      } else if(booping == true && lastMillsBoop+1000<millis() && !digitalRead(T_in)) {
        logPrint(F("[I] Touch unBOOP"));
        booping = false;
        if(!wasTilt) {
          loadAnim(boopoldanim,"");
        }
      }
    } else if (boopMode == "APDS9960") {
      if(ToFInitDone) {
        apdsprox = apds.readProximity();
        //Serial.println(String(apdsprox));
        if(booping == false && (255 - apdsprox) < cfg.boopThresh) {
          logPrint(F("[I] ToF BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        } else if(booping == true && lastMillsBoop+1000<millis() && (255 - apdsprox) > cfg.boopThresh) {
          logPrint(F("[I] ToF unBOOP"));
          booping = false;
          if(!wasTilt) {
            loadAnim(boopoldanim,"");
          }
        }
      } else {
        if(!apds.begin()){
          cfg.boopEna = false;
          logPrint(F("[E] An Error has occurred while initializing APDS9960 chip!"));
        } else {
          ToFInitDone = true;
          apds.enableProximity(true);
        }
      }
    } else if (boopMode == "VL53L1X") {
      if(ToFInitDone) {
        int16_t distance = -1;
        bool dr = vl53.dataReady();
        // Serial.print("[ToF] dataReady="); Serial.print(dr);
        if (dr) {
          distance = vl53.distance();
          lastToFDistance = distance; //cache for /tof endpoint
          // Serial.print(" dist="); Serial.println(distance);
        } else {
          // Serial.println(" (no new data)");
        }
        if(booping == false && distance != -1 && distance < cfg.boopThresh) {
          logPrint(F("[I] ToF BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        } else if(booping == true && lastMillsBoop+1000<millis() && (lastToFDistance == -1 || lastToFDistance > cfg.boopThresh)) {
          logPrint(F("[I] ToF unBOOP"));
          booping = false;
          if(!wasTilt) {
            loadAnim(boopoldanim,"");
          }
        }
      } else {
        Wire.beginTransmission(0x29);
        if(Wire.endTransmission() != 0) {
          cfg.boopEna = false;
          logPrint(F("[E] An Error has occurred while finding VL53L1X chip!"));
        } else {
          if(!vl53.begin()){
            cfg.boopEna = false;
            logPrint(F("[E] An Error has occurred while initializing VL53L1X chip!"));
          } else {
            if (!vl53.startRanging()) {
              cfg.boopEna = false;
              logPrint(F("[E] An Error has occurred while starting ranging with VL53L1X chip!"));
            } else {
              ToFInitDone = true;
              vl53.setTimingBudget(50);
              vl53.VL53L1X_SetROI(cfg.toFov, cfg.toFov);
            }
          }
        }
      }
    }
    lastBoopCheck=millis();
  }
  //Serial.println(">BP:"+String(micros()-looptime));

  //--------------------------------//OLED routine, ~~10ms qwq~~, 1-5ms.. eh better
  if(cfg.oledEna && oledInitDone && vaStatLast+1000<millis()) {
    //looptime = micros();
    if(INApresent) {
      float busVolt = ina219.getBusVoltage_V();
      float busCurr = ina219.getCurrent_mA();
      // Compensate for internal resistance voltage sag: estimate what voltage would be at rest
      float compVolt = busVolt + (busCurr / 1000.0f) * BATT_INT_RESISTANCE_OHMS;
      int battPct = batteryPercent(compVolt);
      bool belowThreshold = (compVolt > 0.5f && compVolt < LOW_BATT_THRESHOLD);
      if(belowThreshold) {
        lowBattConsecutiveCount++;
      } else {
        lowBattConsecutiveCount = 0;
      }
      bool triggerLowBatt = (lowBattConsecutiveCount >= LOW_BATT_CONSECUTIVE_SECS);
      if(triggerLowBatt) {
        if(!lowBattActive) { // first time crossing the threshold
          lowBattActive = true;
          if(cfg.lowBattSwitch) { // only switch animation if the toggle is enabled
            preLowBattAnim = (currentAnim != "low batt.json") ? currentAnim : "default.json";
            logPrint(F("[W] Low battery! Switching to low batt animation."));
            loadAnim("low batt.json", "");
          } else {
            logPrint(F("[W] Low battery! (anim switch disabled)"));
          }
        }
        lowBattFlash = !lowBattFlash; // flash "!! LOW BATT !!" / voltage alternately every second
        if(lowBattFlash) {
          oled.writeLowBatt(true);
        } else {
          oled.writeINA(busVolt, busCurr, battPct);
        }
        logPrint("[W] Low battery: "+String(busVolt,2)+"V (comp: "+String(compVolt,2)+"V, "+String(battPct)+"%%)");
      } else {
        if(lowBattActive) { // battery recovered (pack swapped) — restore previous animation
          lowBattActive = false;
          lowBattConsecutiveCount = 0;
          if(cfg.lowBattSwitch) {
            logPrint(F("[I] Battery recovered, restoring previous animation."));
            loadAnim(preLowBattAnim, "");
          } else {
            logPrint(F("[I] Battery recovered."));
          }
        }
        lowBattFlash = false;
        oled.writeINA(busVolt, busCurr, battPct);
      }
    }
    if(cfg.bleEna) {
      if(pServer->getConnectedCount() == 0) {
        remoteSign = !remoteSign;
        oled.remote(remoteSign);
      } else if (pServer->getConnectedCount() > 0 && remoteSign == false) {
        oled.remote(true);
        remoteSign = true;
      }
    } else if (!cfg.bleEna && remoteSign) {
      oled.remote(false);
      remoteSign = false;
    }
    vaStatLast = millis();
    //Serial.println(">OLED:"+String(micros()-looptime));
  }
  if (!oledInitDone && cfg.oledEna) {
    if(!oled.init(oledAddr,cfg.bOled,INApresent)) {
      logPrint(F("[E] An Error has occurred while initializing SSD1306."));
      cfg.oledEna = false;
    } else {
      oledInitDone = true;
    }
  }

  if((FdisplayEar || FdisplayBlush || FdisplayVisor) ) {
    // -- Prepare ear/blush data into c2Leds before the combined FastLED.show()
    if(earPresent && !blushPresent && FdisplayEar) {
      for(int i = 0; i < earLedsNum; i++) { c2Leds[i] = earLeds[i]; }
    } else if (blushPresent && !earPresent && FdisplayBlush) {
      for(int i = 0; i < blushLedsNum; i++) { c2Leds[i] = blushLeds[i]; }
    } else if (blushPresent && earPresent && (FdisplayBlush || FdisplayEar)) { //ear-blush-ear
      for(int i = 0; i < (earLedsNum/2); i++) { c2Leds[i] = earLeds[i]; }
      for(int i = (earLedsNum/2); i < (earLedsNum/2)+blushLedsNum; i++) { c2Leds[i] = blushLeds[i-(earLedsNum/2)]; }
      for(int i = (earLedsNum/2)+blushLedsNum; i < earLedsNum+blushLedsNum; i++) { c2Leds[i] = earLeds[i-blushLedsNum]; }
    }
    if(blushPresent && useRGBblush) {
      for(int i = 0; i < blushLedsNum; i++) {
        uint32_t temp = blushLeds[i].r;
        blushLeds[i].r = blushLeds[i].g;
        blushLeds[i].g = temp;
      }
    }

    if(visorType == "WS2812") {
      if(FdisplayVisor) {
        if(visorNow->type == 0) {
          if(FADESTEPS == 0) { //no fading, copy directly
            memcpy(visorLeds, visorLedsNEW, sizeof(CRGB) * visorLedsNum);
            // Show each controller with its own brightness to decouple visor and ear brightness
            ledController[0]->showLeds(flashlightMode ? 255 : cfg.bVisor);
            ledController[1]->showLeds(flashlightMode ? 255 : cfg.bEar);
            ledController[2]->showLeds(flashlightMode ? 255 : cfg.bVisor);
            FdisplayVisor = false;
            FdisplayEar = false;
            FdisplayBlush = false;
          } else if(fadeTime + FADE_INTERVAL_MS < millis()) {
            for (uint16_t i = 0; i < visorLedsNum; i++) {
              visorLeds[i] = blend(visorLeds[i], visorLedsNEW[i], (currFade * 255) / FADESTEPS);
            }
            ledController[0]->showLeds(flashlightMode ? 255 : cfg.bVisor);
            ledController[1]->showLeds(flashlightMode ? 255 : cfg.bEar);
            ledController[2]->showLeds(flashlightMode ? 255 : cfg.bVisor);
            fadeTime = millis();
            currFade++;
            if(currFade > FADESTEPS) {
              memcpy(visorLeds, visorLedsNEW, sizeof(CRGB) * visorLedsNum);
              FdisplayVisor = false;
              FdisplayEar = false;
              FdisplayBlush = false;
              currFade = 1;
            }
          }
        } else { //all_rainbow
          ledController[0]->showLeds(flashlightMode ? 255 : cfg.bVisor);
          ledController[1]->showLeds(flashlightMode ? 255 : cfg.bEar);
          ledController[2]->showLeds(flashlightMode ? 255 : cfg.bVisor);
          FdisplayVisor = false;
          FdisplayEar = false;
          FdisplayBlush = false;
        }
      } else if(FdisplayEar || FdisplayBlush) {
        // Ears/blush changed but visor didn't — update ears controller only with ear brightness
        ledController[1]->showLeds(flashlightMode ? 255 : cfg.bEar);
        FdisplayEar = false;
        FdisplayBlush = false;
      }
    } else if (visorType == "MAX72XX") {
      if(FdisplayVisor) {
        int effectiveBVisor = flashlightMode ? 15 : cfg.bVisor;
        if(effectiveBVisor > 15) { effectiveBVisor = 15;}
        mx.control(MD_MAX72XX::INTENSITY, effectiveBVisor);
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
        FdisplayVisor = false;
      }
      // For MAX72XX visor, ears/blush use their own controller
      if(FdisplayEar || FdisplayBlush) {
        ledController[1]->showLeds(flashlightMode ? 255 : cfg.bEar);
        FdisplayEar = false;
        FdisplayBlush = false;
      }
    }
  }

  //press boot button for 10sec to reset
  if(check0button+10000 < millis() && check0button+10500 > millis() && digitalRead(0) == LOW) {
    logPrint(F("[I] Resetting to defaults"));
    cfg.setDefault();
    delay(20);
    ESP.restart();
  } else if (digitalRead(0) == HIGH && check0button+10000 > millis() && check0button < millis()) {
    check0button = 0;
  } else if (digitalRead(0) == LOW && check0button+10000 < millis() && check0button < millis()) {
    check0button = millis();
  }

  if(animToLoad != "") {
    String toLoad = animToLoad;
    animToLoad = "";
    if(toLoad.startsWith("POST:")) {
      loadAnim("POSTAnimLoad", toLoad.substring(5));
    } else {
      loadAnim(toLoad, "");
    }
    wasTilt = false;
    booping = false;
  }
}
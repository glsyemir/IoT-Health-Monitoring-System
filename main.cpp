#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

// ====== Wi‑Fi + HTTP ======
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include <time.h>   // NTP timestamp

// -------- Wi‑Fi / Server Config --------
const char* WIFI_SSID     = "wifi name";
const char* WIFI_PASS     = "password";
const char* SERVER_URL    = "http://192.168.43.245:8080"; // Raspberry Pi 5 listener
const char* NODE_ID       = "Node5";                      // make unique per device
const uint32_t SEND_PERIOD_MS = 2000;                     // JSON POST period

// NTP (Türkiye GMT+3)
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3 * 3600;
const int   daylightOffset_sec = 0;

// ---- Forward declarations ----
void processSample(uint32_t irRaw, uint32_t redRaw);
int  bpmToInt(float x);
String getTimestamp();

// ---- PPG ----
MAX30105 ppg;

// ---- ENV SENSORS ----
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pins (NodeMCU): D5=GPIO14, D6=GPIO12
#define DHT_PIN   D5
#define DHT_TYPE  DHT11
#define DS_PIN    D6

DHT dht(DHT_PIN, DHT_TYPE);
OneWire oneWire(DS_PIN);
DallasTemperature ds(&oneWire);

// Value buffers
float dhtTemp = NAN, dhtHum = NAN;
float dsTemp  = DEVICE_DISCONNECTED_C;

// Timers
unsigned long nextDhtMs   = 0;
unsigned long dsReadyAt   = 0;

// ---- User Settings ----
#define USE_MAX3010X   1
#define USE_DHT        1
#define USE_DS18B20    1

// MAX3010x config
const byte  ledBrightness = 0x2A;
const byte  sampleAverage = 8;
const byte  ledMode       = 2;
const byte  sampleRate    = 100;
const int   pulseWidth    = 411;
const int   adcRange      = 16384;

// Filters / HR
const float emaAlphaDC    = 0.95f;
const float emaAlphaAC    = 0.70f;
const int   avgWinHR      = 5;
const int   avgWinR       = 8;
const float thrFactor     = 0.55f;
const uint32_t minBeatMs  = 300;
const uint32_t maxBeatMs  = 2000;

inline float spo2FromR(float R) { return 110.0f - 25.0f * R; }

struct EMA { float y=NAN,a; EMA(float alpha):a(alpha){} inline float step(float x){ if(isnan(y)) y=x; y=a*y+(1-a)*x; return y; } };
template<int N>
struct MovAvg { float buf[N]; int idx=0,cnt=0; MovAvg(){for(int i=0;i<N;i++) buf[i]=0;} inline float push(float v){ buf[idx]=v; idx=(idx+1)%N; if(cnt<N) cnt++; float s=0; for(int i=0;i<cnt;i++) s+=buf[i]; return s/(float)cnt; } };

EMA dcIR(emaAlphaDC), dcRED(emaAlphaDC);
EMA acIR(emaAlphaAC),  acRED(emaAlphaAC);
MovAvg<avgWinHR>  hrAvg;
MovAvg<avgWinR>   rAvg;
uint32_t lastBeatMs = 0;
float bpmInstant = NAN;
EMA acRmsIR(0.90f), acRmsRED(0.90f);

// “latest” values to send
volatile uint32_t latestIR  = 0;
volatile uint32_t latestRED = 0;
volatile int      latestBPM = 0;
volatile int      latestSpO2= 0;

// send timer
uint32_t lastSendMs = 0;

// ---- ENV services (non-blocking) ----
void serviceEnvSensors(){
#if USE_DHT
  unsigned long now = millis();
  if (now >= nextDhtMs) {
    float t = dht.readTemperature();  // °C
    float h = dht.readHumidity();     // %
    if (!isnan(t)) dhtTemp = t;
    if (!isnan(h)) dhtHum  = h;
    nextDhtMs = now + 2000;           // 2s
  }
#endif

#if USE_DS18B20
  unsigned long now2 = millis();
  if (now2 >= dsReadyAt) {
    float tC = ds.getTempCByIndex(0);
    if (tC != DEVICE_DISCONNECTED_C) dsTemp = tC;
    ds.requestTemperatures();
    dsReadyAt = now2 + 750;           // 12-bit conversion time
  }
#endif
}

// ---- Non-blocking Wi‑Fi reconnect ----
void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 5000) { // try every 5s
      Serial.println(F("[Wi‑Fi] Not connected, retrying..."));
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      lastAttempt = millis();
    }
  }
}

// ---- Timestamp helper (YYYY-MM-DD HH:MM:SS) ----
String getTimestamp() {
  struct tm tm_info;
  if (!getLocalTime(&tm_info)) {
    return "1970-01-01 00:00:00";  // before NTP sync
  }
  char buf[20];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_info);
  return String(buf);
}

// ---- POST JSON to Raspberry Pi ----
void sendIfDue() {
  uint32_t now = millis();
  if (now - lastSendMs < SEND_PERIOD_MS) return;
  lastSendMs = now;

  ensureWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("[POST] Skipped (no Wi‑Fi)"));
    return;
  }

  // snapshot values
  uint32_t IR   = latestIR;
  uint32_t RED  = latestRED;
  int BPM       = latestBPM;
  int SPO2      = latestSpO2;
  float tDHT    = dhtTemp;
  float hDHT    = dhtHum;
  float tDS     = dsTemp;

  String ts = getTimestamp();

  // build JSON manually
  String payload = "{";
  payload += "\"NodeID\":\""; payload += NODE_ID; payload += "\",";
  payload += "\"Timestamp\":\""; payload += ts; payload += "\",";
  payload += "\"IR\":";       payload += IR;      payload += ",";
  payload += "\"RED\":";      payload += RED;     payload += ",";
  payload += "\"BPM\":";      payload += BPM;     payload += ",";
  payload += "\"SpO2\":";     payload += SPO2;    payload += ",";

  payload += "\"TempDHT\":";
  if (isnan(tDHT)) payload += "null"; else payload += String(tDHT, 1);
  payload += ",";

  payload += "\"Humidity\":";
  if (isnan(hDHT)) payload += "null"; else payload += String((int)round(hDHT));
  payload += ",";

  payload += "\"TempDS18B20\":";
  if (tDS == DEVICE_DISCONNECTED_C) payload += "-127"; else payload += String(tDS, 1);

  payload += "}";

  HTTPClient http;
  WiFiClient client;
  http.setTimeout(3000);
  if (!http.begin(client, SERVER_URL)) {
    Serial.println(F("[POST] begin() failed"));
    return;
  }
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(payload);
  Serial.print(F("[POST] code=")); Serial.println(code);
  if (code > 0) {
    String resp = http.getString();
    Serial.print(F("[POST] resp=")); Serial.println(resp);
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(D2, D1);                // SDA=D2(GPIO4), SCL=D1(GPIO5)
  Wire.setClock(400000);

  // Wi‑Fi (start, but do not block)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // NTP (no blocking wait)
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

#if USE_MAX3010X
  if (!ppg.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX3010x not found!"));
    while(1) delay(10);
  }
  ppg.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  ppg.enableFIFORollover();
  ppg.setFIFOAverage(sampleAverage);
#endif

#if USE_DHT
  dht.begin();
#endif

#if USE_DS18B20
  ds.begin();
  ds.setResolution(12);
  ds.setWaitForConversion(false);
  ds.requestTemperatures();
  dsReadyAt = millis() + 750;
#endif

  Serial.println(F("Setup done."));
}

void loop() {
  serviceEnvSensors();

#if USE_MAX3010X
  if (!ppg.check()) {
    sendIfDue();
    delay(1);
    return;
  }
  while (ppg.available()) {
    uint32_t ir  = ppg.getIR();
    uint32_t red = ppg.getRed();
    ppg.nextSample();
    processSample(ir, red);
  }
#else
  delay(5);
#endif

  sendIfDue();
}

void processSample(uint32_t irRaw, uint32_t redRaw) {
  static EMA dcIR(emaAlphaDC), dcRED(emaAlphaDC);
  static EMA acIR(emaAlphaAC),  acRED(emaAlphaAC);
  static EMA acRmsIR(0.90f),    acRmsRED(0.90f);
  static MovAvg<avgWinHR> hrAvg;
  static MovAvg<avgWinR>  rAvg;

  float dc_ir  = dcIR.step((float)irRaw);
  float dc_red = dcRED.step((float)redRaw);
  float ac_ir  = acIR.step((float)irRaw  - dc_ir);
  float ac_red = acRED.step((float)redRaw - dc_red);

  float rmsIR  = acRmsIR.step(fabsf(ac_ir));
  float rmsRED = acRmsRED.step(fabsf(ac_red));

  float thr = thrFactor * rmsIR;
  uint32_t now = millis();
  static bool wasAbove = false;
  bool above = (ac_ir > thr);
  bool risingEdge = (above && !wasAbove && ac_ir > 0);
  wasAbove = above;

  if (risingEdge) {
    static uint32_t lastBeatMs = 0;
    uint32_t dt = now - lastBeatMs;
    if (dt > minBeatMs && dt < maxBeatMs) {
      float bpm = 60000.0f / (float)dt;
      bpmInstant = hrAvg.push(bpm);
    }
    lastBeatMs = now;
  }

  float ratio_ir  = (dc_ir  > 1.0f) ? (fabsf(ac_ir)  / dc_ir)  : 0.0f;
  float ratio_red = (dc_red > 1.0f) ? (fabsf(ac_red) / dc_red) : 0.0f;
  float R = (ratio_ir > 1e-6f) ? (ratio_red / ratio_ir) : NAN;
  static float R_smooth = NAN;
  if (!isnan(R)) R_smooth = rAvg.push(R);

  float spo2 = NAN;
  if (!isnan(R_smooth)) {
    spo2 = spo2FromR(R_smooth);
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 70.0f)  spo2 = 70.0f;
  }

  // Debug line (Serial Plotter friendly)
  Serial.print(F("[DBG]: IR="));   Serial.print(irRaw);
  Serial.print(F(" RED="));        Serial.print(redRaw);
  Serial.print(F(" HP="));         Serial.print(ac_ir, 2);
  Serial.print(F(" thr="));        Serial.print(thr, 2);
  Serial.print(F(" RMS_IR="));     Serial.print(rmsIR,2);
  Serial.print(F(" RMS_RED="));    Serial.print(rmsRED,2);
  Serial.print(F(" BPM="));        Serial.print(isnan(bpmInstant)?0.0f:bpmInstant, 1);
  Serial.print(F(" SPO2="));       Serial.println(isnan(spo2)?0.0f:spo2, 1);

  latestIR   = irRaw;
  latestRED  = redRaw;
  latestBPM  = bpmToInt(bpmInstant);
  latestSpO2 = isnan(spo2) ? 0 : (int)(spo2 + 0.5f);
}

int bpmToInt(float x){
  if (isnan(x) || x<20 || x>220) return 0;
  return (int)(x+0.5f);
}


#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFiClientSecure.h>

#define TRIG_PIN     5
#define ECHO_PIN     18
#define ONE_WIRE_BUS 4
#define FLOW_IN_PIN  25
#define RELAY_PIN    13
#define FLOW_OUT_PIN 26
#define TURB_PIN     34
#define PH_PIN       35

const char* SERVER_URL = "https://smart-water-tank-api.onrender.com/sensor-reading";

float EMPTY_DISTANCE_CM = 44.0;
float FULL_DISTANCE_CM  = 5.0;
float TANK_CAPACITY_L   = 650.0;

//WATER QUALITY CALIBRATION
float PH_SLOPE  = -5.70;
float PH_OFFSET = 21.34;
const float TURB_CLEAR_RAW = 2108.0;
const float TURB_DIRTY_RAW = 1740.0;
const float TURB_MAX_NTU   = 100.0;

const float PH_MIN = 6.5, PH_MAX = 8.5;
const float TURBIDITY_MAX_NTU = 5.0;
const float PUMP_ON_LEVEL  = 20.0;
const float PUMP_OFF_LEVEL = 90.0;

// ---------------- FLOW ----------------
volatile int pulseIn_count = 0, pulseOut_count = 0;
float flowRateIn = 0, flowRateOut = 0;
float totalLitresIn = 0, totalLitresOut = 0;
unsigned long lastFlowCalc = 0;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
bool pumpStatus = false;

Preferences prefs;       // stores Wi-Fi credentials
Preferences calPrefs;    // stores tank calibration
WebServer configServer(80);  // Wi-Fi setup portal
WebServer calServer(81);     // tank calibration endpoints
String savedSSID, savedPASS;

void IRAM_ATTR pulseInISR()  { pulseIn_count++;  }
void IRAM_ATTR pulseOutISR() { pulseOut_count++; }

// Forward declaration (readDistance needs to exist before setupCalibrationEndpoints uses it)
float readDistance(float tempC);

// ============================================================
//  CALIBRATION STORAGE
// ============================================================
void loadCalibration() {
  calPrefs.begin("tankcal", false);
  EMPTY_DISTANCE_CM = calPrefs.getFloat("empty", 44.0);
  FULL_DISTANCE_CM  = calPrefs.getFloat("full", 5.0);
  TANK_CAPACITY_L   = calPrefs.getFloat("capacity", 650.0);
  calPrefs.end();
  Serial.printf("Loaded calibration - Empty:%.1fcm Full:%.1fcm Capacity:%.0fL\n",
                EMPTY_DISTANCE_CM, FULL_DISTANCE_CM, TANK_CAPACITY_L);
}

void saveCalibration() {
  calPrefs.begin("tankcal", false);
  calPrefs.putFloat("empty", EMPTY_DISTANCE_CM);
  calPrefs.putFloat("full", FULL_DISTANCE_CM);
  calPrefs.putFloat("capacity", TANK_CAPACITY_L);
  calPrefs.end();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("========================================");
  Serial.println("  SMART WATER TANK - FULL SYSTEM");
  Serial.println("========================================");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(FLOW_IN_PIN, INPUT_PULLUP);
  pinMode(FLOW_OUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_IN_PIN),  pulseInISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_OUT_PIN), pulseOutISR, RISING);

  analogReadResolution(12);
  analogSetPinAttenuation(TURB_PIN, ADC_11db);
  analogSetPinAttenuation(PH_PIN,   ADC_11db);

  tempSensor.begin();
  loadCalibration();          // load saved tank calibration (or defaults)
  connectWiFi();               // app-driven Wi-Fi setup / reconnect
  setupCalibrationEndpoints(); // start the tank calibration web server

  Serial.println("Sensors initialised. Starting readings...\n");
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  calServer.handleClient();   // MUST be called every loop or calibration won't respond

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C) tempC = 25.0;

  float distance = readDistance(tempC);
  float levelPct = 0;
  if (distance >= 0) {
    levelPct = (EMPTY_DISTANCE_CM - distance) /
               (EMPTY_DISTANCE_CM - FULL_DISTANCE_CM) * 100.0;
    levelPct = constrain(levelPct, 0, 100);
  }
  float availableLitres = (levelPct / 100.0) * TANK_CAPACITY_L;

  updateFlow();

  float phValue   = readPH();
  float turbidity = readTurbidity();
  bool  qualitySafe = (phValue >= PH_MIN && phValue <= PH_MAX &&
                       turbidity <= TURBIDITY_MAX_NTU);

  // EDGE FAIL-SAFE PUMP LOGIC - runs locally, always
  if (!qualitySafe)                    pumpOff();
  else if (levelPct < PUMP_ON_LEVEL)   pumpOn();
  else if (levelPct > PUMP_OFF_LEVEL)  pumpOff();

  Serial.println("---------------------------------------");
  Serial.printf("Temperature : %.1f C\n", tempC);
  Serial.printf("Distance    : %.1f cm\n", distance);
  Serial.printf("Tank Level  : %.1f %% (%.0f L)\n", levelPct, availableLitres);
  Serial.printf("Flow In     : %.2f L/min\n", flowRateIn);
  Serial.printf("Flow Out    : %.2f L/min\n", flowRateOut);
  Serial.printf("pH Value    : %.2f\n", phValue);
  Serial.printf("Turbidity   : %.1f NTU\n", turbidity);
  Serial.printf("Pump        : %s (%s)\n", pumpStatus ? "ON" : "OFF",
                qualitySafe ? "SAFE" : "UNSAFE");

  if (millis() - lastSend >= SEND_INTERVAL) {
    sendData(levelPct, flowRateOut, totalLitresOut, phValue, turbidity, pumpStatus);
    lastSend = millis();
  }

  delay(1000);
}

// ============================================================
//  SENSORS
// ============================================================
float readDistance(float tempC) {
  unsigned long waitStart = millis();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (millis() - waitStart > 50) break;
  }
  float speed = 331.4 + 0.6 * tempC;
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(20);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) {
    Serial.println("  [warning] no echo received this cycle");
    return -1;
  }
  return (dur * (speed / 10000.0)) / 2.0;
}

void updateFlow() {
  if (millis() - lastFlowCalc >= 1000) {
    detachInterrupt(digitalPinToInterrupt(FLOW_IN_PIN));
    detachInterrupt(digitalPinToInterrupt(FLOW_OUT_PIN));
    flowRateIn  = pulseIn_count  / 7.5;
    flowRateOut = pulseOut_count / 7.5;
    totalLitresIn  += flowRateIn  / 60.0;
    totalLitresOut += flowRateOut / 60.0;
    pulseIn_count = 0; pulseOut_count = 0;
    lastFlowCalc = millis();
    attachInterrupt(digitalPinToInterrupt(FLOW_IN_PIN),  pulseInISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(FLOW_OUT_PIN), pulseOutISR, RISING);
  }
}

float readPH() {
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(PH_PIN); delay(5); }
  float voltage = (sum / 20.0) * (3.3 / 4095.0);
  return PH_SLOPE * voltage + PH_OFFSET;
}

float readTurbidity() {
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogRead(TURB_PIN); delay(5); }
  float raw = sum / 20.0;
  float ntu = (TURB_CLEAR_RAW - raw) / (TURB_CLEAR_RAW - TURB_DIRTY_RAW) * TURB_MAX_NTU;
  return constrain(ntu, 0, TURB_MAX_NTU);
}

// ============================================================
//  PUMP CONTROL
// ============================================================
void pumpOn()  { digitalWrite(RELAY_PIN, HIGH); pumpStatus = true;  }
void pumpOff() { digitalWrite(RELAY_PIN, LOW);  pumpStatus = false; }

// ============================================================
//  TANK CALIBRATION ENDPOINTS (app talks to ESP32 on port 81)
// ============================================================
void setupCalibrationEndpoints() {
  calServer.on("/calibrate-empty", HTTP_POST, []() {
    tempSensor.requestTemperatures();
    float tempC = tempSensor.getTempCByIndex(0);
    if (tempC == DEVICE_DISCONNECTED_C) tempC = 25.0;
    float d = readDistance(tempC);
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    if (d >= 0) {
      EMPTY_DISTANCE_CM = d;
      saveCalibration();
      calServer.send(200, "application/json", "{\"empty_distance_cm\":" + String(d, 1) + "}");
    } else {
      calServer.send(500, "application/json", "{\"error\":\"sensor read failed\"}");
    }
  });

  calServer.on("/calibrate-full", HTTP_POST, []() {
    tempSensor.requestTemperatures();
    float tempC = tempSensor.getTempCByIndex(0);
    if (tempC == DEVICE_DISCONNECTED_C) tempC = 25.0;
    float d = readDistance(tempC);
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    if (d >= 0) {
      FULL_DISTANCE_CM = d;
      saveCalibration();
      calServer.send(200, "application/json", "{\"full_distance_cm\":" + String(d, 1) + "}");
    } else {
      calServer.send(500, "application/json", "{\"error\":\"sensor read failed\"}");
    }
  });

  calServer.on("/set-capacity", HTTP_POST, []() {
    String body = calServer.arg("plain");
    int start = body.indexOf("\"capacity\":") + 11;
    int end = body.indexOf(",", start);
    if (end == -1) end = body.indexOf("}", start);
    TANK_CAPACITY_L = body.substring(start, end).toFloat();
    saveCalibration();
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    calServer.send(200, "application/json", "{\"status\":\"saved\"}");
  });

  calServer.on("/calibration-status", HTTP_GET, []() {
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{\"empty_distance_cm\":" + String(EMPTY_DISTANCE_CM, 1) +
                  ",\"full_distance_cm\":" + String(FULL_DISTANCE_CM, 1) +
                  ",\"capacity_l\":" + String(TANK_CAPACITY_L, 0) + "}";
    calServer.send(200, "application/json", json);
  });

  calServer.on("/calibrate-empty", HTTP_OPTIONS, []() {
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    calServer.sendHeader("Access-Control-Allow-Methods", "POST, GET");
    calServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    calServer.send(200);
  });
  calServer.on("/calibrate-full", HTTP_OPTIONS, []() {
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    calServer.sendHeader("Access-Control-Allow-Methods", "POST, GET");
    calServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    calServer.send(200);
  });
  calServer.on("/set-capacity", HTTP_OPTIONS, []() {
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    calServer.sendHeader("Access-Control-Allow-Methods", "POST, GET");
    calServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    calServer.send(200);
  });

  calServer.onNotFound([]() {
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    calServer.send(404);
  });

  calServer.begin();
  Serial.println("Tank calibration server started on port 81");
}

// ============================================================
//  WIFI - App-driven credential setup (no hardcoded SSID/password)
// ============================================================
void loadWiFiCredentials() {
  prefs.begin("wifi", false);
  savedSSID = prefs.getString("ssid", "");
  savedPASS = prefs.getString("pass", "");
  prefs.end();
}

void saveWiFiCredentials(String ssid, String pass) {
  prefs.begin("wifi", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
}

void handleCorsPreflight() {
  configServer.sendHeader("Access-Control-Allow-Origin", "*");
  configServer.sendHeader("Access-Control-Allow-Methods", "POST, GET");
  configServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  configServer.send(200);
}

void handleWifiConfigSubmit() {
  String body = configServer.arg("plain");

  int ssidStart = body.indexOf("\"ssid\":\"") + 8;
  int ssidEnd   = body.indexOf("\"", ssidStart);
  int passStart = body.indexOf("\"password\":\"") + 12;
  int passEnd   = body.indexOf("\"", passStart);

  String ssid = body.substring(ssidStart, ssidEnd);
  String pass = body.substring(passStart, passEnd);

  Serial.print("Received Wi-Fi from app - SSID: "); Serial.println(ssid);

  saveWiFiCredentials(ssid, pass);

  configServer.sendHeader("Access-Control-Allow-Origin", "*");
  configServer.send(200, "application/json", "{\"status\":\"saved, rebooting\"}");

  delay(1000);
  ESP.restart();
}

void startConfigMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("SmartWaterTank-Setup", "12345678");
  Serial.println("========================================");
  Serial.println("  WI-FI SETUP MODE");
  Serial.println("  1. On your phone, join Wi-Fi network:");
  Serial.println("     SmartWaterTank-Setup  (password: 12345678)");
  Serial.println("  2. Open the app's Device Setup screen");
  Serial.println("  3. Enter your home Wi-Fi name + password there");
  Serial.print("  Device config address: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("========================================");

  configServer.onNotFound(handleCorsPreflight);
  configServer.on("/wifi-config", HTTP_OPTIONS, handleCorsPreflight);
  configServer.on("/wifi-config", HTTP_POST, handleWifiConfigSubmit);
  configServer.begin();

  while (true) {
    configServer.handleClient();
    delay(10);
  }
}

void connectWiFi() {
  loadWiFiCredentials();

  if (savedSSID == "") {
    Serial.println("No saved Wi-Fi - entering setup mode");
    startConfigMode();  // blocks until configured + reboots, never returns
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(savedSSID.c_str(), savedPASS.c_str());
  Serial.print("Connecting to saved Wi-Fi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print("."); tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nSaved Wi-Fi failed - entering setup mode");
    startConfigMode();  // blocks until configured + reboots, never returns
  }
}

// ============================================================
//  UPLOAD TO SERVER / CLOUD
// ============================================================
void sendData(float lvl, float flow, float total,
              float ph, float ntu, bool pump) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("  [offline] upload skipped, edge logic continues");
    return;
  }
  WiFiClientSecure client;
  client.setInsecure();  // skip certificate validation (simplest option for Render)

  HTTPClient http;
  http.begin(client, SERVER_URL);   // use the secure client for https://
  http.addHeader("Content-Type", "application/json");
  
  String body = "{";
  body += "\"tank_level_pct\":" + String(lvl, 1) + ",";
  body += "\"flow_rate\":"      + String(flow, 2) + ",";
  body += "\"total_litres\":"   + String(total, 2) + ",";
  body += "\"ph_value\":"       + String(ph, 2) + ",";
  body += "\"turbidity\":"      + String(ntu, 1) + ",";
  body += "\"pump_status\":"    + String(pump ? "true" : "false");
  body += "}";

  int code = http.POST(body);
  Serial.print("  [upload] HTTP "); Serial.println(code);
  http.end();
}


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
#define RELAY_PIN    14
#define FLOW_OUT_PIN 26
#define TURB_PIN     34
#define PH_PIN       35

const char* SERVER_URL = "https:

float EMPTY_DISTANCE_CM = 44.0;
float FULL_DISTANCE_CM  = 5.0;
float TANK_CAPACITY_L   = 650.0;







const float TANK_HEIGHT_CM        = 55.0;
const float TANK_TOP_RADIUS_CM    = 20.0;
const float TANK_BOTTOM_RADIUS_CM = 13.5;



float radiusAtHeight(float heightCm) {
  float h = constrain(heightCm, 0, TANK_HEIGHT_CM);
  return TANK_BOTTOM_RADIUS_CM +
         (TANK_TOP_RADIUS_CM - TANK_BOTTOM_RADIUS_CM) * (h / TANK_HEIGHT_CM);
}




float volumeAtHeightL(float heightCm) {
  float h = constrain(heightCm, 0, TANK_HEIGHT_CM);
  if (h <= 0) return 0;
  float rTop = radiusAtHeight(h);
  float volumeCm3 = (PI * h / 3.0) *
    (sq(TANK_BOTTOM_RADIUS_CM) + TANK_BOTTOM_RADIUS_CM * rTop + sq(rTop));
  return volumeCm3 / 1000.0;
}








float MIN_DISTANCE_CM = 22.0;
float MAX_DISTANCE_CM = 400.0;


float PH_SLOPE  = -5.70;
float PH_OFFSET = 21.34;
const float TURB_CLEAR_RAW = 1150.0;
const float TURB_DIRTY_RAW = 1740.0;
const float TURB_MAX_NTU   = 100.0;

const float PH_MIN = 6.5, PH_MAX = 8.5;
const float TURBIDITY_MAX_NTU = 5.0;
const float PUMP_ON_LEVEL  = 20.0;




const float PUMP_OFF_LEVEL = 10.0;










const bool RELAY_ACTIVE_LOW = true;








const bool FORCE_PUMP_OFF = false;







const float MIN_LEVEL_FOR_QUALITY_PCT = 8.0;


volatile int pulseIn_count = 0, pulseOut_count = 0;
float flowRateIn = 0, flowRateOut = 0;
float totalLitresIn = 0, totalLitresOut = 0;
unsigned long lastFlowCalc = 0;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000;
unsigned long lastConfigSync = 0;
const unsigned long CONFIG_SYNC_INTERVAL = 60000;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
bool pumpStatus = false;







unsigned long pumpOnSince = 0;
const unsigned long DRY_RUN_TIMEOUT_MS = 120000;
const float MIN_LEVEL_RISE_PCT = 1.0;
float levelAtPumpStart = 0;
bool dryRunFault = false;
unsigned long dryRunFaultSince = 0;
const unsigned long DRY_RUN_RETRY_MS = 6000;







#define OFFLINE_BUFFER_SIZE 40
struct BufferedReading {
  unsigned long millisAtCapture;
  float levelPct;
  float flowOut;
  float totalOut;
  float ph;
  float turbidity;
  bool pump;
};
BufferedReading offlineBuffer[OFFLINE_BUFFER_SIZE];
int offlineBufferCount = 0;
bool wasOffline = false;
unsigned long offlineSince = 0;
unsigned long reconnectCount = 0;


bool remoteCommandPending = false;
bool remoteCommandValue = false;
unsigned long remoteCommandTimestamp = 0;
const unsigned long REMOTE_COMMAND_MAX_AGE_MS = 20000;












#define USAGE_HISTORY_DAYS 7
float dailyUsageHistory[USAGE_HISTORY_DAYS] = {0};
int usageHistoryCount = 0;
float litresAtDayStart = 0;
unsigned long lastDayRollover = 0;
const unsigned long DAY_MS = 86400000UL;





const float COLD_START_FALLBACK_L = 651.0;

Preferences prefs;
Preferences calPrefs;
WebServer configServer(80);
WebServer calServer(81);
String savedSSID, savedPASS;

void IRAM_ATTR pulseInISR()  { pulseIn_count++;  }
void IRAM_ATTR pulseOutISR() { pulseOut_count++; }


float readDistance(float tempC);







bool beginHttpAuto(HTTPClient &http, const String &url) {
  if (url.startsWith("https:
    static WiFiClientSecure secureClient;
    secureClient.setInsecure();
    return http.begin(secureClient, url);
  } else {
    return http.begin(url);
  }
}


























float movingAverageOfHistory() {
  if (usageHistoryCount == 0) return COLD_START_FALLBACK_L;
  float sum = 0;
  for (int i = 0; i < usageHistoryCount; i++) sum += dailyUsageHistory[i];
  return sum / usageHistoryCount;
}

float stdDevOfHistory(float mean) {
  if (usageHistoryCount < 2) return 0;
  float sumSq = 0;
  for (int i = 0; i < usageHistoryCount; i++) {
    float diff = dailyUsageHistory[i] - mean;
    sumSq += diff * diff;
  }
  return sqrt(sumSq / usageHistoryCount);
}




void updateDailyUsageHistory(float totalLitresOutNow) {
  if (lastDayRollover == 0) {
    lastDayRollover = millis();
    litresAtDayStart = totalLitresOutNow;
    return;
  }
  if (millis() - lastDayRollover < DAY_MS) return;

  float todayUsage = totalLitresOutNow - litresAtDayStart;

  if (usageHistoryCount < USAGE_HISTORY_DAYS) {
    dailyUsageHistory[usageHistoryCount] = todayUsage;
    usageHistoryCount++;
  } else {
    for (int i = 1; i < USAGE_HISTORY_DAYS; i++) dailyUsageHistory[i - 1] = dailyUsageHistory[i];
    dailyUsageHistory[USAGE_HISTORY_DAYS - 1] = todayUsage;
  }

  Serial.printf("  [PREDICT] Day rolled over - usage: %.1fL, history depth: %d/%d\n",
                todayUsage, usageHistoryCount, USAGE_HISTORY_DAYS);

  litresAtDayStart = totalLitresOutNow;
  lastDayRollover = millis();
}











void computeEdgePrediction(float levelPct, float todaySoFarLitres,
                            float &outPredictedLitres, float &outDaysRemaining,
                            bool &outAlert, bool &outLeakDetected, float &outLeakThreshold) {


  outPredictedLitres = movingAverageOfHistory();

  float availableLitres = (levelPct / 100.0) * TANK_CAPACITY_L;
  outDaysRemaining = (outPredictedLitres > 0)
                      ? (availableLitres / outPredictedLitres)
                      : 999;
  outAlert = outDaysRemaining < 2.0;


  if (usageHistoryCount >= 3) {
    float mean = movingAverageOfHistory();
    float std = stdDevOfHistory(mean);
    outLeakThreshold = mean + 2 * std;
    outLeakDetected = todaySoFarLitres > outLeakThreshold;
  } else {
    outLeakThreshold = 0;
    outLeakDetected = false;
  }
}




bool emptyManuallyCalibrated = false;






bool autoModeEnabled = true;

void loadCalibration() {
  calPrefs.begin("tankcal", false);
  EMPTY_DISTANCE_CM = calPrefs.getFloat("empty", 44.0);
  FULL_DISTANCE_CM  = calPrefs.getFloat("full", 5.0);
  TANK_CAPACITY_L   = calPrefs.getFloat("capacity", 650.0);
  emptyManuallyCalibrated = calPrefs.getBool("emptyCal", false);
  autoModeEnabled = calPrefs.getBool("autoMode", true);
  calPrefs.end();
  Serial.printf("Loaded calibration - Empty:%.1fcm Full:%.1fcm Capacity:%.0fL (manually calibrated: %s)\n",
                EMPTY_DISTANCE_CM, FULL_DISTANCE_CM, TANK_CAPACITY_L,
                emptyManuallyCalibrated ? "yes" : "no - using estimate");
}

void saveCalibration() {
  calPrefs.begin("tankcal", false);
  calPrefs.putFloat("empty", EMPTY_DISTANCE_CM);
  calPrefs.putFloat("full", FULL_DISTANCE_CM);
  calPrefs.putFloat("capacity", TANK_CAPACITY_L);
  calPrefs.putBool("emptyCal", emptyManuallyCalibrated);
  calPrefs.putBool("autoMode", autoModeEnabled);
  calPrefs.end();
}




void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("========================================");
  Serial.println("  SMART WATER TANK - FULL SYSTEM");
  Serial.println("========================================");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);




  if (RELAY_ACTIVE_LOW) {
    pinMode(RELAY_PIN, INPUT);
  } else {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
  }
  pinMode(FLOW_IN_PIN, INPUT_PULLUP);
  pinMode(FLOW_OUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_IN_PIN),  pulseInISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_OUT_PIN), pulseOutISR, RISING);

  analogReadResolution(12);
  analogSetPinAttenuation(TURB_PIN, ADC_11db);
  analogSetPinAttenuation(PH_PIN,   ADC_11db);

  tempSensor.begin();
  loadCalibration();
  connectWiFi();
  fetchDeviceConfig();
  setupCalibrationEndpoints();

  Serial.println("Sensors initialised. Starting readings...\n");
}




void loop() {
  calServer.handleClient();

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C) tempC = 25.0;

  float distance = readDistance(tempC);
  float levelPct = 0;
  float availableLitres = 0;
  if (distance >= 0 && distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM) {



    float waterHeightCm = EMPTY_DISTANCE_CM - distance;
    waterHeightCm = constrain(waterHeightCm, 0, TANK_HEIGHT_CM);
    availableLitres = volumeAtHeightL(waterHeightCm);
    levelPct = (TANK_CAPACITY_L > 0) ? (availableLitres / TANK_CAPACITY_L) * 100.0 : 0;
    levelPct = constrain(levelPct, 0, 100);
  } else if (distance >= 0) {


    Serial.printf("  [warning] distance %.1fcm outside sensor range [%.0f-%.0fcm] - ignoring this reading\n",
                  distance, MIN_DISTANCE_CM, MAX_DISTANCE_CM);
  }

  if (millis() - lastConfigSync >= CONFIG_SYNC_INTERVAL) {
    fetchDeviceConfig();
    lastConfigSync = millis();
  }

  updateFlow();

  float phValue   = readPH();
  float turbidity = readTurbidity();
  bool sensorsSubmerged = levelPct >= MIN_LEVEL_FOR_QUALITY_PCT;
  bool  qualitySafe = !sensorsSubmerged ||
                       (phValue >= PH_MIN && phValue <= PH_MAX &&
                        turbidity <= TURBIDITY_MAX_NTU);
  if (!sensorsSubmerged) {
    Serial.println("  [QUALITY] Level too low for reliable pH/turbidity reading - quality gate bypassed for initial fill.");
  }









  if (FORCE_PUMP_OFF) {
    pumpOff();
    dryRunFault = false;
  } else if (!qualitySafe) {
    pumpOff();
    dryRunFault = false;
  } else if (autoModeEnabled && levelPct > PUMP_OFF_LEVEL) {
    pumpOff();
    dryRunFault = false;
  } else if (autoModeEnabled && levelPct < PUMP_ON_LEVEL) {
    if (!dryRunFault) {
      pumpOn();
    } else if (millis() - dryRunFaultSince > DRY_RUN_RETRY_MS) {

      Serial.println("  [SAFETY] Retrying pump after dry-run cooldown...");
      dryRunFault = false;
      pumpOn();
    }
  }








  if (pumpStatus) {
    if (pumpOnSince == 0) {
      pumpOnSince = millis();
      levelAtPumpStart = levelPct;
    }
    if (millis() - pumpOnSince > DRY_RUN_TIMEOUT_MS) {
      float levelRise = levelPct - levelAtPumpStart;
      if (levelRise < MIN_LEVEL_RISE_PCT) {
        Serial.println("  [SAFETY] Pump running but tank level not rising - source may be dry (air, not water). Stopping pump.");
        pumpOff();
        dryRunFault = true;
        dryRunFaultSince = millis();
      } else {

        pumpOnSince = millis();
        levelAtPumpStart = levelPct;
      }
    }
  } else {
    pumpOnSince = 0;
  }


  if (dryRunFault && levelPct >= PUMP_ON_LEVEL + 5.0) {
    dryRunFault = false;
    Serial.println("  [SAFETY] Dry-run fault cleared - level recovered.");
  }






  if (pumpStatus) {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
  } else if (RELAY_ACTIVE_LOW) {
    pinMode(RELAY_PIN, INPUT);
  } else {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
  }


  updateDailyUsageHistory(totalLitresOut);
  float todaySoFar = totalLitresOut - litresAtDayStart;
  float predPredictedLitres, predDaysRemaining, predLeakThreshold;
  bool predAlert, predLeakDetected;
  computeEdgePrediction(levelPct, todaySoFar,
                         predPredictedLitres, predDaysRemaining,
                         predAlert, predLeakDetected, predLeakThreshold);

  Serial.println("---------------------------------------");
  Serial.printf("Temperature : %.1f C\n", tempC);
  Serial.printf("Distance    : %.1f cm\n", distance);
  Serial.printf("Tank Level  : %.1f %% (%.0f L)\n", levelPct, availableLitres);
  Serial.printf("Flow In     : %.2f L/min\n", flowRateIn);
  Serial.printf("Flow Out    : %.2f L/min\n", flowRateOut);
  Serial.printf("pH Value    : %.2f\n", phValue);
  Serial.printf("Turbidity   : %.1f NTU\n", turbidity);
  Serial.printf("Pump        : %s (%s)%s | Auto: %s\n", pumpStatus ? "ON" : "OFF",
                qualitySafe ? "SAFE" : "UNSAFE",
                dryRunFault ? " [DRY-RUN FAULT]" : "",
                autoModeEnabled ? "ON" : "OFF (manual)");
  Serial.printf("Predicted   : %.1f L/day (moving avg, %d-day history) | Days left: %.1f%s\n",
                predPredictedLitres, usageHistoryCount, predDaysRemaining,
                predAlert ? " [LOW - refill soon]" : "");
  if (usageHistoryCount >= 3) {
    Serial.printf("Leak check  : today %.1fL vs threshold %.1fL%s\n",
                  todaySoFar, predLeakThreshold,
                  predLeakDetected ? " [POSSIBLE LEAK]" : " [normal]");
  } else {
    Serial.printf("Leak check  : building history (%d/%d days)\n",
                  usageHistoryCount, USAGE_HISTORY_DAYS);
  }

  if (millis() - lastSend >= SEND_INTERVAL) {
    handleConnectivityAndSync(levelPct, flowRateOut, totalLitresOut, phValue, turbidity, pumpStatus);
    lastSend = millis();
  }

  delay(1000);
}




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
  Serial.print("  [debug] Turbidity raw ADC: "); Serial.println(raw);
  float ntu = (TURB_CLEAR_RAW - raw) / (TURB_CLEAR_RAW - TURB_DIRTY_RAW) * TURB_MAX_NTU;
  return constrain(ntu, 0, TURB_MAX_NTU);
}



















void pumpOn()  {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
  pumpStatus = true;
}
void pumpOff() {
  if (RELAY_ACTIVE_LOW) {
    pinMode(RELAY_PIN, INPUT);
  } else {
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
  }
  pumpStatus = false;
}










void fetchDeviceConfig() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("  [CONFIG] Offline - using existing/default sensor limits.");
    return;
  }

  HTTPClient http;
  String cfgUrl = String(SERVER_URL);
  cfgUrl.replace("/sensor-reading", "/device-config");
  beginHttpAuto(http, cfgUrl);
  int code = http.GET();
  if (code == 200) {
    String resp = http.getString();

    int capIdx = resp.indexOf("\"tank_capacity_liters\":");
    if (capIdx >= 0) {
      float val = resp.substring(capIdx + 23).toFloat();
      if (val > 0) { TANK_CAPACITY_L = val; saveCalibration(); }
    }





    if (!emptyManuallyCalibrated) {
      int estIdx = resp.indexOf("\"estimated_empty_distance_cm\":");
      if (estIdx >= 0) {
        float val = resp.substring(estIdx + 30).toFloat();
        if (val > 0) {
          EMPTY_DISTANCE_CM = val;
          saveCalibration();
          Serial.printf("  [CONFIG] Using spec-estimated empty distance: %.1fcm (not yet manually calibrated)\n", val);
        }
      }
    }

    int minIdx = resp.indexOf("\"min_distance_cm\":");
    if (minIdx >= 0) {
      float val = resp.substring(minIdx + 19).toFloat();
      if (val > 0) MIN_DISTANCE_CM = val;
    }

    int maxIdx = resp.indexOf("\"max_distance_cm\":");
    if (maxIdx >= 0) {
      float val = resp.substring(maxIdx + 19).toFloat();
      if (val > 0) MAX_DISTANCE_CM = val;
    }

    Serial.printf("  [CONFIG] Synced from cloud - Capacity:%.0fL, Sensor range:%.0f-%.0fcm\n",
                  TANK_CAPACITY_L, MIN_DISTANCE_CM, MAX_DISTANCE_CM);
  } else {
    Serial.printf("  [CONFIG] Fetch failed (HTTP %d) - using existing sensor limits.\n", code);
  }
  http.end();
}




void setupCalibrationEndpoints() {
  calServer.on("/calibrate-empty", HTTP_POST, []() {
    tempSensor.requestTemperatures();
    float tempC = tempSensor.getTempCByIndex(0);
    if (tempC == DEVICE_DISCONNECTED_C) tempC = 25.0;
    float d = readDistance(tempC);
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    if (d < 0) {
      calServer.send(500, "application/json", "{\"error\":\"sensor read failed - no echo received\"}");
      return;
    }
    if (d < MIN_DISTANCE_CM) {
      calServer.send(422, "application/json",
        "{\"error\":\"reading " + String(d, 1) + "cm is closer than the sensor's minimum range of " +
        String(MIN_DISTANCE_CM, 0) + "cm. Check the sensor is mounted correctly.\"}");
      return;
    }
    if (d > MAX_DISTANCE_CM) {
      calServer.send(422, "application/json",
        "{\"error\":\"reading " + String(d, 1) + "cm exceeds the sensor's maximum range of " +
        String(MAX_DISTANCE_CM, 0) + "cm. Check the tank is actually empty and in range.\"}");
      return;
    }
    EMPTY_DISTANCE_CM = d;
    emptyManuallyCalibrated = true;
    saveCalibration();
    calServer.send(200, "application/json", "{\"empty_distance_cm\":" + String(d, 1) + "}");
  });

  calServer.on("/calibrate-full", HTTP_POST, []() {
    tempSensor.requestTemperatures();
    float tempC = tempSensor.getTempCByIndex(0);
    if (tempC == DEVICE_DISCONNECTED_C) tempC = 25.0;
    float d = readDistance(tempC);
    calServer.sendHeader("Access-Control-Allow-Origin", "*");
    if (d < 0) {
      calServer.send(500, "application/json", "{\"error\":\"sensor read failed - no echo received\"}");
      return;
    }
    if (d < MIN_DISTANCE_CM) {





      calServer.send(422, "application/json",
        "{\"error\":\"reading " + String(d, 1) + "cm is closer than the sensor's minimum range of " +
        String(MIN_DISTANCE_CM, 0) + "cm. The sensor needs to be mounted further above the maximum water level, or this tank/mounting combination cannot be reliably measured.\"}");
      return;
    }
    if (d > MAX_DISTANCE_CM) {
      calServer.send(422, "application/json",
        "{\"error\":\"reading " + String(d, 1) + "cm exceeds the sensor's maximum range of " +
        String(MAX_DISTANCE_CM, 0) + "cm.\"}");
      return;
    }
    FULL_DISTANCE_CM = d;




    float fullWaterHeightCm = constrain(EMPTY_DISTANCE_CM - FULL_DISTANCE_CM, 0, TANK_HEIGHT_CM);
    TANK_CAPACITY_L = volumeAtHeightL(fullWaterHeightCm);
    saveCalibration();
    calServer.send(200, "application/json",
      "{\"full_distance_cm\":" + String(d, 1) +
      ",\"computed_capacity_l\":" + String(TANK_CAPACITY_L, 1) + "}");
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
  Serial.print("  Device config address: http:
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
    startConfigMode();
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
    startConfigMode();
  }
}














void handleConnectivityAndSync(float lvl, float flow, float total,
                                float ph, float ntu, bool pump) {
  bool isOnline = (WiFi.status() == WL_CONNECTED);

  if (!isOnline) {
    if (!wasOffline) {
      wasOffline = true;
      offlineSince = millis();
      Serial.println("  [RECONCILE] Connectivity lost - buffering readings locally. Edge logic continues unaffected.");
    }
    bufferReading(lvl, flow, total, ph, ntu, pump);
    return;
  }


  if (wasOffline) {


    unsigned long downtimeMs = millis() - offlineSince;
    reconnectCount++;
    Serial.printf("  [RECONCILE] Connectivity restored after %.1f min offline. Syncing %d buffered readings (event #%lu)...\n",
                  downtimeMs / 60000.0, offlineBufferCount, reconnectCount);
    syncOfflineBuffer(downtimeMs);
    wasOffline = false;
  }








  pollForRemoteCommand();
  applyRemoteCommandIfValid();
  pollForAutoMode();


  sendData(lvl, flow, total, ph, ntu, pump);
}

void bufferReading(float lvl, float flow, float total, float ph, float ntu, bool pump) {
  if (offlineBufferCount >= OFFLINE_BUFFER_SIZE) {


    for (int i = 1; i < OFFLINE_BUFFER_SIZE; i++) offlineBuffer[i - 1] = offlineBuffer[i];
    offlineBufferCount = OFFLINE_BUFFER_SIZE - 1;
  }
  offlineBuffer[offlineBufferCount] = { millis(), lvl, flow, total, ph, ntu, pump };
  offlineBufferCount++;
}

void syncOfflineBuffer(unsigned long downtimeMs) {
  if (offlineBufferCount == 0) {
    Serial.println("  [RECONCILE] No buffered readings to sync (outage was shorter than one cycle).");
    return;
  }

  HTTPClient http;


  String body = "{";
  body += "\"reconnect_event\":" + String(reconnectCount) + ",";
  body += "\"downtime_ms\":" + String(downtimeMs) + ",";
  body += "\"readings\":[";
  for (int i = 0; i < offlineBufferCount; i++) {
    if (i > 0) body += ",";
    body += "{";
    body += "\"tank_level_pct\":" + String(offlineBuffer[i].levelPct, 1) + ",";
    body += "\"flow_rate\":"      + String(offlineBuffer[i].flowOut, 2) + ",";
    body += "\"total_litres\":"   + String(offlineBuffer[i].totalOut, 2) + ",";
    body += "\"ph_value\":"       + String(offlineBuffer[i].ph, 2) + ",";
    body += "\"turbidity\":"      + String(offlineBuffer[i].turbidity, 1) + ",";
    body += "\"pump_status\":"    + String(offlineBuffer[i].pump ? "true" : "false");
    body += "}";
  }
  body += "]}";

  String syncUrl = String(SERVER_URL);
  syncUrl.replace("/sensor-reading", "/sync-batch");

  beginHttpAuto(http, syncUrl);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  Serial.print("  [RECONCILE] Batch sync HTTP "); Serial.println(code);
  http.end();

  offlineBufferCount = 0;
}











void pollForRemoteCommand() {
  HTTPClient http;
  String cmdUrl = String(SERVER_URL);
  cmdUrl.replace("/sensor-reading", "/pump-command");
  beginHttpAuto(http, cmdUrl);
  int code = http.GET();
  if (code == 200) {
    String resp = http.getString();

    if (resp.indexOf("\"has_command\":true") >= 0) {
      remoteCommandValue = (resp.indexOf("\"pump_on\":true") >= 0);
      remoteCommandPending = true;
      remoteCommandTimestamp = millis();
      Serial.print("  [RECONCILE] Remote pump command received: ");
      Serial.println(remoteCommandValue ? "ON" : "OFF");
    }
  }
  http.end();
}





void pollForAutoMode() {
  HTTPClient http;
  String url = String(SERVER_URL);
  url.replace("/sensor-reading", "/auto-mode");
  beginHttpAuto(http, url);
  int code = http.GET();
  if (code == 200) {
    String resp = http.getString();
    bool cloudAutoMode = (resp.indexOf("\"auto_mode\":true") >= 0);
    if (cloudAutoMode != autoModeEnabled) {
      autoModeEnabled = cloudAutoMode;
      saveCalibration();
      Serial.print("  [CONFIG] Auto mode updated from app: ");
      Serial.println(autoModeEnabled ? "ON (automatic fill active)" : "OFF (manual control only)");
    }
  }
  http.end();
}

void applyRemoteCommandIfValid() {
  if (!remoteCommandPending) return;
  if (FORCE_PUMP_OFF) {
    Serial.println("  [OVERRIDE] FORCE_PUMP_OFF is active - remote command ignored, pump stays off.");
    pumpOff();
    remoteCommandPending = false;
    return;
  }

  bool tooStale = (millis() - remoteCommandTimestamp) > REMOTE_COMMAND_MAX_AGE_MS;
  bool safetyBlocked = dryRunFault;

  if (tooStale) {
    Serial.println("  [RECONCILE] Remote command rejected - stale (>5s), local state unchanged.");
  } else if (safetyBlocked) {
    Serial.println("  [RECONCILE] Remote command rejected - local safety fault active, physical state wins.");
  } else {
    Serial.print("  [RECONCILE] Remote command accepted: pump ");
    Serial.println(remoteCommandValue ? "ON" : "OFF");
    if (remoteCommandValue) pumpOn(); else pumpOff();
  }

  remoteCommandPending = false;
}




void sendData(float lvl, float flow, float total,
              float ph, float ntu, bool pump) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("  [offline] upload skipped, edge logic continues");
    return;
  }
  HTTPClient http;
  beginHttpAuto(http, SERVER_URL);
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
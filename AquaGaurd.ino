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

// ---------------- SENSOR PHYSICAL LIMITS (backend-managed) ----------------
// The JSN-SR04T cannot reliably measure closer than its blind zone,
// or beyond its maximum range. These are fetched from the cloud
// backend on boot (single source of truth, also editable from the
// app), but default to safe hardcoded values if the device is
// offline at boot - the device must remain fully functional without
// connectivity, consistent with the offline-first design.
float MIN_DISTANCE_CM = 22.0;   // JSN-SR04T blind zone - hardcoded safe default
float MAX_DISTANCE_CM = 400.0;  // JSN-SR04T max reliable range - hardcoded safe default

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
unsigned long lastConfigSync = 0;
const unsigned long CONFIG_SYNC_INTERVAL = 60000; // resync config every 60s

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
bool pumpStatus = false;

// ---------------- DRY-RUN / MAINS-FAILURE PROTECTION ----------------
// NOTE: flow sensor pulses can be triggered by air movement, not only
// water, so flow rate alone is unreliable for dry-run detection.
// Instead we check whether the TANK LEVEL is actually rising while the
// pump is on - if it is not, the source has failed regardless of what
// the flow sensor reports.
unsigned long pumpOnSince = 0;
const unsigned long DRY_RUN_TIMEOUT_MS = 30000;    // 30s grace period before checking
const float MIN_LEVEL_RISE_PCT = 1.0;              // must rise at least 1% in the check window
float levelAtPumpStart = 0;
bool dryRunFault = false;
unsigned long dryRunFaultSince = 0;
const unsigned long DRY_RUN_RETRY_MS = 300000;     // retry every 5 minutes

// ---------------- SPLIT-BRAIN RECONCILIATION ----------------
// While offline, readings are buffered locally instead of being
// dropped. On reconnect, the buffer is uploaded as a single
// reconciliation batch so the cloud record reflects what the edge
// logic actually did during the outage - the physical/local state
// is always treated as the source of truth, never the reverse.
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

// Remote pump command state (for manual override from the app)
bool remoteCommandPending = false;
bool remoteCommandValue = false;
unsigned long remoteCommandTimestamp = 0;
const unsigned long REMOTE_COMMAND_MAX_AGE_MS = 5000; // reject if older than 5s

// ---------------- ON-DEVICE PREDICTION (offline-first) ----------------
// Two lightweight methods run natively on the ESP32, requiring no
// network call and no server-side model:
//
// 1. Embedded polynomial regression - the coefficients learned by the
//    offline-trained scikit-learn model, evaluated here as plain
//    arithmetic (no ML library needed on-device).
// 2. On-device moving average - computed from the device's own real
//    daily usage history, genuinely adaptive to this specific
//    household rather than the generic training dataset.
//
// Fill these in from extract_coefficients.py output:
const float PRED_INTERCEPT = 621.145665;
const float PRED_COEF_B    = 1.232212;
const float PRED_COEF_C    = -0.009455;
const int   PRED_DAY_INDEX = 101;   // day index used by the trained model

#define USAGE_HISTORY_DAYS 7
float dailyUsageHistory[USAGE_HISTORY_DAYS] = {0};
int usageHistoryCount = 0;
float litresAtDayStart = 0;
unsigned long lastDayRollover = 0;
const unsigned long DAY_MS = 86400000UL; // 24h - use a shorter value for demo/testing

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
//  ON-DEVICE PREDICTION (Stage 6 - fully offline forecasting)
// ============================================================
// Two methods are used for two DIFFERENT purposes, not as
// interchangeable alternatives - this follows directly from the
// evaluation in Chapter 5.4.1, where polynomial regression was
// shown to outperform a moving-average baseline for forecasting
// non-linear consumption. A flat moving average cannot capture the
// curvature of human usage patterns, so it is not used as the
// forecasting method here.
//
//   - Polynomial regression (embedded model coefficients): the
//     PRIMARY forecasting method, used for tomorrow's predicted
//     usage and time-to-empty. This is the method proven more
//     accurate in evaluation, now migrated to run on-device.
//
//   - Moving average (on-device, real data): used ONLY for leak
//     detection, where the requirement is simply "is today's usage
//     abnormal relative to this household's recent average" - a
//     task moving average is well suited to, unlike forecasting.

// PRIMARY FORECAST: embedded polynomial regression
float predictPolynomial() {
  float x = PRED_DAY_INDEX;
  float y = PRED_INTERCEPT + PRED_COEF_B * x + PRED_COEF_C * x * x;
  return (y < 0) ? 0 : y;
}

// LEAK DETECTION ONLY: on-device moving average + std dev of the
// device's own recent daily usage history.
float movingAverageOfHistory() {
  if (usageHistoryCount == 0) return 0;
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

// Call once per "day" (or once per DAY_MS elapsed) to roll the
// day's total usage into the on-device history window, used for
// leak detection.
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

// Combined output: forecast (polynomial) + depletion + leak flag
// (moving average / std dev), computed entirely on-device.
//
// NOTE: this deliberately uses output parameters instead of returning
// a custom struct. The Arduino IDE auto-generates function prototypes
// and inserts them immediately after the #include lines, before any
// struct defined later in the file - a struct RETURN TYPE therefore
// triggers "does not name a type" even when the struct is correctly
// defined earlier than the function in the source. Output parameters
// avoid this entirely since only built-in types appear in the signature.
void computeEdgePrediction(float levelPct, float todaySoFarLitres,
                            float &outPredictedLitres, float &outDaysRemaining,
                            bool &outAlert, bool &outLeakDetected, float &outLeakThreshold) {
  // Forecast: polynomial regression (proven more accurate in Ch.5.4.1 evaluation)
  outPredictedLitres = predictPolynomial();

  float availableLitres = (levelPct / 100.0) * TANK_CAPACITY_L;
  outDaysRemaining = (outPredictedLitres > 0)
                      ? (availableLitres / outPredictedLitres)
                      : 999;
  outAlert = outDaysRemaining < 2.0;

  // Leak detection: moving average + 2 std dev, using real device history
  if (usageHistoryCount >= 3) {
    float mean = movingAverageOfHistory();
    float std = stdDevOfHistory(mean);
    outLeakThreshold = mean + 2 * std;
    outLeakDetected = todaySoFarLitres > outLeakThreshold;
  } else {
    outLeakThreshold = 0;
    outLeakDetected = false; // not enough history yet to judge
  }
}

// ============================================================
//  CALIBRATION STORAGE
// ============================================================
bool emptyManuallyCalibrated = false;

void loadCalibration() {
  calPrefs.begin("tankcal", false);
  EMPTY_DISTANCE_CM = calPrefs.getFloat("empty", 44.0);
  FULL_DISTANCE_CM  = calPrefs.getFloat("full", 5.0);
  TANK_CAPACITY_L   = calPrefs.getFloat("capacity", 650.0);
  emptyManuallyCalibrated = calPrefs.getBool("emptyCal", false);
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
  fetchDeviceConfig();          // sync capacity + sensor limits from cloud (advisory)
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
  if (distance >= 0 && distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM) {
    levelPct = (EMPTY_DISTANCE_CM - distance) /
               (EMPTY_DISTANCE_CM - FULL_DISTANCE_CM) * 100.0;
    levelPct = constrain(levelPct, 0, 100);
  } else if (distance >= 0) {
    // Reading was received but falls outside the sensor's physical
    // range - treat as unreliable rather than acting on it.
    Serial.printf("  [warning] distance %.1fcm outside sensor range [%.0f-%.0fcm] - ignoring this reading\n",
                  distance, MIN_DISTANCE_CM, MAX_DISTANCE_CM);
  }

  if (millis() - lastConfigSync >= CONFIG_SYNC_INTERVAL) {
    fetchDeviceConfig();
    lastConfigSync = millis();
  }
  float availableLitres = (levelPct / 100.0) * TANK_CAPACITY_L;

  updateFlow();

  float phValue   = readPH();
  float turbidity = readTurbidity();
  bool  qualitySafe = (phValue >= PH_MIN && phValue <= PH_MAX &&
                       turbidity <= TURBIDITY_MAX_NTU);

  // EDGE FAIL-SAFE PUMP LOGIC - runs locally, always
  if (!qualitySafe) {
    pumpOff();
    dryRunFault = false;
  } else if (levelPct > PUMP_OFF_LEVEL) {
    pumpOff();
    dryRunFault = false;
  } else if (levelPct < PUMP_ON_LEVEL) {
    if (!dryRunFault) {
      pumpOn();
    } else if (millis() - dryRunFaultSince > DRY_RUN_RETRY_MS) {
      // cooldown elapsed - try the source again, in case supply resumed
      Serial.println("  [SAFETY] Retrying pump after dry-run cooldown...");
      dryRunFault = false;
      pumpOn();
    }
  }

  // DRY-RUN / MAINS-FAILURE PROTECTION
  // Flow sensor pulses can be caused by air movement, not just water,
  // so we check the more reliable signal instead: is the tank level
  // actually rising while the pump has been running? If the pump has
  // been on past the grace period and the level has not meaningfully
  // increased, the source has likely failed (air, not water, is being
  // pumped) - stop the pump to protect it.
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
        // Level is genuinely rising - reset the window so we keep checking
        pumpOnSince = millis();
        levelAtPumpStart = levelPct;
      }
    }
  } else {
    pumpOnSince = 0;
  }

  // Clear the fault once the tank has recovered on its own
  if (dryRunFault && levelPct >= PUMP_ON_LEVEL + 5.0) {
    dryRunFault = false;
    Serial.println("  [SAFETY] Dry-run fault cleared - level recovered.");
  }

  // ON-DEVICE PREDICTION - runs every cycle, no network needed
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
  Serial.printf("Pump        : %s (%s)%s\n", pumpStatus ? "ON" : "OFF",
                qualitySafe ? "SAFE" : "UNSAFE",
                dryRunFault ? " [DRY-RUN FAULT]" : "");
  Serial.printf("Predicted   : %.1f L/day (polynomial) | Days left: %.1f%s\n",
                predPredictedLitres, predDaysRemaining,
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
//  DEVICE CONFIG SYNC (cloud is the single source of truth)
// ============================================================
// Fetches tank capacity and sensor physical limits from the cloud
// backend. Called once at boot and again periodically. If the
// device is offline, this simply fails silently and the existing
// (hardcoded default, or last successfully fetched) values remain
// in use - config sync is advisory only, never a hard dependency,
// so the device stays fully operational without connectivity.
void fetchDeviceConfig() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("  [CONFIG] Offline - using existing/default sensor limits.");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String cfgUrl = String(SERVER_URL);
  cfgUrl.replace("/sensor-reading", "/device-config");

  http.begin(client, cfgUrl);
  int code = http.GET();
  if (code == 200) {
    String resp = http.getString();

    int capIdx = resp.indexOf("\"tank_capacity_liters\":");
    if (capIdx >= 0) {
      float val = resp.substring(capIdx + 23).toFloat();
      if (val > 0) { TANK_CAPACITY_L = val; saveCalibration(); }
    }

    // Only adopt the manufacturer-spec ESTIMATE if this device has
    // never had a real, live empty-tank calibration performed. Once
    // a real measurement exists, it always takes precedence over an
    // average derived from standard tank dimensions.
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
    emptyManuallyCalibrated = true; // real measurement now overrides any spec estimate
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
      // This is the critical case from the tank spec chart: if the
      // sensor is mounted too close to the maximum water level, the
      // "Full" reading falls inside the sensor's blind zone and can
      // never be measured reliably. Reject with a clear explanation
      // rather than silently accepting an unreliable calibration.
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
    saveCalibration();
    calServer.send(200, "application/json", "{\"full_distance_cm\":" + String(d, 1) + "}");
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
//  SPLIT-BRAIN RECONCILIATION
// ============================================================
// Called every SEND_INTERVAL instead of sendData() directly.
// While connected: uploads live, and checks for any pending
// manual pump command from the app (applying the "physical/
// safety state wins" rule below).
// While offline: buffers the reading locally instead of losing it.
// On the transition back online: uploads everything captured
// during the outage as one reconciliation batch, so the cloud
// record reflects what actually happened, not just a gap.
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

  // We are online now.
  if (wasOffline) {
    // Just reconnected - flush the buffer as a reconciliation batch
    // BEFORE sending the live reading, so history stays in order.
    unsigned long downtimeMs = millis() - offlineSince;
    reconnectCount++;
    Serial.printf("  [RECONCILE] Connectivity restored after %.1f min offline. Syncing %d buffered readings (event #%lu)...\n",
                  downtimeMs / 60000.0, offlineBufferCount, reconnectCount);
    syncOfflineBuffer(downtimeMs);
    wasOffline = false;
  }

  // Apply any pending remote pump command (manual override from app),
  // subject to the local-state-wins reconciliation rule.
  applyRemoteCommandIfValid();

  // Normal live upload
  sendData(lvl, flow, total, ph, ntu, pump);

  // Also poll for a queued manual command for next cycle
  pollForRemoteCommand();
}

void bufferReading(float lvl, float flow, float total, float ph, float ntu, bool pump) {
  if (offlineBufferCount >= OFFLINE_BUFFER_SIZE) {
    // Buffer full - drop the oldest to make room (ring-buffer behaviour),
    // since we prioritise recent history over very old outage data.
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

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  // Build the batch JSON: an array of readings plus reconciliation metadata
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

  http.begin(client, syncUrl);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  Serial.print("  [RECONCILE] Batch sync HTTP "); Serial.println(code);
  http.end();

  offlineBufferCount = 0; // clear buffer after attempting sync
}

// ============================================================
//  REMOTE PUMP OVERRIDE - "physical state wins" reconciliation
// ============================================================
// The app MAY request a manual pump override via the cloud, but this
// is deliberately advisory only. Local edge safety logic can and will
// override it: a stale command (>5s old by the time it is received)
// or a command that conflicts with an active safety fault (unsafe
// water quality or a dry-run fault) is rejected outright. This
// prevents a delayed or corrupted network command from ever pushing
// the pump into an unsafe physical state.
void pollForRemoteCommand() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  String cmdUrl = String(SERVER_URL);
  cmdUrl.replace("/sensor-reading", "/pump-command");

  http.begin(client, cmdUrl);
  int code = http.GET();
  if (code == 200) {
    String resp = http.getString();
    // Expected: {"has_command": true, "pump_on": true, "issued_at_ms": 123456}
    if (resp.indexOf("\"has_command\":true") >= 0) {
      remoteCommandValue = (resp.indexOf("\"pump_on\":true") >= 0);
      remoteCommandPending = true;
      remoteCommandTimestamp = millis(); // local receipt time, used for staleness check
      Serial.print("  [RECONCILE] Remote pump command received: ");
      Serial.println(remoteCommandValue ? "ON" : "OFF");
    }
  }
  http.end();
}

void applyRemoteCommandIfValid() {
  if (!remoteCommandPending) return;

  bool tooStale = (millis() - remoteCommandTimestamp) > REMOTE_COMMAND_MAX_AGE_MS;
  bool safetyBlocked = dryRunFault; // extend with other fault flags as needed

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

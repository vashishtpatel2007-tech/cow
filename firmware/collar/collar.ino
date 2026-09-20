/*

  Hardware:
    ESP32-S3
    GPS
    MPU6050
    SX1262
    Buzzer
    Vibration motor

  Architecture:

    GPS + IMU
        |
        v
    LOCAL PREDICTION ENGINE
        |
        +----> Buzzer / Vibration
        |
        +----> LoRa telemetry
                 |
                 v
              Gateway

  Gateway commands:

    CMD|PING
    CMD|REQ_TELEMETRY
    CMD|SET_BOUNDARY|count|lat|lon|lat|lon...
    CMD|SET_ROAD|index|id|risk|lat1|lon1|lat2|lon2
    CMD|CLEAR_BOUNDARY

  Collar responses:

    ACK|PONG
    ACK|BOUNDARY_OK
    ACK|ROAD_OK
    ACK|BOUNDARY_CLEARED
    TEL|...

  LoRa:
    868 MHz
    SF10
    BW 125 kHz
    CR 4/5
    Sync 0x34
*/

#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// GPS
// ============================================================

#define GPS_RX_PIN 17
#define GPS_TX_PIN 18

// ============================================================
// MPU6050
// ============================================================

#define I2C_SDA 8
#define I2C_SCL 9
#define MPU_ADDR 0x68

// ============================================================
// SX1262
// ============================================================

#define LORA_SCK 12
#define LORA_MISO 13
#define LORA_MOSI 11
#define LORA_NSS 10

SX1262 radio = new Module(LORA_NSS,
                          16, // DIO1
                          14, // RESET
                          15  // BUSY
);

// ============================================================
// ACTUATOR PINS
// CHANGE ONLY IF YOUR ACTUAL WIRING IS DIFFERENT
// ============================================================

#define BUZZER_PIN 5
#define VIBRATION_PIN 6

// ============================================================
// OBJECTS
// ============================================================

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Preferences prefs;

// ============================================================
// STATUS
// ============================================================

bool loraOK = false;
bool imuOK = false;

float accelScale = 4096.0;

uint32_t telemetrySequence = 0;

// ============================================================
// IMPROVED IMU CALIBRATION / STATE
// ============================================================

// Number of stationary samples used during startup calibration.
#define IMU_CALIBRATION_SAMPLES 300

// Small deadband to prevent tiny sensor noise being shown
// as movement.
#define IMU_DEADBAND_G 0.05f

// Smoothed motion value.
float smoothedDynamicAccel = 0.0f;

// Smoothed motion-energy value.
float smoothedMotionEnergy = 0.0f;

// Stationary baseline vector.
float baselineX = 0.0f;
float baselineY = 0.0f;
float baselineZ = 0.0f;

// Baseline acceleration magnitude.
float baselineMagnitude = 1.0f;

// Previous calibrated sample.
float previousX = 0.0f;
float previousY = 0.0f;
float previousZ = 0.0f;

// Current IMU measurements.
float currentAccelX = 0.0f;
float currentAccelY = 0.0f;
float currentAccelZ = 0.0f;

float currentAccelMagnitude = 0.0f;
float currentDynamicAcceleration = 0.0f;
float currentMotionEnergy = 0.0f;

// ============================================================
// BOUNDARY
// ============================================================

#define MAX_BOUNDARY_POINTS 8

struct GeoPoint {
  double lat;
  double lon;
};

GeoPoint boundary[MAX_BOUNDARY_POINTS];

int boundaryCount = 0;
bool boundaryEnabled = false;

// ============================================================
// ROADS
// ============================================================

#define MAX_ROADS 4

struct Road {
  bool enabled;
  int id;
  int riskLevel; // 1 LOW, 2 MEDIUM, 3 HIGH
  double lat1;
  double lon1;
  double lat2;
  double lon2;
};

Road roads[MAX_ROADS];

// ============================================================
// PREDICTION STATE
// ============================================================

double boundaryDistance = -1.0;
double roadDistance = -1.0;

int nearestRoadRisk = 0;

int riskScore = 0;

// Consecutive prediction cycles at/above the alarm threshold -- see the
// buzzer-arming debounce in runPrediction().
int highRiskStreak = 0;

String riskType = "SAFE";
String state = "WAITING";
String action = "NONE";

bool buzzerOn = false;
bool vibrationOn = false;

// A manual "make it beep" (CMD|BEEP|<seconds>) overrides the actuators for a
// fixed window rather than blocking with delay() -- GPS/IMU/LoRa must keep
// running while it sounds. 0 duration means no manual beep is active.
unsigned long manualBeepStartMs = 0;
unsigned long manualBeepDurationMs = 0;

String eventName = "NONE";

double previousBoundaryDistance = -1.0;
double previousRoadDistance = -1.0;

int previousRisk = 0;

// ============================================================
// MPU HELPERS
// ============================================================

void mpuWrite(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// ------------------------------------------------------------

uint8_t mpuRead(uint8_t reg) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 1, true);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0;
}

// ============================================================
// READ RAW ACCELERATION
// ============================================================

bool readRawAcceleration(float &ax, float &ay, float &az) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  if (Wire.available() < 6) {
    return false;
  }

  int16_t x = ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t y = ((int16_t)Wire.read() << 8) | Wire.read();

  int16_t z = ((int16_t)Wire.read() << 8) | Wire.read();

  ax = x / accelScale;
  ay = y / accelScale;
  az = z / accelScale;

  return true;
}

// ============================================================
// CALIBRATE MPU6050
// ============================================================
//
// KEEP THE COLLAR COMPLETELY STILL DURING THIS.
//
// We calculate a stationary reference vector and stationary
// magnitude. These are then used to separate normal baseline
// acceleration from actual movement.
//

void calibrateIMU() {

  Serial.println();
  Serial.println("========================================");
  Serial.println("         IMU CALIBRATION");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Keep the collar COMPLETELY STILL.");
  Serial.println("Calibration starting in 2 seconds...");

  delay(2000);

  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;
  double sumMagnitude = 0.0;

  int validSamples = 0;

  for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {

    float ax;
    float ay;
    float az;

    if (readRawAcceleration(ax, ay, az)) {

      float magnitude = sqrt(ax * ax + ay * ay + az * az);

      sumX += ax;
      sumY += ay;
      sumZ += az;
      sumMagnitude += magnitude;

      validSamples++;
    }

    delay(5);
  }

  if (validSamples < 100) {

    Serial.println();
    Serial.println("IMU calibration FAILED.");

    Serial.println("Not enough valid samples.");

    imuOK = false;

    return;
  }

  baselineX = sumX / validSamples;

  baselineY = sumY / validSamples;

  baselineZ = sumZ / validSamples;

  baselineMagnitude = sumMagnitude / validSamples;

  previousX = baselineX;

  previousY = baselineY;

  previousZ = baselineZ;

  smoothedDynamicAccel = 0.0f;
  smoothedMotionEnergy = 0.0f;

  Serial.println();
  Serial.println("IMU calibration COMPLETE ✓");

  Serial.printf("Baseline X : %.4f g\n", baselineX);

  Serial.printf("Baseline Y : %.4f g\n", baselineY);

  Serial.printf("Baseline Z : %.4f g\n", baselineZ);

  Serial.printf("Baseline |A| : %.4f g\n", baselineMagnitude);

  Serial.printf("Samples    : %d\n", validSamples);

  Serial.println("========================================");

  imuOK = true;
}

// ============================================================
// READ IMPROVED IMU DATA
// ============================================================
//
// dynamicAcceleration:
//   Difference between current acceleration magnitude and the
//   calibrated stationary magnitude.
//
// motionEnergy:
//   Magnitude of change across X/Y/Z from the previous sample.
//
// This makes vigorous shaking significantly more visible than
// the old single sqrt(ax² + ay² + az²) value.
//

void updateIMU() {

  if (!imuOK) {
    return;
  }

  float ax;
  float ay;
  float az;

  if (!readRawAcceleration(ax, ay, az)) {

    return;
  }

  currentAccelX = ax;
  currentAccelY = ay;
  currentAccelZ = az;

  // ----------------------------------------------------------
  // TOTAL ACCELERATION MAGNITUDE
  // ----------------------------------------------------------

  currentAccelMagnitude = sqrt(ax * ax + ay * ay + az * az);

  // ----------------------------------------------------------
  // DYNAMIC ACCELERATION
  // ----------------------------------------------------------

  float rawDynamic = fabs(currentAccelMagnitude - baselineMagnitude);

  if (rawDynamic < IMU_DEADBAND_G) {

    rawDynamic = 0.0f;
  }

  // ----------------------------------------------------------
  // AXIS-TO-AXIS MOTION ENERGY
  // ----------------------------------------------------------

  float dx = ax - previousX;

  float dy = ay - previousY;

  float dz = az - previousZ;

  float rawMotionEnergy = sqrt(dx * dx + dy * dy + dz * dz);

  if (rawMotionEnergy < IMU_DEADBAND_G) {

    rawMotionEnergy = 0.0f;
  }

  // ----------------------------------------------------------
  // EXPONENTIAL SMOOTHING
  // ----------------------------------------------------------
  //
  // Lower = smoother.
  // Higher = more responsive.
  //

  const float ALPHA = 0.35f;

  smoothedDynamicAccel =
      (ALPHA * rawDynamic) + ((1.0f - ALPHA) * smoothedDynamicAccel);

  smoothedMotionEnergy =
      (ALPHA * rawMotionEnergy) + ((1.0f - ALPHA) * smoothedMotionEnergy);

  currentDynamicAcceleration = smoothedDynamicAccel;

  currentMotionEnergy = smoothedMotionEnergy;

  // ----------------------------------------------------------
  // STORE CURRENT SAMPLE
  // ----------------------------------------------------------

  previousX = ax;
  previousY = ay;
  previousZ = az;
}

// ============================================================
// GPS
// ============================================================

void updateGPS() {

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Once a second: enough to tell wiring/baud dead (charsProcessed stays 0
  // forever) apart from alive-but-still-searching (chars flow, sentences
  // parse, but no fix yet) apart from wrong baud (chars flow, checksums
  // fail). Rate-limited and one line, unlike the earlier per-byte echo that
  // fought the LoRa RX window for CPU time.
  static unsigned long lastStatusAt = 0;

  if (millis() - lastStatusAt >= 1000) {
    lastStatusAt = millis();

    Serial.printf("GPS status: chars=%lu checksumOK=%lu fixSentences=%lu "
                  "checksumFail=%lu sats=%d fixValid=%s\n",
                  gps.charsProcessed(), gps.passedChecksum(),
                  gps.sentencesWithFix(), gps.failedChecksum(),
                  gps.satellites.isValid() ? gps.satellites.value() : -1,
                  gps.location.isValid() ? "YES" : "no");
  }
}

// ============================================================
// GEO DISTANCE
// ============================================================

double distanceMeters(double lat1, double lon1, double lat2, double lon2) {

  const double R = 6371000.0;

  double p1 = radians(lat1);

  double p2 = radians(lat2);

  double dp = radians(lat2 - lat1);

  double dl = radians(lon2 - lon1);

  double a = sin(dp / 2.0) * sin(dp / 2.0) +
             cos(p1) * cos(p2) * sin(dl / 2.0) * sin(dl / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

  return R * c;
}

// ============================================================
// GPS -> LOCAL XY
// ============================================================

void gpsToXY(double lat, double lon, double refLat, double refLon, double &x,
             double &y) {

  const double R = 6371000.0;

  x = radians(lon - refLon) * cos(radians(refLat)) * R;

  y = radians(lat - refLat) * R;
}

// ============================================================
// POINT -> LINE SEGMENT
// ============================================================

double pointToSegmentDistance(double pLat, double pLon, double aLat,
                              double aLon, double bLat, double bLon) {

  double px;
  double py;

  double ax;
  double ay;

  double bx;
  double by;

  gpsToXY(pLat, pLon, pLat, pLon, px, py);

  gpsToXY(aLat, aLon, pLat, pLon, ax, ay);

  gpsToXY(bLat, bLon, pLat, pLon, bx, by);

  double dx = bx - ax;

  double dy = by - ay;

  double len2 = dx * dx + dy * dy;

  if (len2 < 0.000001) {

    return sqrt(ax * ax + ay * ay);
  }

  double t = ((px - ax) * dx + (py - ay) * dy) / len2;

  if (t < 0.0) {
    t = 0.0;
  }

  if (t > 1.0) {
    t = 1.0;
  }

  double cx = ax + t * dx;

  double cy = ay + t * dy;

  double ex = px - cx;

  double ey = py - cy;

  return sqrt(ex * ex + ey * ey);
}

// ============================================================
// POINT IN POLYGON
// ============================================================

bool isInsideBoundary(double lat, double lon) {

  if (!boundaryEnabled || boundaryCount < 3) {

    return false;
  }

  bool inside = false;

  for (int i = 0, j = boundaryCount - 1; i < boundaryCount; j = i++) {

    double xi = boundary[i].lon;

    double yi = boundary[i].lat;

    double xj = boundary[j].lon;

    double yj = boundary[j].lat;

    bool intersect = ((yi > lat) != (yj > lat)) &&
                     (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi);

    if (intersect) {

      inside = !inside;
    }
  }

  return inside;
}

// ============================================================
// DISTANCE TO BOUNDARY
// ============================================================

double getBoundaryDistance(double lat, double lon) {

  if (!boundaryEnabled || boundaryCount < 3) {

    return -1.0;
  }

  double minimum = 1e12;

  for (int i = 0; i < boundaryCount; i++) {

    int j = (i + 1) % boundaryCount;

    double d =
        pointToSegmentDistance(lat, lon, boundary[i].lat, boundary[i].lon,
                               boundary[j].lat, boundary[j].lon);

    if (d < minimum) {

      minimum = d;
    }
  }

  return minimum;
}

// ============================================================
// DISTANCE TO NEAREST ROAD
// ============================================================

double getNearestRoadDistance(double lat, double lon, int &risk) {

  risk = 0;

  double minimum = 1e12;

  for (int i = 0; i < MAX_ROADS; i++) {

    if (!roads[i].enabled) {

      continue;
    }

    double d = pointToSegmentDistance(lat, lon, roads[i].lat1, roads[i].lon1,
                                      roads[i].lat2, roads[i].lon2);

    if (d < minimum) {

      minimum = d;

      risk = roads[i].riskLevel;
    }
  }

  if (minimum == 1e12) {

    return -1.0;
  }

  return minimum;
}

// ============================================================
// ACTUATORS
// ============================================================

void setWarningActuators(bool enabled) {

  // A manual "make it beep" in progress wins over the risk engine wanting
  // quiet -- otherwise a low-risk prediction cycle silences the beep the
  // farmer just asked for, half a second after it starts.
  if (!enabled && manualBeepDurationMs != 0 &&
      millis() - manualBeepStartMs < manualBeepDurationMs) {
    return;
  }

  buzzerOn = enabled;

  vibrationOn = enabled;

  digitalWrite(BUZZER_PIN, enabled ? HIGH : LOW);

  digitalWrite(VIBRATION_PIN, enabled ? HIGH : LOW);
}

// ============================================================
// LOAD BOUNDARY
// ============================================================

void loadBoundary() {

  prefs.begin("boundary", true);

  boundaryCount = prefs.getInt("count", 0);

  if (boundaryCount < 0 || boundaryCount > MAX_BOUNDARY_POINTS) {

    boundaryCount = 0;
  }

  boundaryEnabled = prefs.getBool("enabled", false);

  for (int i = 0; i < boundaryCount; i++) {

    char keyLat[12];
    char keyLon[12];

    snprintf(keyLat, sizeof(keyLat), "lat%d", i);

    snprintf(keyLon, sizeof(keyLon), "lon%d", i);

    boundary[i].lat = prefs.getDouble(keyLat, 0.0);

    boundary[i].lon = prefs.getDouble(keyLon, 0.0);
  }

  prefs.end();

  Serial.printf("Boundary loaded: %d points\n", boundaryCount);
}

// ============================================================
// SAVE BOUNDARY
// ============================================================

void saveBoundary() {

  prefs.begin("boundary", false);

  prefs.putInt("count", boundaryCount);

  prefs.putBool("enabled", boundaryEnabled);

  for (int i = 0; i < boundaryCount; i++) {

    char keyLat[12];
    char keyLon[12];

    snprintf(keyLat, sizeof(keyLat), "lat%d", i);

    snprintf(keyLon, sizeof(keyLon), "lon%d", i);

    prefs.putDouble(keyLat, boundary[i].lat);

    prefs.putDouble(keyLon, boundary[i].lon);
  }

  prefs.end();
}

// ============================================================
// LOAD ROADS
// ============================================================

void loadRoads() {

  prefs.begin("roads", true);

  for (int i = 0; i < MAX_ROADS; i++) {

    char key[16];

    snprintf(key, sizeof(key), "en%d", i);

    roads[i].enabled = prefs.getBool(key, false);

    snprintf(key, sizeof(key), "id%d", i);

    roads[i].id = prefs.getInt(key, i);

    snprintf(key, sizeof(key), "risk%d", i);

    roads[i].riskLevel = prefs.getInt(key, 0);

    snprintf(key, sizeof(key), "lat1_%d", i);

    roads[i].lat1 = prefs.getDouble(key, 0.0);

    snprintf(key, sizeof(key), "lon1_%d", i);

    roads[i].lon1 = prefs.getDouble(key, 0.0);

    snprintf(key, sizeof(key), "lat2_%d", i);

    roads[i].lat2 = prefs.getDouble(key, 0.0);

    snprintf(key, sizeof(key), "lon2_%d", i);

    roads[i].lon2 = prefs.getDouble(key, 0.0);
  }

  prefs.end();
}

// ============================================================
// SAVE ONE ROAD
// ============================================================

void saveRoad(int index) {

  if (index < 0 || index >= MAX_ROADS) {

    return;
  }

  prefs.begin("roads", false);

  char key[16];

  snprintf(key, sizeof(key), "en%d", index);

  prefs.putBool(key, roads[index].enabled);

  snprintf(key, sizeof(key), "id%d", index);

  prefs.putInt(key, roads[index].id);

  snprintf(key, sizeof(key), "risk%d", index);

  prefs.putInt(key, roads[index].riskLevel);

  snprintf(key, sizeof(key), "lat1_%d", index);

  prefs.putDouble(key, roads[index].lat1);

  snprintf(key, sizeof(key), "lon1_%d", index);

  prefs.putDouble(key, roads[index].lon1);

  snprintf(key, sizeof(key), "lat2_%d", index);

  prefs.putDouble(key, roads[index].lat2);

  snprintf(key, sizeof(key), "lon2_%d", index);

  prefs.putDouble(key, roads[index].lon2);

  prefs.end();
}

// ============================================================
// LOCAL PREDICTION ENGINE
// ============================================================

void runPrediction() {

  eventName = "NONE";

  if (!gps.location.isValid()) {

    riskScore = 0;

    highRiskStreak = 0;

    riskType = "NO_GPS";

    state = "WAITING";

    action = "NONE";

    setWarningActuators(false);

    return;
  }

  double lat = gps.location.lat();

  double lon = gps.location.lng();

  bool inside = isInsideBoundary(lat, lon);

  boundaryDistance = getBoundaryDistance(lat, lon);

  roadDistance = getNearestRoadDistance(lat, lon, nearestRoadRisk);

  // ----------------------------------------------------------
  // BOUNDARY RISK
  // ----------------------------------------------------------

  int boundaryRisk = 0;

  if (boundaryEnabled && boundaryDistance >= 0) {

    if (!inside) {

      boundaryRisk = 100;

    } else if (boundaryDistance < 5) {

      boundaryRisk = 95;

    } else if (boundaryDistance < 10) {

      boundaryRisk = 80;

    } else if (boundaryDistance < 20) {

      boundaryRisk = 55;

    } else if (boundaryDistance < 30) {

      boundaryRisk = 30;

    } else {

      boundaryRisk = 10;
    }

    // Moving away from the boundary:
    // reduce boundary risk.
    if (inside && previousBoundaryDistance >= 0 &&
        boundaryDistance > previousBoundaryDistance + 0.5) {

      boundaryRisk = min(boundaryRisk, 20);
    }
  }

  // ----------------------------------------------------------
  // ROAD RISK
  // ----------------------------------------------------------

  int roadRisk = 0;

  if (roadDistance >= 0 && nearestRoadRisk > 0) {

    if (roadDistance < 8) {

      roadRisk = nearestRoadRisk == 3 ? 95 : 75;

    } else if (roadDistance < 15) {

      roadRisk = nearestRoadRisk == 3 ? 85 : 65;

    } else if (roadDistance < 30) {

      roadRisk = nearestRoadRisk == 3 ? 65 : 45;

    } else if (roadDistance < 50) {

      roadRisk = nearestRoadRisk == 3 ? 35 : 25;

    } else {

      roadRisk = 5;
    }

    // Moving away from road:
    // lower the immediate road risk.
    if (previousRoadDistance >= 0 &&
        roadDistance > previousRoadDistance + 0.5) {

      roadRisk = min(roadRisk, 20);
    }
  }

  // ----------------------------------------------------------
  // OVERALL RISK
  // ----------------------------------------------------------

  int newRisk = max(boundaryRisk, roadRisk);

  if (newRisk >= 75) {

    state = "HIGH_RISK";

    action = "STEER_AWAY";

    highRiskStreak++;

    // Autonomous steering: activate buzzer and vibration when danger or
    // boundary breach is detected to turn the animal back to safety.
    setWarningActuators(true);

  } else if (newRisk >= 35) {

    state = "WARNING";

    action = "NONE";

    highRiskStreak = 0;

    // Approaching danger but not yet crossing: quiet actuators
    setWarningActuators(false);

  } else {

    state = "SAFE";

    action = "NONE";

    highRiskStreak = 0;

    // Animal is safe: silence actuators
    setWarningActuators(false);
  }

  // ----------------------------------------------------------
  // PRIMARY RISK TYPE
  // ----------------------------------------------------------

  if (boundaryRisk >= roadRisk && boundaryRisk >= 35) {

    riskType = "PERIMETER";

  } else if (roadRisk > boundaryRisk && roadRisk >= 35) {

    riskType = "HIGH_TRAFFIC_ROAD";

  } else {

    riskType = "SAFE";
  }

  // ----------------------------------------------------------
  // EVENTS
  // ----------------------------------------------------------

  if (previousRisk < 75 && newRisk >= 75) {

    eventName = "THREAT_DETECTED";

  } else if (previousRisk >= 75 && newRisk < 35) {

    eventName = "STEERED_AWAY";
  }

  riskScore = newRisk;

  previousRisk = riskScore;

  previousBoundaryDistance = boundaryDistance;

  previousRoadDistance = roadDistance;
}

// ============================================================
// SEND RADIO PACKET
// ============================================================

bool sendRadioPacket(const String &packet) {

  String tx = packet;

  int result = radio.transmit(tx);

  if (result == RADIOLIB_ERR_NONE) {

    return true;
  }

  Serial.printf("LoRa TX error: %d\n", result);

  return false;
}

// ============================================================
// SEND TELEMETRY
// ============================================================

void sendTelemetry() {

  // Update IMU immediately before prediction/telemetry.
  updateIMU();

  runPrediction();

  telemetrySequence++;

  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;

  double lon = gps.location.isValid() ? gps.location.lng() : 0.0;

  double speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

  double heading = gps.course.isValid() ? gps.course.deg() : 0.0;

  int satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;

  // IMPORTANT:
  // The existing telemetry packet format is unchanged.
  //
  // The old "acceleration" value is now the calibrated
  // dynamic acceleration so the existing gateway/dashboard
  // continues working without any protocol modification.

  double acceleration = imuOK ? currentDynamicAcceleration : 0.0;

  /*
   * Built as ONE snprintf into a fixed buffer, not ~20 chained String +=.
   *
   * Every += on a String is its own small heap allocation -- with a value
   * concatenated via String(x, precision), that is a temporary String
   * object constructed, copied in, and freed, PER FIELD, on this exact
   * function, every single telemetry cycle, forever. Over enough cycles the
   * heap fragments, and a fragmented heap is the textbook cause of the
   * "collar received a corrupted, spliced-together packet" symptom this
   * replaced -- not a wiring fault, not RF noise, just a buffer that no
   * longer allocated cleanly after running for hours. One buffer, one
   * write, no per-field heap churn.
   */
  char buf[256]; // real headroom over the string fields (riskType, state,
                 // action, eventName can each run ~20 chars)

  snprintf(buf, sizeof(buf),
           "TEL|%lu|%.6f|%.6f|%.2f|%.1f|%d|%.3f|%.1f|%.1f|%d|%s|%s|%s|%d|%d|%s",
           (unsigned long)telemetrySequence, lat, lon, speed, heading,
           satellites, acceleration, boundaryDistance, roadDistance, riskScore,
           riskType.c_str(), state.c_str(), action.c_str(), buzzerOn ? 1 : 0,
           vibrationOn ? 1 : 0, eventName.c_str());

  String packet(buf);

  // ----------------------------------------------------------
  // COLLAR SERIAL ENGINE DISPLAY
  // ----------------------------------------------------------

  Serial.println();

  Serial.println("========== COLLAR PREDICTION ==========");

  Serial.printf("GPS              : %.6f, %.6f\n", lat, lon);

  Serial.printf("Speed            : %.2f km/h\n", speed);

  Serial.printf("Heading          : %.1f deg\n", heading);

  Serial.printf("Satellites       : %d\n", satellites);

  Serial.println();

  Serial.printf("IMU X            : %.3f g\n", currentAccelX);

  Serial.printf("IMU Y            : %.3f g\n", currentAccelY);

  Serial.printf("IMU Z            : %.3f g\n", currentAccelZ);

  Serial.printf("IMU |A|          : %.3f g\n", currentAccelMagnitude);

  Serial.printf("Dynamic accel    : %.3f g\n", currentDynamicAcceleration);

  Serial.printf("Motion energy    : %.3f g\n", currentMotionEnergy);

  Serial.printf("IMU baseline     : %.3f g\n", baselineMagnitude);

  Serial.println();

  Serial.printf("Boundary         : %.1f m\n", boundaryDistance);

  Serial.printf("Road             : %.1f m\n", roadDistance);

  Serial.printf("Road risk        : %d\n", nearestRoadRisk);

  Serial.printf("Risk             : %d / 100\n", riskScore);

  Serial.printf("Type             : %s\n", riskType.c_str());

  Serial.printf("State            : %s\n", state.c_str());

  Serial.printf("Action           : %s\n", action.c_str());

  Serial.printf("Buzzer           : %s\n", buzzerOn ? "ON" : "OFF");

  Serial.printf("Vibration        : %s\n", vibrationOn ? "ON" : "OFF");

  Serial.printf("Event            : %s\n", eventName.c_str());

  Serial.println("========================================");

  // ----------------------------------------------------------
  // TRANSMIT
  // ----------------------------------------------------------

  if (sendRadioPacket(packet)) {

    Serial.println("Telemetry sent ✓");
  }
}

// ============================================================
// PARSE BOUNDARY COMMAND
//
// CMD|SET_BOUNDARY|count|lat|lon|lat|lon...
// ============================================================

bool parseBoundaryCommand(const String &msg) {

  int commandSeparator = msg.indexOf('|');

  if (commandSeparator < 0) {

    return false;
  }

  int countSeparator = msg.indexOf('|', commandSeparator + 1);

  if (countSeparator < 0) {

    return false;
  }

  // Packet layout is CMD|SET_BOUNDARY|count|lat|lon|...
  // The previous code accidentally parsed "SET_BOUNDARY" as the count,
  // which made every valid boundary command fail.
  int firstCoordinateSeparator = msg.indexOf('|', countSeparator + 1);

  if (firstCoordinateSeparator < 0) {

    return false;
  }

  int count =
      msg.substring(countSeparator + 1, firstCoordinateSeparator).toInt();

  // Diagnostic only. Without this, a rejected boundary gave no clue why --
  // in particular the exact "SET_BOUNDARY parsed as the count" regression
  // this function's header comment documents would silently show count=0
  // with no way to tell from Serial Monitor.
  Serial.printf(
      "parseBoundaryCommand: count field = \"%s\" -> parsed as %d\n",
      msg.substring(countSeparator + 1, firstCoordinateSeparator).c_str(),
      count);

  if (count < 3 || count > MAX_BOUNDARY_POINTS) {

    Serial.printf(
        "parseBoundaryCommand: REJECTED -- count %d out of range [3, %d]\n",
        count, MAX_BOUNDARY_POINTS);

    return false;
  }

  int cursor = firstCoordinateSeparator + 1;

  for (int i = 0; i < count; i++) {

    int next = msg.indexOf('|', cursor);

    if (next < 0) {

      // The header claimed `count` points but the message ran out early --
      // either a genuinely short payload, or (rare, since CRC is on) an
      // in-flight LoRa corruption that dropped a separator.
      Serial.printf("parseBoundaryCommand: REJECTED -- ran out of data at "
                    "point %d of %d\n",
                    i, count);

      return false;
    }

    boundary[i].lat = msg.substring(cursor, next).toDouble();

    cursor = next + 1;

    next = msg.indexOf('|', cursor);

    String lonString;

    if (next < 0) {

      lonString = msg.substring(cursor);

      cursor = msg.length();

    } else {

      lonString = msg.substring(cursor, next);

      cursor = next + 1;
    }

    boundary[i].lon = lonString.toDouble();
  }

  boundaryCount = count;

  boundaryEnabled = true;

  saveBoundary();

  Serial.println();
  Serial.println("BOUNDARY STORED LOCALLY ✓");

  for (int i = 0; i < boundaryCount; i++) {

    Serial.printf("P%d = %.6f, %.6f\n", i + 1, boundary[i].lat,
                  boundary[i].lon);
  }

  return true;
}

// ============================================================
// PARSE ROAD COMMAND
//
// CMD|SET_ROAD|index|id|risk|lat1|lon1|lat2|lon2
// ============================================================

bool parseRoadCommand(const String &msg) {

  const int REQUIRED_FIELDS = 9;

  String fields[REQUIRED_FIELDS];

  int fieldCount = 0;
  int start = 0;

  while (fieldCount < REQUIRED_FIELDS) {

    int separator = msg.indexOf('|', start);

    if (separator < 0) {

      fields[fieldCount++] = msg.substring(start);

      break;
    }

    fields[fieldCount++] = msg.substring(start, separator);

    start = separator + 1;
  }

  if (fieldCount != REQUIRED_FIELDS) {

    return false;
  }

  if (fields[1] != "SET_ROAD") {

    return false;
  }

  int index = fields[2].toInt();

  if (index < 0 || index >= MAX_ROADS) {

    return false;
  }

  roads[index].id = fields[3].toInt();

  roads[index].riskLevel = fields[4].toInt();

  roads[index].lat1 = fields[5].toDouble();

  roads[index].lon1 = fields[6].toDouble();

  roads[index].lat2 = fields[7].toDouble();

  roads[index].lon2 = fields[8].toDouble();

  roads[index].enabled = true;

  saveRoad(index);

  Serial.printf("ROAD %d STORED | risk=%d\n", index, roads[index].riskLevel);

  return true;
}

// ============================================================
// SANITIZE INCOMING PACKET
// ============================================================

String cleanRadioPacket(const String &raw) {

  String clean;

  clean.reserve(raw.length());

  for (size_t i = 0; i < raw.length(); i++) {

    char c = raw[i];

    if (c >= 32 && c <= 126) {

      clean += c;
    }
  }

  clean.trim();

  return clean;
}

// ============================================================
// HANDLE COMMAND
// ============================================================

void handleCommand(const String &raw) {

  String msg = cleanRadioPacket(raw);

  // Every real CMD| this collar issues is well under 200 chars. Anything
  // longer is not a bigger command -- it is two packets' worth of bytes
  // stuck together (radio buffer reuse, RF noise that still passed CRC by
  // chance). Reject it outright rather than letting a startsWith() check
  // below match on its first few bytes and act on a spliced packet.
  if (msg.length() > 200) {

    Serial.printf("REJECTED oversized/garbled packet (%d bytes): [%s]\n",
                  msg.length(), msg.c_str());

    return;
  }

  Serial.printf("Gateway command CLEAN: [%s]\n", msg.c_str());

  // ----------------------------------------------------------
  // PING
  // ----------------------------------------------------------

  if (msg == "CMD|PING" || msg.startsWith("CMD|PING|")) {

    sendRadioPacket("ACK|PONG");

    Serial.println("PONG sent ✓");

    return;
  }

  // ----------------------------------------------------------
  // TELEMETRY REQUEST
  // ----------------------------------------------------------

  if (msg.startsWith("CMD|REQ_TELEMETRY")) {

    // startsWith, not ==: an occasional stray tail byte after a TX/RX turn-
    // around (still under review, but harmless here) must not make a real,
    // valid request get dropped as "Unknown command" -- sendTelemetry()
    // takes nothing from the message, so trailing bytes cannot do harm.
    sendTelemetry();

    return;
  }

  // ----------------------------------------------------------
  // CLEAR BOUNDARY
  // ----------------------------------------------------------

  if (msg == "CMD|CLEAR_BOUNDARY") {

    boundaryCount = 0;

    boundaryEnabled = false;

    prefs.begin("boundary", false);

    prefs.clear();

    prefs.end();

    sendRadioPacket("ACK|BOUNDARY_CLEARED");

    Serial.println("Boundary cleared ✓");

    return;
  }

  // ----------------------------------------------------------
  // SET BOUNDARY
  // ----------------------------------------------------------

  if (msg.startsWith("CMD|SET_BOUNDARY|")) {

    if (parseBoundaryCommand(msg)) {

      sendRadioPacket("ACK|BOUNDARY_OK");

    } else {

      sendRadioPacket("ACK|BOUNDARY_FAIL");
    }

    return;
  }

  // ----------------------------------------------------------
  // SET ROAD
  // ----------------------------------------------------------

  if (msg.startsWith("CMD|SET_ROAD|")) {

    if (parseRoadCommand(msg)) {

      sendRadioPacket("ACK|ROAD_OK");

    } else {

      sendRadioPacket("ACK|ROAD_FAIL");
    }

    return;
  }

  // ----------------------------------------------------------
  // BEEP (manual "find the animal" command from the app)
  // ----------------------------------------------------------

  if (msg == "CMD|BEEP" || msg.startsWith("CMD|BEEP|")) {

    int seconds = 3;

    int bar = msg.indexOf('|', 4);

    if (bar >= 0) {
      int value = msg.substring(bar + 1).toInt();
      if (value > 0 && value <= 60) {
        seconds = value;
      }
    }

    manualBeepStartMs = millis();
    manualBeepDurationMs = (unsigned long)seconds * 1000UL;

    setWarningActuators(true);

    sendRadioPacket("ACK|BEEP_OK");

    Serial.printf("Manual beep for %d s ✓\n", seconds);

    return;
  }

  Serial.println("Unknown command.");
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(2500);

  Serial.println();
  Serial.println("============================================");

  Serial.println("      PAASHUGUARD - COLLAR");

  Serial.println("      LOCAL PREDICTION ENGINE");

  Serial.println("============================================");

  // ----------------------------------------------------------
  // ACTUATORS
  // ----------------------------------------------------------

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(VIBRATION_PIN, OUTPUT);

  setWarningActuators(false);

  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Serial.println("GPS serial opened.");

  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  Wire.begin(I2C_SDA, I2C_SCL);

  Wire.beginTransmission(MPU_ADDR);

  if (Wire.endTransmission() == 0) {

    // Wake MPU6050.
    mpuWrite(0x6B, 0x00);

    delay(200);

    // ±8g accelerometer range.
    //
    // Keep this unchanged from your original code.
    //
    mpuWrite(0x1C, 0x10);

    // ±500 deg/s gyro range.
    mpuWrite(0x1B, 0x08);

    // Digital low-pass filter.
    mpuWrite(0x1A, 0x03);

    delay(50);

    uint8_t range = (mpuRead(0x1C) >> 3) & 0x03;

    accelScale = (range == 0)   ? 16384.0
                 : (range == 1) ? 8192.0
                 : (range == 2) ? 4096.0
                                : 2048.0;

    imuOK = true;

    Serial.println("IMU hardware detected ✓");

    Serial.printf("Accelerometer scale: %.0f LSB/g\n", accelScale);

    // --------------------------------------------------------
    // IMPORTANT:
    // CALIBRATE AFTER HARDWARE SETUP.
    // KEEP COLLAR STILL.
    // --------------------------------------------------------

    calibrateIMU();

  } else {

    Serial.println("IMU FAIL");

    imuOK = false;
  }

  // ----------------------------------------------------------
  // LOAD STORED DATA
  // ----------------------------------------------------------

  loadBoundary();
  loadRoads();

  // ----------------------------------------------------------
  // LORA
  // ----------------------------------------------------------

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int result = -1;

  for (int attempt = 1; attempt <= 3; attempt++) {

    result = radio.begin(868.0);

    if (result == RADIOLIB_ERR_NONE) {

      loraOK = true;

      break;
    }

    Serial.printf("LoRa attempt %d failed: %d\n", attempt, result);

    delay(400);
  }

  if (!loraOK) {

    Serial.printf("LoRa FAILED: %d\n", result);

    while (true) {
      delay(1000);
    }
  }

  // Keep existing working LoRa configuration.
  radio.setSpreadingFactor(10);

  radio.setBandwidth(125.0);

  radio.setCodingRate(5);

  radio.setOutputPower(14);

  radio.setSyncWord(0x34);

  radio.setCRC(true);

  Serial.println();
  Serial.println("LoRa OK ✓");

  Serial.println("868 MHz / SF10 / BW125 / CR4/5");

  Serial.println();
  Serial.println("COLLAR READY");

  Serial.println("Prediction engine ACTIVE");

  Serial.println("Waiting for Gateway...");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  // GPS must always be fed.
  updateGPS();

  // Keep the IMU updated continuously too.
  updateIMU();

  // End a manual beep once its window is up.
  if (manualBeepDurationMs != 0 &&
      millis() - manualBeepStartMs >= manualBeepDurationMs) {
    manualBeepDurationMs = 0;
    setWarningActuators(false);
  }

  if (!loraOK) {
    return;
  }

  String incoming;

  int result = radio.receive(incoming, 100, 0);

  if (result == RADIOLIB_ERR_NONE) {

    Serial.printf("LoRa RX RAW: [%s]\n", incoming.c_str());

    handleCommand(incoming);
  }
}

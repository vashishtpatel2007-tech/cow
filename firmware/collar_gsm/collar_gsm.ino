

#define TINY_GSM_MODEM_SIM7600 // EC200U shares AT commands with SIM7600
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <math.h>

// --- GSM Pins ---
#define GSM_TX 17
#define GSM_RX 16
#define GSM_BAUD 115200
HardwareSerial SerialGSM(2);

// --- TinyGSM Setup ---
TinyGsm modem(SerialGSM);
TinyGsmClientSecure client(modem);
const char server[] = "YOUR_PROJECT_ID.supabase.co";
const int port = 443;
HttpClient http(client, server, port);

// --- API Keys ---
const char anonKey[] = "YOUR_ANON_KEY";

// --- State Variables ---
float currentLat = 0.0;
float currentLon = 0.0;
String deviceId = "COLLAR-001";

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Initializing GSM Module...");
  SerialGSM.begin(GSM_BAUD, SERIAL_8N1, GSM_RX, GSM_TX);
  
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, checking connection...");
  }
  
  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");

  Serial.print("Connecting to GPRS...");
  if (!modem.gprsConnect("airtelgprs.com", "", "")) { // Replace APN
    Serial.println(" fail");
    delay(10000);
    return;
  }
  Serial.println(" success");
}

void loop() {
  // Simulate GPS coordinates
  currentLat = 12.9716;
  currentLon = 77.5946;

  sendTelemetry();
  fetchCommands();
  
  delay(30000); // 30 sec interval
}

void sendTelemetry() {
  Serial.println("Sending Telemetry over HTTPS...");
  
  String payload = "{\"device_id\":\"" + deviceId + "\",\"lat\":" + String(currentLat, 6) + ",\"lon\":" + String(currentLon, 6) + "}";
  
  http.beginRequest();
  http.post("/functions/v1/ingest");
  http.sendHeader("Content-Type", "application/json");
  http.sendHeader("Authorization", String("Bearer ") + anonKey);
  http.sendHeader("Content-Length", payload.length());
  http.beginBody();
  http.print(payload);
  http.endRequest();
  
  int statusCode = http.responseStatusCode();
  String response = http.responseBody();
  
  Serial.print("Status code: ");
  Serial.println(statusCode);
  Serial.print("Response: ");
  Serial.println(response);
}

void fetchCommands() {
  // Fetch commands from Supabase directly via REST API
  Serial.println("Fetching commands over HTTPS...");
  
  http.beginRequest();
  http.get("/rest/v1/device_commands?device_id=eq." + deviceId + "&status=eq.pending");
  http.sendHeader("apikey", anonKey);
  http.sendHeader("Authorization", String("Bearer ") + anonKey);
  http.endRequest();
  
  int statusCode = http.responseStatusCode();
  String response = http.responseBody();
  
  Serial.print("Command Status: ");
  Serial.println(statusCode);
  Serial.print("Commands: ");
  Serial.println(response);
}

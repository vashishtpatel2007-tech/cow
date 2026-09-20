
#define TINY_GSM_DEBUG Serial
#define TINY_GSM_MODEM_SIM7600 // We keep this just for the basic modem setup
#include <TinyGsmClient.h>

// --- UART PINS ---
#define ESP32_RX_PIN 6  
#define ESP32_TX_PIN 7  
#define MODEM_PWRKEY 4
#define GSM_BAUD 115200

// --- NETWORK SETTINGS ---
const char apn[] = "airtelgprs.com";
const char gprsUser[] = "";
const char gprsPass[] = "";

// --- SUPABASE CLOUD SETTINGS ---
const String url = "https://bzufqeuaordhrykgsiub.supabase.co/functions/v1/ingest";
const String host = "bzufqeuaordhrykgsiub.supabase.co";
const String secret = "6be3390d243cfc602fbce1b5985ef88dd320fdb79593d35640a76bc9a17da336";

HardwareSerial SerialGSM(1);
TinyGsm modem(SerialGSM);

// Simulated Cow Position
float cowLat = 13.082700;
float cowLon = 77.587700;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== GSM NATIVE TELEMETRY TEST ===");

  SerialGSM.begin(GSM_BAUD, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);

  // Check if modem is already awake before pressing the power button
  if (!modem.testAT()) {
    Serial.println("Modem is asleep. Pressing PWRKEY to wake it up...");
    pinMode(MODEM_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, HIGH);
    
    // Wait for the Linux OS inside the modem to boot
    Serial.println("Waiting 10 seconds for modem to boot...");
    delay(10000); 
  } else {
    Serial.println("Modem is already awake!");
  }

  SerialGSM.begin(GSM_BAUD, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);
  
  Serial.println("Initializing Modem...");
  if (!modem.init()) {
    Serial.println("[ERROR] Modem failed to initialize.");
    while(true);
  }
  
  Serial.print("Connecting to Airtel Network...");
  if (!modem.waitForNetwork(60000L)) {
    Serial.println(" [FAILED]");
    while(true);
  }
  Serial.println(" [OK]");
  
  Serial.print("Connecting to GPRS (Native Quectel Mode)...");
  
  // 0. Deactivate any stuck or existing connection first!
  modem.sendAT("+QIDEACT=1");
  modem.waitResponse(10000L);
  
  // 1. Configure the PDP Context natively for Quectel
  modem.sendAT("+QICSGP=1,1,\"", apn, "\",\"", gprsUser, "\",\"", gprsPass, "\",1");
  modem.waitResponse(10000L);
  
  // 2. Activate the PDP Context
  modem.sendAT("+QIACT=1");
  if (modem.waitResponse(60000L, "OK", "ERROR") != 1) {
    Serial.println(" [FAILED]");
    // Check if it's already active or get error code
    modem.sendAT("+QIACT?");
    modem.waitResponse();
    while(true) delay(100);
  }
  Serial.println(" [OK]");
  
  // Configure the Modem's internal HTTP engine
  Serial.println("Configuring Native HTTP Engine...");
  
  // 0. Link HTTP to PDP Context 1
  modem.sendAT("+QHTTPCFG=\"contextid\",1");
  modem.waitResponse();
  
  // 1. Enable custom HTTP headers
  modem.sendAT("+QHTTPCFG=\"requestheader\",1");
  modem.waitResponse();
  
  // 3. Disable SSL Certificate Verification
  modem.sendAT("+QSSLCFG=\"seclevel\",1,0");
  modem.waitResponse();

  // 3.5 Force TLS 1.2 (Supabase strictly rejects TLS 1.0/1.1)
  modem.sendAT("+QSSLCFG=\"sslversion\",1,4");
  modem.waitResponse();
  
  // 3.6 Enable SNI (Supabase Edge network requires SNI to route the handshake)
  modem.sendAT("+QSSLCFG=\"sni\",1,1");
  modem.waitResponse();
  
  // 3.7 Enable all cipher suites
  modem.sendAT("+QSSLCFG=\"ciphersuite\",1,0xFFFF");
  modem.waitResponse();
  
  // 4. Link HTTP to SSL context 1
  modem.sendAT("+QHTTPCFG=\"sslctxid\",1");
  modem.waitResponse();
  
  Serial.println("System Ready. Beginning Telemetry Loop...");
}

bool sendNativeHTTP(String payload) {
  // 1. Set the URL
  modem.sendAT("+QHTTPURL=", url.length(), ",80");
  if (modem.waitResponse(10000L, "CONNECT") != 1) {
    Serial.println("[ERROR] Failed to set URL");
    return false;
  }
  modem.streamWrite(url);
  modem.waitResponse(); // Wait for OK

  // 2. Build the Raw HTTP 1.1 Request
  String request = "POST /functions/v1/ingest HTTP/1.1\r\n";
  request += "Host: " + host + "\r\n";
  request += "Content-Type: application/json\r\n";
  request += "x-device-secret: " + secret + "\r\n";
  request += "Content-Length: " + String(payload.length()) + "\r\n\r\n";
  request += payload;

  // 3. Send the Request
  Serial.println("Sending AT+QHTTPPOST...");
  modem.sendAT("+QHTTPPOST=", request.length(), ",80,80");
  
  bool connectSuccess = false;
  long t = millis();
  while(millis() - t < 15000) {
      if (modem.stream.available()) {
          String s = modem.stream.readStringUntil('\n');
          s.trim();
          if (s.length() > 0) {
              Serial.println("MODEM SAYS: " + s);
              if (s == "CONNECT") {
                  connectSuccess = true;
                  break;
              }
              if (s.indexOf("ERROR") != -1) {
                  Serial.println("[FATAL] Modem rejected the POST request.");
                  return false;
              }
          }
      }
  }
  
  if (!connectSuccess) {
      Serial.println("[ERROR] Timeout waiting for CONNECT");
      return false;
  }
  
  modem.streamWrite(request);
  
  // 4. Wait for the server's asynchronous reply URC
  Serial.println("Waiting for server response URC...");
  bool postSuccess = false;
  t = millis();
  while(millis() - t < 30000) {
      if (modem.stream.available()) {
          String s = modem.stream.readStringUntil('\n');
          s.trim();
          if (s.length() > 0) {
              Serial.println("MODEM SAYS: " + s);
              if (s.startsWith("+QHTTPPOST:")) {
                  postSuccess = true;
                  break;
              }
          }
      }
  }
  
  if (!postSuccess) {
      Serial.println("[ERROR] Server never replied to POST");
      return false;
  }

  // 5. Actually read the JSON response (Downlink commands) from Supabase
  modem.sendAT("+QHTTPREAD=80");
  if (modem.waitResponse(10000L, "CONNECT") != 1) {
    Serial.println("[ERROR] Modem refused to output HTTP response body");
    return false;
  }

  // The modem is now streaming the raw JSON response to us. Read it!
  String response = "";
  long readStart = millis();
  while (millis() - readStart < 5000) {
    if (modem.stream.available()) {
      String line = modem.stream.readStringUntil('\n');
      line.trim();
      if (line == "OK") break; // End of HTTP body
      if (line.length() > 0) {
        response += line;
      }
    }
  }

  Serial.println("\n--- DOWNLINK RECEIVED ---");
  Serial.println("Supabase says: " + response);
  Serial.println("-------------------------\n");

  // Wait for the final +QHTTPREAD: 0 success message
  modem.waitResponse(5000L, "+QHTTPREAD:");

  // Supabase edge functions don't always return "ok: true" if it's not custom programmed, 
  // but if we get a response body that doesn't say "error", it usually succeeded.
  if (response.indexOf("error") == -1) {
    return true;
  }
  
  Serial.println("[ERROR] Supabase returned an error inside the body");
  return false;
}

void loop() {
  Serial.println("\n--- Sending GPS Data to Supabase ---");
  
  String payload = "{";
  payload += "\"device_id\":\"PASHU-A01\",";
  payload += "\"animal_id\":\"\",";
  payload += "\"lat\":" + String(cowLat, 6) + ",";
  payload += "\"lon\":" + String(cowLon, 6) + ",";
  payload += "\"speed_kmh\":1.2,";
  payload += "\"heading_deg\":45,";
  payload += "\"fix_quality\":3,";
  payload += "\"hdop\":1.2,";
  payload += "\"sats\":8,";
  payload += "\"movement_state\":1,";
  payload += "\"event_code\":0,";
  payload += "\"seq\":999,";
  payload += "\"battery_pct\":95,";
  payload += "\"recorded_at\":\"2026-08-28T00:00:00Z\"";
  payload += "}";

  Serial.println("Payload: " + payload);
  Serial.println("Transmitting via Native AT HTTP...");

  if (sendNativeHTTP(payload)) {
    Serial.println("[SUCCESS] Data reached Supabase!");
  } else {
    Serial.println("[ERROR] Transmission Failed.");
  }

  // Walk slightly Northeast
  cowLat += 0.000050;
  cowLon += 0.000050;

  Serial.println("Waiting 2 seconds...");
  delay(2000);
}

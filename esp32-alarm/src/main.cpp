#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "config.h"

WiFiClientSecure client;

bool alarmActive = false;
unsigned long alarmStartTime = 0;
unsigned long lastMotionTime = 0;
bool motionDetected = false;

// Motion confirmation tracking
int motionEventCount = 0;
unsigned long firstMotionTime = 0;
unsigned long motionStartTime = 0;
bool continuousMotion = false;
unsigned long startupTime = 0;

void sendWebhook(const char* event) {
  if (!client.connect(WEBHOOK_HOST, WEBHOOK_PORT)) {
    Serial.println("Connection to webhook failed");
    return;
  }

  StaticJsonDocument<256> doc;
  String message;
  
  if (strcmp(event, "alarm_triggered") == 0) {
    message = "🚨 **ALARM TRIGGERED!** 🚨\nMotion detected by ESP32 alarm system!";
  } else if (strcmp(event, "alarm_stopped") == 0) {
    message = "✅ Alarm has been deactivated";
  } else {
    message = String("Alarm event: ") + event;
  }
  
  doc["content"] = message;
  doc["username"] = "ESP32 Alarm System";
  
  String jsonString;
  serializeJson(doc, jsonString);

  String request = String("POST ") + WEBHOOK_PATH + " HTTP/1.1\r\n" +
                   "Host: " + WEBHOOK_HOST + "\r\n" +
                   "Content-Type: application/json\r\n" +
                   "Content-Length: " + jsonString.length() + "\r\n" +
                   "Connection: close\r\n\r\n" +
                   jsonString;

  client.print(request);

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      break;
    }
  }

  String response = client.readString();
  Serial.println("Webhook response: " + response);
  
  client.stop();
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(PIR_PIN, INPUT);
  pinMode(ALARM_CONTROL_PIN, OUTPUT);
  digitalWrite(ALARM_CONTROL_PIN, LOW);
  
  // PIR sensor stabilization
  Serial.print("Waiting for PIR sensor to stabilize (");
  Serial.print(PIR_STABILIZATION_TIME / 1000);
  Serial.println(" seconds)...");
  
  for (int i = 0; i < PIR_STABILIZATION_TIME / 1000; i++) {
    int pirLevel = digitalRead(PIR_PIN);
    Serial.print("PIR stabilization ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(PIR_STABILIZATION_TIME / 1000);
    Serial.print(": GPIO");
    Serial.print(PIR_PIN);
    Serial.print(" = ");
    Serial.println(pirLevel);
    delay(1000);
  }
  
  Serial.println("PIR stabilization complete");
  
  connectToWiFi();
  
  client.setInsecure(); // For HTTPS without certificate verification
  
  startupTime = millis();
  
  Serial.println("ESP32 Alarm System Ready");
  Serial.println("PIR Sensor on GPIO4");
  Serial.println("Alarm Control on GPIO10");
  Serial.print("Startup grace period: ");
  Serial.print(STARTUP_GRACE_PERIOD / 1000);
  Serial.println(" seconds");
}

void loop() {
  // Check PIR sensor
  int pirState = digitalRead(PIR_PIN);
  unsigned long currentTime = millis();
  
  // Check for startup grace period
  bool startupGracePeriod = (currentTime - startupTime < STARTUP_GRACE_PERIOD);
  
  // Motion detection with confirmation logic
  if (pirState == HIGH) {  // Motion detected by PIR
    if (!continuousMotion) {
      continuousMotion = true;
      motionStartTime = currentTime;
    }
    
    // Check if motion has been continuous for minimum duration
    if ((currentTime - motionStartTime) >= MIN_MOTION_DURATION) {
      if (!motionDetected && !startupGracePeriod &&
          (currentTime - lastMotionTime > PIR_DEBOUNCE_TIME)) {
        
        // First motion event or new motion after debounce
        if (motionEventCount == 0 || 
            (currentTime - firstMotionTime > MOTION_CONFIRMATION_WINDOW)) {
          // Start new confirmation window
          motionEventCount = 1;
          firstMotionTime = currentTime;
          Serial.print("Motion event 1/");
          Serial.print(MOTION_CONFIRMATION_COUNT);
          Serial.println(" detected, waiting for confirmation...");
        } else {
          // Additional motion within confirmation window
          motionEventCount++;
          Serial.print("Motion event ");
          Serial.print(motionEventCount);
          Serial.print("/");
          Serial.print(MOTION_CONFIRMATION_COUNT);
          Serial.println(" detected");
          
          // Check if we have enough confirmations
          if (motionEventCount >= MOTION_CONFIRMATION_COUNT && !alarmActive) {
            alarmActive = true;
            alarmStartTime = currentTime;
            digitalWrite(ALARM_CONTROL_PIN, HIGH);
            Serial.print("ALARM ACTIVATED! (confirmed after ");
            Serial.print(motionEventCount);
            Serial.println(" motion events)");
            
            sendWebhook("alarm_triggered");
            motionEventCount = 0;  // Reset for next detection
          }
        }
        
        motionDetected = true;
        lastMotionTime = currentTime;
      }
    }
  } else {  // No motion (pirState == LOW)
    continuousMotion = false;
    motionDetected = false;
    
    // Reset motion count if confirmation window expired
    if (motionEventCount > 0 && 
        (currentTime - firstMotionTime > MOTION_CONFIRMATION_WINDOW)) {
      Serial.println("Motion confirmation window expired, resetting count");
      motionEventCount = 0;
    }
  }
  
  // Log ignored motion during grace period
  if (pirState == HIGH && startupGracePeriod) {
    static unsigned long lastGraceLog = 0;
    if (currentTime - lastGraceLog > 5000) {  // Log every 5 seconds max
      Serial.print("Motion ignored during startup grace period (");
      Serial.print((STARTUP_GRACE_PERIOD - (currentTime - startupTime)) / 1000);
      Serial.print("/");
      Serial.print(STARTUP_GRACE_PERIOD / 1000);
      Serial.println(" seconds remaining)");
      lastGraceLog = currentTime;
    }
  }
  
  // Check if alarm should be turned off
  if (alarmActive && (currentTime - alarmStartTime > ALARM_DURATION)) {
    alarmActive = false;
    digitalWrite(ALARM_CONTROL_PIN, LOW);
    Serial.println("Alarm deactivated");
    
    sendWebhook("alarm_stopped");
  }
  
  // Reconnect WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  
  delay(100);
}
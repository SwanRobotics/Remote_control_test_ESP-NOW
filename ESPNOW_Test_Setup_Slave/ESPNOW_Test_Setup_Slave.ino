/*
 * Example from www.swanrobotics.com
 *
 * ESP-NOW communication example for two ESP32 modules.
 *
 * ESP-NOW Receiver (Slave):
 * - Receives joystick data and heartbeat from Master ESP32.
 * - Maps joystick value to a servo position.
 * - Reads distance from ultrasonic sensor and sends it back to the Master.
 * - Checks for heartbeat
 */

#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

// === Ultrasonic Sensor Pins ===
#define TRIG_PIN 5
#define ECHO_PIN 18

// === Servo ===
Servo myServo;
const int SERVO_PIN = 4;  // PWM pin for servo
int servoPosition = 0;    // Current servo angle

// === Received Data Struct ===
typedef struct {
  int servo;               // Joystick X value mapped to servo
  unsigned long heartbeat; // Heartbeat timestamp
} IncomingMessage;

IncomingMessage incomingData;

// === Outgoing Data Struct===
typedef struct {
  float distance_cm; // Measured distance in cm
} OutgoingMessage;

OutgoingMessage outgoingData;

// === Master MAC Address ===
// Replace with your Master ESP32 MAC address
uint8_t masterMAC[] = {0x78, 0x21, 0x84, 0xC6, 0xAA, 0x14};

// === Heartbeat Handling ===
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_TIMEOUT = 1000;  // ms

// === Callback: Data Received ===
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  lastHeartbeat = millis();

  Serial.print("Heartbeat: ");
  Serial.println(incomingData.heartbeat);
  Serial.print("Joystick Value: ");
  Serial.println(incomingData.servo);

  // === Control Servo ===
  servoPosition = map(incomingData.servo, 0, 4095, 0, 180);
  servoPosition = constrain(servoPosition, 0, 180);
  myServo.write(servoPosition);

  // === Measure Distance and Send Response ===
  outgoingData.distance_cm = readUltrasonic();
  esp_err_t result = esp_now_send(masterMAC, (uint8_t *)&outgoingData, sizeof(outgoingData));
  if (result != ESP_OK) {
    Serial.println("❌ Failed to send response");
  }
}

// === Ultrasonic Sensor Read ===
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30 ms

  if (duration == 0) {
    return -1; // No reading
  }

  float distance = duration * 0.034 / 2;  // cm
  return distance;
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Wi-Fi STA mode required for ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Register Master as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer (Master)");
  } else {
    Serial.println("✔ Master peer added");
  }

  // Initialize Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myServo.setPeriodHertz(50); // Standard 50 Hz servo
  myServo.attach(SERVO_PIN, 500, 2400);
}

// === Loop ===
void loop() {
  // Heartbeat timeout handling
  if (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT) {
    Serial.println("⛔ Connection lost - Centering servo");
    myServo.write(90);  // Safe default position
  }

  delay(100);
}

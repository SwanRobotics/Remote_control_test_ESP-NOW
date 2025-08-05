/*
 * Example from www.swanrobotics.com
 *
 * ESP-NOW communication example for two ESP32 modules.
 * 
 * ESP-NOW Sender (Master):
 * - Reads a joystick X-axis value
 * - Sends a heartbeat and the X-axis value
 * - Prints distance send by Slave
 */

#include <esp_now.h>
#include <WiFi.h>

// === Pin Definitions ===
#define VRX_PIN 34  // ESP32 GPIO34 (ADC1_CH6) connected to VRX pin

// === Joystick State ===
int joystickX = 0;  // Stores the current X-axis reading

// === ESP-NOW Outgoing Data ===
typedef struct {
  int valueX;              // Joystick X-axis value
  unsigned long heartbeat; // Heartbeat timestamp (ms)
} OutgoingMessage;

OutgoingMessage outgoingData;

// === Receiver MAC Address ===
// Replace with the receiver's MAC address
uint8_t receiverMAC[] = {0x40, 0x22, 0xD8, 0x60, 0xE3, 0xCC};

// === ESP-NOW Incoming Data ===
typedef struct {
  float distance_cm; // Example response: distance in centimeters
} IncomingMessage;

IncomingMessage incomingData;

// === Callback: Data Sent ===
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✔ Success" : "❌ Failed");
}

// === Callback: Data Received ===
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  Serial.print("Received Distance: ");
  Serial.print(incomingData.distance_cm);
  Serial.println(" cm");
}

// === Read Joystick X-Axis ===
void readJoystick() {
  joystickX = analogRead(VRX_PIN);
}

// === Print Joystick Data ===
void printJoystickData() {
  Serial.print("Joystick X: ");
  Serial.println(joystickX);
}

// === Arduino Setup ===
void setup() {
  Serial.begin(115200);

  // Set device as Wi-Fi Station (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Ensure disconnected from any AP

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer (receiver)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;    // Use default channel
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    return;
  }
  Serial.println("✔ Peer added");

  // Initialize outgoing data
  outgoingData.valueX = 0;
  outgoingData.heartbeat = 0;
}

// === Arduino Loop ===
void loop() {
  readJoystick();
  printJoystickData();

  outgoingData.valueX = joystickX;
  outgoingData.heartbeat = millis();

  // Send data to receiver
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *) &outgoingData, sizeof(outgoingData));

  if (result != ESP_OK) {
    Serial.println("❌ Error sending data");
  }

  delay(200);  // Send every 200 ms
}

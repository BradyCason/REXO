#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

// ESP32 NOW Definitions ------------------------------------------------------------------------------------------

#define ESPNOW_WIFI_CHANNEL 6

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin() || !add()) {
      log_e("Failed to initialize ESP-NOW or register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};

ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);


// Flex Sensor Functions ------------------------------------------------------------------------------------------

int adcPins[5] = {34, 35, 32, 33, 36};
int adcValues[5];

void init_flex_pins(){
  for (int i = 0; i < 5; i++) {
    analogSetPinAttenuation(adcPins[i], ADC_ATTENDB_MAX);
  }

  // 12-bit resolution: 0–4095
  analogReadResolution(12);
}

void read_flex_pins(){
  for (int i = 0; i < 5; i++) {
    adcValues[i] = analogRead(adcPins[i]);
  }
}

void send_flex_values(){
  char data[64];
  snprintf(data, sizeof(data), "%d,%d,%d,%d,%d",
           adcValues[0],
           adcValues[1],
           adcValues[2],
           adcValues[3],
           adcValues[4]);

  Serial.printf("Broadcasting message: %s\n", data);

  if (!broadcast_peer.send_message((uint8_t *)data, sizeof(data))) {
    Serial.println("Failed to broadcast message");
  }
}

uint32_t msg_count = 0;

// Main Functions ------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  // Register the broadcast peer
  if (!broadcast_peer.begin()) {
    Serial.println("Failed to initialize broadcast peer");
    Serial.println("Reebooting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  init_flex_pins();
}

void loop() {
  read_flex_pins();

  send_flex_values();

  delay(100);
}
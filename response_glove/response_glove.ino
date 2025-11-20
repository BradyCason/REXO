#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#include <vector>

// ESP32 NOW Definitions ------------------------------------------------------------------------------------------

#define ESPNOW_WIFI_CHANNEL 6

class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  // Function to print the received messages from the master
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    Serial.printf("  Message: %s\n", (char *)data);
  }
};

std::vector<ESP_NOW_Peer_Class *> masters;

void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    ESP_NOW_Peer_Class *new_master = new ESP_NOW_Peer_Class(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (!new_master->add_peer()) {
      Serial.println("Failed to register the new master");
      delete new_master;
      return;
    }
    masters.push_back(new_master);
    Serial.printf("Successfully registered master " MACSTR " (total masters: %zu)\n", MAC2STR(new_master->addr()), masters.size());
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}


// Servo Functions

const int SERVO_PIN[5] = {18, 19, 21, 22, 23};
const int PWM_FREQ = 50;
const int PWM_CHANNEL[5] = {0, 1, 2, 3, 4};     // LEDC channel
const int PWM_RES = 16;        // 16-bit resolution

void init_servos(){
  for (int i = 0; i < 5; ++i){
    ledcSetup(PWM_CHANNEL[i], PWM_FREQ, PWM_RES);
    ledcAttachPin(SERVO_PIN[i], PWM_CHANNEL[i]);

    // Start stopped
    ledcWrite(PWM_CHANNEL[i], usToDuty(1500));
  }
}

void setPulse(int ch, int us) {
  uint32_t maxDuty = 65535;  
  uint32_t duty = (uint32_t)((float)us / 20000.0 * maxDuty);
  ledcWrite(ch, duty);
}


// Main Functions ------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW Example - Broadcast Slave");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  Serial.printf("ESP-NOW version: %d, max data length: %d\n", ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, nullptr);

  Serial.println("Setup complete. Waiting for a master to broadcast a message...");
}

void loop() {
  // Print debug information every 10 seconds
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 10000) {
    last_debug = millis();
    Serial.printf("Registered masters: %zu\n", masters.size());
    for (size_t i = 0; i < masters.size(); i++) {
      if (masters[i]) {
        Serial.printf("  Master %zu: " MACSTR "\n", i, MAC2STR(masters[i]->addr()));
      }
    }
  }

  delay(100);
}

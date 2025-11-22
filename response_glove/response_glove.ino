#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#include <vector>

#define MIN_SERVO_SPEED 1000
#define MAX_SERVO_SPEED 2000
#define MIN_FLEX_VALUE 700
#define MAX_FLEX_VALUE 2600

// Flex Sensor Functions ------------------------------------------------------------------------------------------

int adcPins[5] = {34, 35, 32, 33, 36};
int adcValues[5];
int target_flex_values[5];

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

    String csv = String((char *)data);
    int start = 0;
    for(int i = 0; i < 5; ++i){
      int commaPos = csv.indexOf(',', start);
      if (commaPos == -1) {
        // Last value
        target_flex_values[5] = csv.substring(start).toInt();
        break;
      }
      target_flex_values[i] = csv.substring(start, commaPos).toInt();
      start = commaPos + 1;
    }
  }
};

std::vector<ESP_NOW_Peer_Class *> masters;

void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {

    ESP_NOW_Peer_Class *new_master = new ESP_NOW_Peer_Class(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (!new_master->add_peer()) {
      Serial.println("Failed to register the new master");
      delete new_master;
      return;
    }
    masters.push_back(new_master);
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}


// Servo Functions ------------------------------------------------------------------------------------------

const int SERVO_PIN[5] = {18, 19, 21, 22, 23};
const int PWM_FREQ = 50;
const int PWM_RES = 16;        // 16-bit resolution

void servoWriteMicroseconds(int pin, int us) {
  // Map microseconds to duty cycle
  // 50 Hz period = 20,000 µs
  uint32_t duty = (uint32_t)((us / 20000.0) * ((1 << PWM_RES) - 1));
  ledcWrite(pin, duty);
}

void init_servos(){
  for (int i = 0; i < 5; ++i){
    ledcAttach(SERVO_PIN[i], PWM_FREQ, PWM_RES);

    // Start stopped
    servoWriteMicroseconds(SERVO_PIN[i], 1500);
  }
}

void set_servo_speeds(){
  for (int i = 0; i < 5; ++i){
    int dif = adcValues[i] - target_flex_values[i];
    float normalized_dif = (float)dif / (float)(MAX_FLEX_VALUE - MIN_FLEX_VALUE) * (float)(MAX_SERVO_SPEED - MIN_SERVO_SPEED) / 2;
    Serial.println(normalized_dif);
    servoWriteMicroseconds(SERVO_PIN[i], max(MIN_SERVO_SPEED, min(MIN_SERVO_SPEED + (MAX_SERVO_SPEED - MIN_SERVO_SPEED) / 2 + (int)(normalized_dif), MAX_SERVO_SPEED)));
    // if (dif > 0){
    //   servoWriteMicroseconds(SERVO_PIN[i], MIN_SERVO_SPEED);
    // }
    // else if (dif < 0){
    //   servoWriteMicroseconds(SERVO_PIN[i], MAX_SERVO_SPEED);
    // }
    // else{
    //   servoWriteMicroseconds(SERVO_PIN[i], (MAX_SERVO_SPEED + MIN_SERVO_SPEED) / 2);
    // }
  }
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

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, nullptr);

  init_servos();
  init_flex_pins();
}

void loop() {
  read_flex_pins();
  set_servo_speeds();

  // servoWriteMicroseconds(SERVO_PIN[0], 1000 + 1000 * (adcValues[0] - 1200) / 1200);

  delay(100);
}

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#include <vector>

// 1500 is not moving
// #define MIN_SERVO_SPEED 1000
// #define MAX_SERVO_SPEED 2000
#define MIN_SERVO_SPEED 1250
#define MAX_SERVO_SPEED 1750
#define SERVO_SPEED_COEFFICIENT 250

#define MIN_FLEX_VALUE 700
#define MAX_FLEX_VALUE 3100

// Flex Sensor Functions ------------------------------------------------------------------------------------------

int adcPins[5] = {32, 33, 34, 35, 36};
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

  // char data[64];
  // snprintf(data, sizeof(data), "%d,%d,%d,%d,%d",
  //          adcValues[0],
  //          adcValues[1],
  //          adcValues[2],
  //          adcValues[3],
  //          adcValues[4]);

  // Serial.println(data);
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
        target_flex_values[4] = csv.substring(start).toInt();
        break;
      }
      target_flex_values[i] = csv.substring(start, commaPos).toInt();
      start = commaPos + 1;
    }

    // char print_data[64];
    // snprintf(print_data, sizeof(print_data), "%d,%d,%d,%d,%d",
    //         target_flex_values[0],
    //         target_flex_values[1],
    //         target_flex_values[2],
    //         target_flex_values[3],
    //         target_flex_values[4]);

    // Serial.println(print_data);
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
// index, thumb, ring, pinky, middle

// Control Glove ranges:
const int CONTROL_MAX[5] = {2400, 1000, 2950, 2900, 2850};
const int CONTROL_MIN[5] = {1000, 0, 1970, 1600, 1750};

// Response Glove ranges:
const int RESPONSE_MAX[5] = {2800, 1000, 3100, 2600, 3000};
const int RESPONSE_MIN[5] = {1700, 0, 2150, 1300, 2100};

const int READING_OFFSETS[5] = {430, 0, 200, -320, 210};
const int SERVO_PIN[5] = {22, 23, 19, 18, 21};
const float SERVO_DIRECTIONS[5] = {1, 1, -1, -1, -1};
const int PWM_FREQ = 50;
const int PWM_RES = 16;        // 16-bit resolution

// Prevent servos from going past straight, causing the finger to bend
float servo_cur_positions[5] = {0, 0, 0, 0, 0};
float servo_cur_speeds[5] = {0, 0, 0, 0, 0};
unsigned long prev_servo_update_time = 0;

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
    float response_percentage_bent = (float)(adcValues[i] - RESPONSE_MIN[i]) / (float)(RESPONSE_MAX[i] - RESPONSE_MIN[i]);
    float control_percentage_bent = (float)(target_flex_values[i] - CONTROL_MIN[i]) / (float)(CONTROL_MAX[i] - CONTROL_MIN[i]);

    float dif = (control_percentage_bent - response_percentage_bent) * SERVO_DIRECTIONS[i];
    // Ensure servos don't go past straight
    servo_cur_speeds[i] = (dif > 0 || servo_cur_positions[i] >= 0) ? (dif * SERVO_SPEED_COEFFICIENT) : 0;

    int us = max(MIN_SERVO_SPEED, min(MAX_SERVO_SPEED, MIN_SERVO_SPEED + (MAX_SERVO_SPEED - MIN_SERVO_SPEED) / 2 + (int)(servo_cur_speeds[i])));
    servoWriteMicroseconds(SERVO_PIN[i], us);
  }
}

void detect_servo_positions(){
  unsigned long now = micros();
  float dt = (now - prev_servo_update_time) * 1e-6f;
  prev_servo_update_time = now;

  for (int i = 0; i < 5; ++i){
    servo_cur_positions[i] += servo_cur_speeds[i] * dt;
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
  detect_servo_positions();

  // servoWriteMicroseconds(SERVO_PIN[0], 1000 + 1000 * (adcValues[0] - 1200) / 1200);

  delay(100);
}

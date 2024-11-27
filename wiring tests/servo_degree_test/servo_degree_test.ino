#include <Arduino.h>
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>                                  
#include <Wire.h>
#include <SPI.h>

// Definitions
#define ESPNOW_WIFI_IFACE WIFI_IF_STA                 
#define ESPNOW_WIFI_CHANNEL 4                         

// control data packet structure
typedef struct {
  int dir;                                            // drive direction: 1 = forward, -1 = reverse, 0 = stop
  uint32_t time;                                      // time packet sent
  int driveSpeed;                                     // variable for receiving motor speed
  int hopper;                                         // variable for hopper gate value
  bool left;                                          // variable for left button, either on or off
  bool right;                                         // variable for right button, either on or off
  int collectorStart;
} __attribute__((packed)) esp_now_control_data_t;

// drive data packet structure
typedef struct {
  uint32_t time;                                      // time packet received
  int collectorSpeed;                                 // pwm speed of water speed
  int spinDir;                                        // direction of sorting spin (0 = baseline, 1 = good, 2 = bad)
  int pwmL;
  int pwmR;
} __attribute__((packed)) esp_now_drive_data_t;

// encoder structure
struct Encoder {
  const int chanA;                                    
  const int chanB;                                    
  int32_t pos;                                        
};

// function declarations
void setMotor(int dir, int pwm, int in1, int in2);
void failReboot();
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// communication
uint8_t receiverMacAddress[] = {0xA8,0x42,0xE3,0xCA,0xF1,0xBC};  // MAC address of controller 00:01:02:03:04:05
esp_now_control_data_t inData;                        // control data packet from controller
esp_now_drive_data_t driveData;                       // data packet to send to controller

// added content
const int sorterPin = 5;                         // pin for sorter servo
uint16_t commsLossCount = 0;                          // number of sequential sent packets have dropped 
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop
const int cMinDutyCycle = 1700;                  // duty cycle for 0 degrees
const int cMaxDutyCycle = 8300;                  // duty cycle for 180 degrees  

class ESP_NOW_Network_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Network_Peer(const uint8_t *mac_addr, const uint8_t *lmk = NULL)
    : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IFACE, lmk) {}
    ~ESP_NOW_Network_Peer() {}
  bool begin() {
    if (!add()) {
      log_e("Failed to initialize ESP-NOW or register the peer");
      return false;
    }
      return true;
    }
  bool send_message(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
      log_e("Data to be sent is NULL or has a length of 0");
      return false;
    }
      return send(data, len);
    }
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == 0)                                       // if empty packet
    {
      return;                                           // return
    }
    memcpy(&inData, data, sizeof(inData));              // store drive data from controller
    #ifdef PRINT_INCOMING
      Serial.printf("time: %d, f/r: %d, driveSpeed: %d, hopper: %d, left: %d, right: %d, collector on/off: %d\n", inData.time, inData.dir, inData.driveSpeed, inData.hopper, inData.left, inData.right, inData.collectorStart);
    #endif
    }
  void onSent(bool success) {
    if (success) {
    #ifdef PRINT_SEND_STATUS
        log_i("Unicast message reported as sent %s to peer " MACSTR, success ? "successfully" : "unsuccessfully", MAC2STR(addr()));
        Serial.printf("time: %d, spin dir: %d, collector speed: %d\n", driveData.time, driveData.spinDir, driveData.collectorSpeed);
    #endif
      commsLossCount = 0;
    }
    else {
      commsLossCount++;
    }
    }
};

ESP_NOW_Network_Peer *peer;

void setup() {
  Serial.begin(115200);                               
  while (!Serial) {                                   
    delay(10);                                        
    }
  // esp wifi setup
  WiFi.mode(WIFI_STA);                                
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);               
  while (!WiFi.STA.started()) {                       
    delay(100);                                       
   }
  Serial.print("MAC address for drive "); 
  Serial.println(WiFi.macAddress());                  
  if (!ESP_NOW.begin()) {
    Serial.printf("Failed to initialize ESP-NOW\n");
    failReboot();
   }
  peer = new ESP_NOW_Network_Peer(receiverMacAddress);
  if (peer == nullptr || !peer->begin()) {
    Serial.printf("Failed to create or register the controller peer\n");
    failReboot();
    }
    else {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n", receiverMacAddress[0], receiverMacAddress[1], 
                                                                 receiverMacAddress[2], receiverMacAddress[3], 
                                                                 receiverMacAddress[4], receiverMacAddress[5]);
   }
  memset(&inData, 0, sizeof(inData));                 
  memset(&driveData, 0, sizeof(driveData));           

  // initialize servo pins
  ledcAttach(sorterPin, 50, 16);                    // setup sorter pin for 50 Hz, 16-bit resolution

  // servo starting at baseline 
  ledcWrite(sorterPin, degreesToDutyCycle(90));
}

void loop() {                              

  // reboot device if communication is lost
  if (commsLossCount > cMaxDroppedPackets) {
    failReboot();
  }                                      

  // hopper gate control
  ledcWrite(sorterPin, degreesToDutyCycle(inData.hopper)); // hopper gate control for depositing, 
  Serial.println(inData.hopper);

}

void failReboot() {
  Serial.printf("Rebooting in 3 seconds...\n");
  delay(3000);
  ESP.restart();
}

long degreesToDutyCycle(int deg) {
  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
  return dutyCycle;
}
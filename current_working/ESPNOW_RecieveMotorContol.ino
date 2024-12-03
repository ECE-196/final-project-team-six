// #include <esp_now.h>
// #include <WiFi.h>

// const unsigned int MTR_HI{13};
// const unsigned int MTR_LO{14};

// int grab;
// int ungrab;
 
// // Define a data structure
// typedef struct struct_message {
//   int grab;
//   int ungrab;
// } struct_message;
 
// // Create a structured object
// struct_message myData;
 
 
// // Callback function executed when data is received
// void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
//   memcpy(&myData, incomingData, sizeof(myData));
//   USBSerial.print("grab Value: ");
//   USBSerial.println(myData.grab);
//   grab = myData.grab;
//   USBSerial.print("ungrab Value: ");
//   USBSerial.println(myData.ungrab);
//   ungrab = myData.ungrab;
// }
 
// void setup() {
//   // Set up Serial Monitor
//   USBSerial.begin(115200);
  
//   WiFi.channel(1);
//   // Set ESP32 as a Wi-Fi Station
//   WiFi.mode(WIFI_STA);
 
//   // Initilize ESP-NOW
//   if (esp_now_init() != ESP_OK) {
//     USBSerial.println("Error initializing ESP-NOW");
//     return;
//   }
  
//   // Register callback function
//   esp_now_register_recv_cb(OnDataRecv);

//   pinMode(MTR_HI, OUTPUT);
//   pinMode(MTR_LO, OUTPUT);
// }
 
// void loop() {

// // Map 'grab' value to a PWM range (0-255)
//   int mappedGrab = map(grab, 0, 1023, 0, 255);
//   // Map 'ungrab' value to a PWM range (0-255)
//   int mappedUngrab = map(ungrab, 0, 1023, 0, 255);

//   if (grab > 0) {
//     // Motor spins forward with speed proportional to 'grab'
//     digitalWrite(MTR_HI, 1);  // Set the high motor pin to the mapped 'grab' value
//     digitalWrite(MTR_LO, 0);           // Set the low motor pin to 0 (no reverse)
//   } 
//   else if (ungrab > 0) {
//     // Motor spins in reverse with speed proportional to 'ungrab'
//     digitalWrite(MTR_HI, 0);           // Set the high motor pin to 0 (no forward)
//     digitalWrite(MTR_LO, 1);  // Set the low motor pin to the mapped 'ungrab' value
//   } 
//   else {
//     // Stop the motor when neither grab nor ungrab is active
//     digitalWrite(MTR_HI, 0);
//     digitalWrite(MTR_LO, 0);
//  }
// }

#include <esp_now.h>
#include <WiFi.h>

const unsigned int MTR_HI{13};  // Motor High pin
const unsigned int MTR_LO{14};  // Motor Low pin
unsigned int Lo = 0;
unsigned int Hi = 255;
bool reverse = 0;

int grab;
int ungrab;
 
// Define a data structure
typedef struct struct_message {
  int grab;
  int ungrab;
} struct_message;
 
// Create a structured object
struct_message myData;
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  USBSerial.print("grab Value: ");
  USBSerial.println(myData.grab);
  grab = myData.grab;
  USBSerial.print("ungrab Value: ");
  USBSerial.println(myData.ungrab);
  ungrab = myData.ungrab;
}

void setup() {
  // Set up Serial Monitor
  USBSerial.begin(115200);
  
  WiFi.channel(1);  // Set the Wi-Fi channel
  WiFi.mode(WIFI_STA);  // Set ESP32 as Wi-Fi Station
 
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    USBSerial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize motor pins
  pinMode(MTR_HI, OUTPUT);
  pinMode(MTR_LO, OUTPUT);

  // Setup PWM for the motor pins using ledcWrite()
  ledcSetup(0, 5000, 8);  // Channel 0, 5kHz frequency, 8-bit resolution (0-255 range)
  ledcSetup(1, 5000, 8);  // Channel 1, 5kHz frequency, 8-bit resolution (0-255 range)
  
  ledcAttachPin(MTR_HI, 0);  // Attach motor high pin to channel 0
  ledcAttachPin(MTR_LO, 1);  // Attach motor low pin to channel 1
}

void loop() {
  // Map 'grab' and 'ungrab' values to PWM range (0-255)
  int mappedGrab = map(grab, 0, 1023, 0, 255);
  int mappedUngrab = map(ungrab, 0, 1023, 0, 255);

  if (grab > 0) {
    // Motor spins forward with speed proportional to 'grab'
    ledcWrite(0, mappedGrab);  // Set the high motor pin to the mapped 'grab' value
    USBSerial.println(mappedGrab);
    ledcWrite(1, 0);           // Set the low motor pin to 0 (no reverse)
  } 
  else if (ungrab > 0) {
    // Motor spins in reverse with speed proportional to 'ungrab'
    ledcWrite(0, 0);           // Set the high motor pin to 0 (no forward)
    ledcWrite(1, mappedUngrab);  // Set the low motor pin to the mapped 'ungrab' value
  } 
  else {
    // Stop the motor when neither grab nor ungrab is active
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
  delay(100);
}

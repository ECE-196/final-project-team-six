#include <esp_now.h>
#include <WiFi.h>
 
// Define a data structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;
 
// Create a structured object
struct_message myData;
 
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  USBSerial.print("Data received: ");
  USBSerial.println(len);
  USBSerial.print("Character Value: ");
  USBSerial.println(myData.a);
  USBSerial.print("Integer Value: ");
  USBSerial.println(myData.b);
  USBSerial.print("Float Value: ");
  USBSerial.println(myData.c);
  USBSerial.print("Boolean Value: ");
  USBSerial.println(myData.d);
  USBSerial.println();
}
 
void setup() {
  // Set up Serial Monitor
  USBSerial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    USBSerial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
 
}

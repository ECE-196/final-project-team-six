#include <esp_now.h>
#include <WiFi.h>
#include <Bluepad32.h>

// Define stepper motor pins
const int enable = 7;

const int stepPin = 5;       // Step signal pin
const int dirPin = 6;        // Direction control pin

const int stepPin1 = 3;       // Step signal pin
const int dirPin1 = 4;        // Direction control pin

const int stepPin2 = 1;       // Step signal pin
const int dirPin2 = 2;        // Direction control pin

int isCW = 0;
int isCW1 = 0;
int isCW2 = 0;

// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x48, 0xCA, 0x43, 0x5F, 0xAF, 0x70};
 
// Define a data structure
typedef struct struct_message {
  int grab;
  int ungrab;
} struct_message;
 
// Create a structured object
struct_message myData;
 
// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // USBSerial.print("\r\nLast Packet Send Status:\t");
  // USBSerial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called when a new gamepad is connected.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            USBSerial.printf("CALLBACK: Controller connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            USBSerial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", 
                              ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        USBSerial.println("CALLBACK: Controller connected, but could not find empty slot.");
    }
}

// This callback gets called when a gamepad is disconnected.
void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            USBSerial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    if (!foundController) {
        USBSerial.println("CALLBACK: Controller disconnected, but not found in myControllers.");
    }
}

// Dump gamepad data to serial
void dumpGamepad(ControllerPtr ctl) {
    USBSerial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),
        ctl->dpad(),
        ctl->buttons(),
        ctl->axisX(),
        ctl->axisY(),
        ctl->axisRX(),
        ctl->axisRY(),
        ctl->brake(),
        ctl->throttle(),
        ctl->miscButtons(),
        ctl->gyroX(),
        ctl->gyroY(),
        ctl->gyroZ(),
        ctl->accelX(),
        ctl->accelY(),
        ctl->accelZ()
    );
}

// Process gamepad input and control LED
void processGamepad(ControllerPtr ctl) {
  if(ctl->throttle() == 0 || ctl->brake() == 0){
    myData.grab = ctl->throttle();
    myData.ungrab = ctl->brake();
  }
  else{
    myData.grab = 0;
    myData.ungrab = 0;
  }

  if (ctl->axisX() >= 75) {
    isCW = 1;
  } else if (ctl->axisX() <=  -75) {
    isCW = 2;
  } else {
    isCW = 0;
  }

  if (ctl->axisY() >= 75) {
    isCW1 = 1;
  } else if (ctl->axisY() <=  -75) {
    isCW1 = 2;
  } else {
    isCW1 = 0;
  }

  if (ctl->axisRY() >= 75) {
    isCW2 = 1;
  } else if (ctl->axisRY() <=  -75) {
    isCW2 = 2;
  } else {
    isCW2 = 0;
  }
}

// Process all connected controllers
void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                USBSerial.println("Unsupported controller");
            }
        }
    }
}
 
void setup() {
  pinMode(enable, OUTPUT);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  // Initialize pins to default state
  digitalWrite(enable, LOW);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);

  digitalWrite(stepPin1, LOW);
  digitalWrite(dirPin1, LOW);

  digitalWrite(stepPin2, LOW);
  digitalWrite(dirPin2, LOW);

  // Set up Serial Monitor
  USBSerial.begin(115200);

  USBSerial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  USBSerial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // Optionally forget previous Bluetooth keys
  BP32.enableVirtualDevice(false);  // Disable virtual mouse/touchpad support
 
  WiFi.channel(1);
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // USBSerial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    // USBSerial.println("Failed to add peer, reattempting...");
    esp_now_deinit();  // De-initialize ESP-NOW
    esp_now_init();    // Reinitialize ESP-NOW
    esp_now_add_peer(&peerInfo);  // Re-add peer
}
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
      processControllers();
  }
  while(isCW){
    if (isCW == 1) {
      digitalWrite(dirPin, HIGH);  // Clockwise (CW)
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(5000);  // Half-period HIGH
      digitalWrite(stepPin, LOW);
      delayMicroseconds(5000);  // Half-period LOW
    } else if (isCW == 2) {
      digitalWrite(dirPin, LOW);   // Counter-clockwise (CCW)
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(5000);  // Half-period HIGH
      digitalWrite(stepPin, LOW);
      delayMicroseconds(5000);  // Half-period LOW
    } else if(isCW == 0){
      digitalWrite(stepPin, LOW);
      digitalWrite(stepPin, LOW);
      break;
    }
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
      processControllers();
    }
  }
  if(isCW == 0){
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin, LOW);
  }

  while(isCW1){
    if (isCW1 == 1) {
      digitalWrite(dirPin1, HIGH);  // Clockwise (CW)
      digitalWrite(stepPin1, HIGH);
      delayMicroseconds(5000);  // Half-period HIGH
      digitalWrite(stepPin1, LOW);
      delayMicroseconds(5000);  // Half-period LOW
    } else if (isCW1 == 2) {
      digitalWrite(dirPin1, LOW);   // Counter-clockwise (CCW)
      digitalWrite(stepPin1, HIGH);
      delayMicroseconds(5000);  // Half-period HIGH
      digitalWrite(stepPin1, LOW);
      delayMicroseconds(5000);  // Half-period LOW
    } else if(isCW1 == 0){
      digitalWrite(stepPin1, LOW);
      digitalWrite(stepPin1, LOW);
      break;
    }
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
      processControllers();
    }
  }
  if(isCW1 == 0){
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin1, LOW);
  }

  while(isCW2){
    if (isCW2 == 1) {
      digitalWrite(dirPin2, HIGH);  // Clockwise (CW)
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(5000);  // Half-period HIGH
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(5000);  // Half-period LOW
    } else if (isCW2 == 2) {
      digitalWrite(dirPin2, LOW);   // Counter-clockwise (CCW)
      digitalWrite(stepPin2, HIGH);
      delayMicroseconds(5000);  // Half-period HIGH
      digitalWrite(stepPin2, LOW);
      delayMicroseconds(5000);  // Half-period LOW
    } else if(isCW2 == 0){
      digitalWrite(stepPin2, LOW);
      digitalWrite(stepPin2, LOW);
      break;
    }
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
      processControllers();
    }
  }
  if(isCW2 == 0){
    digitalWrite(stepPin2, LOW);
    digitalWrite(stepPin2, LOW);
  }

  delay(50);
          // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData)); 
  if (result == ESP_OK) {
    // USBSerial.println("Sending confirmed");
  }
  else {
    // USBSerial.println("Sending error");
  }
  delay(50);  // Yield to lower priority tasks to avoid watchdog timeout
}

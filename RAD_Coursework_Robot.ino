#define HAS_SONAR 1
//PWM properties
const int FREQ = 5000;
const int RES = 8;
const int CH_PWM_L = 0;
const int CH_PWM_R = 1;
const int PIN_PWM_LE = 33; //left side motors speed
const int PIN_PWM_LF = 14;
const int PIN_PWM_LB = 27;
const int PIN_PWM_RF = 26;
const int PIN_PWM_RB = 25;
const int PIN_PWM_RE = 12; //right side motors speed
const int PIN_TRIGGER = 32;
const int PIN_ECHO = 35;

//Bluetooth
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *bleServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
const char *SERVICE_UUID = "61653dc3-4021-4d1e-ba83-8b4eec61d613";  // UART service UUID
const char *CHARACTERISTIC_UUID_RX = "06386c14-86ea-4d71-811c-48f97c58f8c9";
const char *CHARACTERISTIC_UUID_TX = "9bf1103b-834c-47cf-b149-c9e4bcf778a7";

enum msgParts {
  HEADER,
  BODY
};

msgParts msgPart = HEADER;  // declaration of the enum msg
char header;
char endChar = '\n';
const char MAX_MSG_SZ = 60;
char msg_buf[MAX_MSG_SZ] = "";
int msg_idx = 0;

void on_ble_rx(char inChar) {
  if (inChar != endChar) {
    switch (msgPart) {
      case HEADER:
        process_header(inChar);
        return;
      case BODY:
        process_body(inChar);
        return;
    }
  } else {
    msg_buf[msg_idx] = '\0';  // end of message
    parse_msg();
  }
}

//Initialization of classes for bluetooth
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *bleServer, esp_ble_gatts_cb_param_t *param) {
    deviceConnected = true;
    Serial.println("BT Connected");
  };

  void onDisconnect(BLEServer *bleServer) {
    deviceConnected = false;
    Serial.println("BT Disconnected");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string bleReceiver = pCharacteristic->getValue();
    if (bleReceiver.length() > 0) {
      for (int i = 0; i < bleReceiver.length(); i++) {
        on_ble_rx(bleReceiver[i]);
      }
    }
  }
};


//------------------------------------------------------//
// INITIALIZATION
//------------------------------------------------------//

#if (HAS_SONAR)
// Sonar sensor
const float US_TO_CM = 0.01715;               //cm/uS -> (343 * 100 / 1000000) / 2;
const unsigned int MAX_SONAR_DISTANCE = 300;  //cm
const unsigned long MAX_SONAR_TIME = (long)MAX_SONAR_DISTANCE * 2 * 10 / 343 + 1;
const unsigned int STOP_DISTANCE = 50;  //cm
unsigned long sonar_interval = 1000;
unsigned long sonar_time = 0;
boolean sonar_sent = false;
boolean ping_success = false;
unsigned int distance = -1;           //cm
unsigned int distance_estimate = -1;  //cm
unsigned long start_time;
unsigned long echo_time = 0;
#else
const unsigned int TURN_DISTANCE = -1;  //cm
const unsigned int STOP_DISTANCE = 0;   //cm
unsigned int distance_estimate = -1;    //cm
#endif

//Vehicle Control
int ctrl_left = 0;
int ctrl_right = 0;

//Heartbeat
unsigned long heartbeat_interval = -1;
unsigned long heartbeat_time = 0;

//------------------------------------------------------//
// SETUP
//------------------------------------------------------//
void setup() {
// Initialize with the I2C addr 0x3C
#if (HAS_SONAR)
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIGGER, OUTPUT);
#endif

  // PWMs
  // Configure PWM functionalitites
  ledcSetup(CH_PWM_L, FREQ, RES);
  ledcSetup(CH_PWM_R, FREQ, RES);

  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(PIN_PWM_LE, CH_PWM_L);
  pinMode(PIN_PWM_LF, OUTPUT);
  pinMode(PIN_PWM_LB, OUTPUT);
  pinMode(PIN_PWM_RF, OUTPUT);
  pinMode(PIN_PWM_RB, OUTPUT);
  ledcAttachPin(PIN_PWM_RE, CH_PWM_R);


  Serial.begin(115200, SERIAL_8N1);
  Serial.println('r');

  //bluetooth
  String ble_name = "Adithya's ESP32";
  BLEDevice::init(ble_name.c_str());
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = bleServer->createService(BLEUUID(SERVICE_UUID));

  pTxCharacteristic = pService->createCharacteristic(BLEUUID(CHARACTERISTIC_UUID_TX), BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(BLEUUID(CHARACTERISTIC_UUID_RX), BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pRxCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID(SERVICE_UUID));
  bleServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

//------------------------------------------------------//
//LOOP
//------------------------------------------------------//
void loop() {
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                     // give the bluetooth stack the chance to get things ready
    bleServer->startAdvertising();  // restart advertising
    Serial.println("Waiting a client connection to notify...");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  if (Serial.available() > 0) {
    on_serial_rx();
  }
  if (distance_estimate <= STOP_DISTANCE && ctrl_left > 0 && ctrl_right > 0) {
    ctrl_left = 0;
    ctrl_right = 0;
  }
  if ((millis() - heartbeat_time) >= heartbeat_interval) {
    ctrl_left = 0;
    ctrl_right = 0;
  }

  update_vehicle();

#if HAS_SONAR
  // Check for successful sonar reading
  if (!sonar_sent && ping_success) {
    distance = echo_time * US_TO_CM;
    update_distance_estimate();
    send_sonar_reading();
    sonar_sent = true;
  }
  // Measure distance every sonar_interval
  if ((millis() - sonar_time) >= max(sonar_interval, MAX_SONAR_TIME)) {
    if (!sonar_sent && !ping_success) {  // Send max val if last ping was not returned
      distance = MAX_SONAR_DISTANCE;
      update_distance_estimate();
      send_sonar_reading();
      sonar_sent = true;
    }
    sonar_time = millis();
    sonar_sent = false;
    send_ping();
  }
#endif
}

//------------------------------------------------------//
// FUNCTIONS
//------------------------------------------------------//

void update_vehicle() {
  update_left_motors();
  update_right_motors();
  //update_servo();
  //update_indicators();
}

void update_left_motors() {
  if (ctrl_left > 0) {
    ledcWrite(CH_PWM_L, ctrl_left);
    digitalWrite(PIN_PWM_LF, HIGH);
    digitalWrite(PIN_PWM_LB, LOW);
  } else if (ctrl_left < 0) {
    ledcWrite(CH_PWM_L, -ctrl_left);
    digitalWrite(PIN_PWM_LF, LOW);
    digitalWrite(PIN_PWM_LB, HIGH);
  } else {
    ledcWrite(CH_PWM_L, ctrl_left);
    digitalWrite(PIN_PWM_LF, LOW);
    digitalWrite(PIN_PWM_LB, LOW);
  }
}

void update_right_motors() {
  if (ctrl_right > 0) {
    ledcWrite(CH_PWM_R, ctrl_right);
    digitalWrite(PIN_PWM_RF, HIGH);
    digitalWrite(PIN_PWM_RB, LOW);
  } else if (ctrl_right < 0) {
    ledcWrite(CH_PWM_R, -ctrl_right);
    digitalWrite(PIN_PWM_RF, LOW);
    digitalWrite(PIN_PWM_RB, HIGH);
  } else {
    ledcWrite(CH_PWM_R, ctrl_right);
    digitalWrite(PIN_PWM_RF, LOW);
    digitalWrite(PIN_PWM_RB, LOW);
  }

  //void update_servo() {
  //
  //}

  //void update_indicators(){
  //  
  //}
}

void process_ctrl_msg() {
  char *tmp;                    // this is used by strtok() as an index
  tmp = strtok(msg_buf, ",:");  // replace delimiter with \0
  ctrl_left = atoi(tmp);        // convert to int
  tmp = strtok(NULL, ",:");     // continues where the previous call left off
  ctrl_right = atoi(tmp);       // convert to int
  Serial.print("Control: ");
  Serial.print(ctrl_left);
  Serial.print(",");
  Serial.println(ctrl_right);
}

void process_heartbeat_msg() {
  heartbeat_interval = atol(msg_buf);  // convert to long
  heartbeat_time = millis();
  Serial.print("Heartbeat Interval: ");
  Serial.println(heartbeat_interval);
}

#if HAS_SONAR
void process_sonar_msg() {
  sonar_interval = atol(msg_buf);  // convert to long
}
#endif

void process_feature_msg() {
  String msg = "fDIY_ESP32:";
#if HAS_SONAR
  msg += "s:";
#endif
  sendData(msg);
}

void on_serial_rx() {
  char inChar = Serial.read();
  if (inChar != endChar) {
    switch (msgPart) {
      case HEADER:
        process_header(inChar);
        return;
      case BODY:
        process_body(inChar);
        return;
    }
  } else {
    msg_buf[msg_idx] = '\0';  // end of message
    parse_msg();
  }
}

void process_header(char inChar) {
  header = inChar;
  msgPart = BODY;
}

void process_body(char inChar) {
  // Add the incoming byte to the buffer
  msg_buf[msg_idx] = inChar;
  msg_idx++;
}

void parse_msg() {
  switch (header) {
    case 'c':
      process_ctrl_msg();
      break;
    case 'f':
      process_feature_msg();
      break;
    case 'h':
      process_heartbeat_msg();
      break;
#if HAS_SONAR
    case 's':
      process_sonar_msg();
      break;
#endif

  }
  msg_idx = 0;
  msgPart = HEADER;
  header = '\0';
}

#if HAS_SONAR
void send_sonar_reading() {
  sendData("s" + String(distance_estimate));
}

// Send pulse by toggling trigger pin
void send_ping() {
  echo_time = 0;
  ping_success = false;
  if (PIN_TRIGGER == PIN_ECHO)
    pinMode(PIN_TRIGGER, OUTPUT);
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
  if (PIN_TRIGGER == PIN_ECHO)
    pinMode(PIN_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), start_timer, RISING);
}

void update_distance_estimate() {
  distance_estimate = distance;
}
#endif

void sendData(String data) {
Serial.print(data);
Serial.println();
  if (deviceConnected) {
    char outData[MAX_MSG_SZ] = "";
    for (int i = 0; i < data.length(); i++) {
      outData[i] = data[i];
    }
    pTxCharacteristic->setValue(outData);
    pTxCharacteristic->notify();
  }
}

//------------------------------------------------------//
// INTERRUPT SERVICE ROUTINES (ISR)
//------------------------------------------------------//
#if HAS_SONAR
// ISR: Start timer to measure the time it takes for the pulse to return
void start_timer() {
  start_time = micros();
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), stop_timer, FALLING);
}

// ISR: Stop timer and record the time
void stop_timer() {
  echo_time = micros() - start_time;
  detachInterrupt(digitalPinToInterrupt(PIN_ECHO));
  ping_success = true;
}
#endif
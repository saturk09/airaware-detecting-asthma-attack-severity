//Libraries to include: BLEDevice, BLEServer, BLEUtils, BLE2902 (searched up after the device wasn't working)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


#define EMG_PIN 2


//3 Lines told to Include by Example Code
BLEServer* pServer = NULL;
BLECharacteristic* emgCharacteristic = NULL;
bool deviceConnected = false;


//Lines retrieved after scanning for BLE peripherals
#define SERVICE_UUID "3cb18870-19c5-4292-b13d-442e2af8447a"
#define CHARACTERISTIC_UUID "e709a157-3ee8-46e7-8a91-bb0489c17ae8"


//MyServerCallBacks tracks connection status, uses this to help print the status
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) { deviceConnected = false; pServer->startAdvertising(); }
};


//readEMG method: returns the value of the Seeed Studio EMG as directed by the sample code
int readEMG() {
  long sum = 0;
  for (int i = 0; i < 32; i++) sum += analogRead(EMG_PIN);
  return sum / 32;
}


//EMG Setup
void setup() {
  pinMode(EMG_PIN, INPUT);
  Serial.begin(115200); //Starts Serial Monitor at 115200 bits per second


  BLEDevice::init("Chest XIAO"); //initializes the XIAO as a BLEDevice
  pServer = BLEDevice::createServer(); //Creates a server for the XIAO
  pServer->setCallbacks(new MyServerCallbacks()); //Creates callback for the XIAO
  BLEService* service = pServer->createService(SERVICE_UUID); //Creates service for the XIAO


  emgCharacteristic = service->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  ); //creates a specific characteristic for BLE that client devices can identify, allowing to request and read the value (PROPERTY_READ) and enable push updates (PROPERTY_NOTIFY)
  emgCharacteristic->addDescriptor(new BLE2902()); //(suggested to input after searching up): adds a configuration descriptor
  service->start(); //Starts the BLE Service


  //Begin advertising of the BLE Device; researched what lines of BLE connecting code were necessary
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  Serial.println("Advertising...");
}


//EMG Loop
void loop() {
  if (deviceConnected) { //if the device is connected to the Arduino NANO
    int data = readEMG(); //read information from the EMG
    Serial.print("EMG: "); Serial.println(data); //Print EMG data
     uint8_t buf[2]; //extra information for EMG delay and properly timed readings (avoiding duplicate readings)
    buf[0] = data & 0xFF; //extra information for EMG delay and properly timed readings (avoiding duplicate readings)
    buf[1] = (data >> 8) & 0xFF; //extra information for EMG delay and properly timed readings (avoiding duplicate readings)
    emgCharacteristic->setValue(buf, 2); //extra information for EMG delay and properly timed readings (avoiding duplicate readings)
    emgCharacteristic->notify(); //extra information for EMG delay and properly timed readings (avoiding duplicate readings)
    delay(500); //Delay 0.5 seconds
  }
}


/* 
 * Heartrate + Battery
 */

#include "BLEDevice.h"
//#include "BLEScan.h"

// The remote service we wish to connect to.
static BLEUUID serviceHR_UUID("0000180d-0000-1000-8000-00805f9b34fb");
static BLEUUID serviceBAT_UUID("0000180F-0000-1000-8000-00805f9b34fb");

// The characteristic of the remote service we are interested in.
static BLEUUID    charHR_UUID("00002A37-0000-1000-8000-00805f9b34fb");
static BLEUUID    charBAT_UUID("00002A19-0000-1000-8000-00805f9b34fb");


//BLE Shizzle
static boolean doConnect = false;
static boolean HRconnected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myHRDevice;


//Our Heartrate & Battery level
int HR = 0;
int BAT = 0;




static void notifyHRCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    HR = pData[1];
    Serial.print("Heart Rate @ ");
    Serial.print(HR, DEC);
    Serial.println("bpm");
   }

static void notifyBATCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    BAT = pData[0];
    Serial.print("Battery level @ ");
    Serial.print(BAT);
    Serial.println("%");
   }

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    HRconnected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    HRconnected = false;
    Serial.println("onDisconnect");
    doScan = true;
  }
};

bool connectTo_HR_BAT_Server() {
    Serial.print("Forming a connection to ");
    Serial.println(myHRDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");
    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myHRDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceHR_UUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceHR_UUID.toString().c_str());
      pClient->disconnect();
      return false;

    }
    Serial.println(" - Found our HR service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charHR_UUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic HR UUID: ");
      Serial.println(charHR_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our HR characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The HR characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyHRCallback);
      Serial.println("Notify HR on");
      
    HRconnected = true;

// Obtain a reference to the service we are after in the remote BLE server.
    //BLERemoteService* 
    pRemoteService = pClient->getService(serviceBAT_UUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service BAT UUID: ");
      Serial.println(serviceHR_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our BAT service");

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charBAT_UUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charBAT_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our BAT characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      BAT = (uint8_t)value[0];
      Serial.print("The BAT characteristic value was: ");
      Serial.println((uint8_t)value[0]);
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyBATCallback);
      Serial.println("Notify BAT on");
      
    HRconnected = true;

    
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceHR_UUID)) {

      BLEDevice::getScan()->stop();
      myHRDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      //doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(60, false);
} // End of setup.


// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectTo_HR_BAT_Server()) {
      Serial.println("We are now connected to the BLE HR_BAT Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (HRconnected) {
    //String newValue = "Time since boot: " + String(millis()/1000);
    //Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    //pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1000); // Delay a second between loops.
} // End of loop

/* 
 * Heartrate + Battery
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include "BLEDevice.h"
#include "SSD1306.h"

///GPIO config/////
const int R1 = 13;
const int R2 = 14;
const int R3 = 15;

//Wifi Settings/////////////////////////
const char* ssid = "SSID";
const char* password =  "PASS";

//MQTT Settings/////////////////////////
const char* mqttServer = "Server_IP";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const String mqttTopic = "Zwift";
const String mqttSubTopicHR = "hr";
const String mqttSubTopicBAT = "bat";
const String mqttSubTopicPWR = "pwr";
const String mqttSubTopicCAD = "cad";


// The remote service we wish to connect to/////////////////////////////////
static BLEUUID serviceHR_UUID("0000180d-0000-1000-8000-00805f9b34fb");
static BLEUUID serviceBAT_UUID("0000180F-0000-1000-8000-00805f9b34fb");
static BLEUUID servicePWR_UUID("00001818-0000-1000-8000-00805f9b34fb");

// The characteristic of the remote service we are interested in////////////////
static BLEUUID    charHR_UUID("00002A37-0000-1000-8000-00805f9b34fb");
static BLEUUID    charBAT_UUID("00002A19-0000-1000-8000-00805f9b34fb");
static BLEUUID    charPWR_UUID("00002A63-0000-1000-8000-00805f9b34fb");

//BLE Shizzle///////////////////////////////////////////////////////////
static boolean HRdoConnect = false;
static boolean PWRdoConnect = false;
static boolean HRconnected = false;
static boolean PWRconnected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myHRDevice;
static BLEAdvertisedDevice* myPWRDevice;

// Oled Display
SSD1306  display(0x3c, 5, 4);
unsigned long screen_update, stats_update;

//Our Heartrate & Battery level/////////////////////////////////////////
int HR = 0;
int BAT = 0;

//Variables for Crank Event Time
float NewCrankEvent;
float DiffCrankEvent;
float OldCrankEvent;
float CrankEventTime;

//Variables for Crank Revolutions
float NewCrankRev;
float OldCrankRev;
float DiffCrankRev;

//Our Cadence
int Cadence = 0;

//Our Power
int Power = 0;

//Fanspeed
String FS;

//Weight
float Weight = 75;
float Ratio = 0;

// Draw OLED
void drawOLED(){
      Ratio = float(Power) / Weight;
      display.clear();
      display.setColor(WHITE);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString( 0, 0, String(HR) + " bpm");
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 24, String(Power) + " W");
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 46, String(Cadence) + " Rmp");
      display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(128, 24, String(Ratio) + " W/kg");
      display.setFont(ArialMT_Plain_10);
      display.drawString(128, 0, String(BAT) + "% Battery");
      display.setFont(ArialMT_Plain_10);
      display.drawString(128, 52, "Fanspeed: " + String(FS));
      
      display.display();
      
} 

///Set outputs to LOW////////////////////
void InitializeOutputs(){
  Serial.println("Initialize OUtputs"); 
  digitalWrite(R1, HIGH);
  digitalWrite(R2, HIGH);
  digitalWrite(R3, HIGH);
  FS = "0";
  States();
}

///Show output states/////////////////////////
void States() {
    Serial.print("State of R1: ");
    Serial.println(digitalRead(R1));
    Serial.print("State of R2: ");
    Serial.println(digitalRead(R2));
    Serial.print("State of R3: ");
    Serial.println(digitalRead(R3));
}

//MQTT Callback for Fanspeed/////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String Fanspeed;
  
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    Fanspeed += (char)payload[i];
  }
  FS = Fanspeed;
  Serial.println();

  if (Fanspeed == "0"){
    Serial.println("Fan off!");
    InitializeOutputs();
    States();
    
  }

  if (Fanspeed == "1"){
    Serial.println("Fanspeed 1!");
    digitalWrite(R1, LOW);
    digitalWrite(R2, HIGH);
    digitalWrite(R3, HIGH);
    States();
  }
    if (Fanspeed == "2"){
    Serial.println("Fanspeed 2!");
    digitalWrite(R1, HIGH);
    digitalWrite(R2, LOW);
    digitalWrite(R3, HIGH);
    States();
  }
  if (Fanspeed == "3"){
    Serial.println("Fanspeed 3!");
    digitalWrite(R1, HIGH);
    digitalWrite(R2, HIGH);
    digitalWrite(R3, LOW);
    States();
  }
  
}

WiFiClient espClient;
PubSubClient client(espClient);

//Notify Callbacks//////////////////////////////////////////////
static void notifyHRCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    HR = pData[1];
//    Serial.print("Heart Rate @ ");
//    Serial.print(HR, DEC);
//    Serial.println("bpm");
   }

static void notifyBATCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    BAT = pData[0];
//    Serial.print("Battery level @ ");
//    Serial.print(BAT);
//    Serial.println("%");
   }

static void notifyPWRCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

//   Serial.print("Data length of pData: ");
//   Serial.println(length);
    Power = pData[2] + pData[3]*256;
    NewCrankEvent = pData[13] + pData[14]*256; // Crank Event Time in 1/1024 seconds
    DiffCrankEvent = NewCrankEvent - OldCrankEvent; // Difference between new & old in 1/1024 seconds
    CrankEventTime = float(DiffCrankEvent) / 1024; // in seconds

    NewCrankRev = pData[11] + pData[12]*256; //Crank Revolutions    
    DiffCrankRev = NewCrankRev - OldCrankRev; //Difference between new % old

 //Prevent deviding by 0
    if (CrankEventTime > 0) {
      Cadence = DiffCrankRev / CrankEventTime * 60; //Cadence in rpm
    }
    else {
      Cadence = 0;
    }
//    Serial.println(String(pData[0]));
//    Serial.println("Old crank event timestamp: " +String(OldCrankEvent));
//    Serial.println("New crank event timestamp: " +String(NewCrankEvent));
//    Serial.println("Diff crank event timestamp: " +String(DiffCrankEvent));
//    Serial.println("Crank event time in seconds: " +String(CrankEventTime));
//    Serial.println("Old crank Rev: " +String(OldCrankRev));
//    Serial.println("New crank Rev: " +String(NewCrankRev));
//    Serial.println("Diff crank Rev: " +String(DiffCrankRev));
//    Serial.println("Cadence: " + String(Cadence) + " Rpm");
//    Serial.println("Power " + String(Power) + " Watt");
    
    OldCrankEvent = NewCrankEvent;
    OldCrankRev = NewCrankRev;

}

//Client Callbacks///////////////////////////////////////
class MyHRClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    HRconnected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    HRconnected = false;
    Serial.println("onDisconnect");
    doScan = true;
  }
};

class MyPWRClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    PWRconnected = true;
  }

  void onDisconnect(BLEClient* pclient) {
    HRconnected = false;
    Serial.println("onDisconnect");
    doScan = true;
  }
};


//Client connections////////////////////////////////////////

bool connectTo_HR_BAT_Server() {
    Serial.print("Forming a connection to ");
    Serial.println(myHRDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");
    pClient->setClientCallbacks(new MyHRClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myHRDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to HRM server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceHR_UUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our HRM service UUID: ");
      Serial.println(serviceHR_UUID.toString().c_str());
      pClient->disconnect();
      return false;

    }
    Serial.println(" - Found our HRM service");


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

bool connectTo_PWR_Server() {
    Serial.print("Forming a connection to ");
    Serial.println(myPWRDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created PWR client");

    pClient->setClientCallbacks(new MyPWRClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myPWRDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to PWR server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(servicePWR_UUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our PWR service UUID: ");
      Serial.println(servicePWR_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our PWR service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charPWR_UUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our PWR characteristic UUID: ");
      Serial.println(charPWR_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our PWR characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyPWRCallback);

    PWRconnected = true;
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

      //BLEDevice::getScan()->stop();
      myHRDevice = new BLEAdvertisedDevice(advertisedDevice);
      HRdoConnect = true;
      //doScan = true;

    } // Found our HRM server

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(servicePWR_UUID)) {

      //BLEDevice::getScan()->stop();
      myPWRDevice = new BLEAdvertisedDevice(advertisedDevice);
      PWRdoConnect = true;
      //doScan = true;

    } // Found our PWR server

    if (HRdoConnect && PWRdoConnect == true) {
      BLEDevice::getScan()->stop();
    } //Stop scanning
  
  } // onResult


}; // MyAdvertisedDeviceCallbacks


void setup() {

  pinMode (R1, OUTPUT);
  pinMode (R2, OUTPUT);
  pinMode (R3, OUTPUT);
  
  InitializeOutputs();
  
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  
//Start the OLED Display////////////////////////////////////
 
  display.init();
  display.setFont(ArialMT_Plain_24);
  display.flipScreenVertically();                 // this is to flip the screen 180 degrees

  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 10, "BLE/MQTT");
  display.drawString(64, 35, "Gateway");

  display.display();

 //Connect to WIFI////////////////////////////////////// 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  Connect_MQQT(); 
  
  
  Serial.println("Starting BLE Client application...");
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

void Connect_MQQT(){ 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
 
      Serial.print("Connected. Message sent to topic ");
      Serial.print(String(mqttTopic));
      Serial.println("/Connected.");
      client.publish( (mqttTopic + "/Connection").c_str(), "ESP32 Connected" );
      client.subscribe((mqttTopic + "/Fanspeed").c_str());
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  } 
}






// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (HRdoConnect == true) {
    if (connectTo_HR_BAT_Server()) {
      Serial.println("We are now connected to the HRM Server.");
      client.publish( (mqttTopic + "/Connection").c_str(), "HRM/BAT Connected" );
    } else {
      Serial.println("We have failed to connect to the HRM server; there is nothin more we will do.");
      client.publish( (mqttTopic + "/Connection").c_str(), "HRM/BAT connection faillor" );
    }
    HRdoConnect = false;
  }

  if (PWRdoConnect == true) {
    if (connectTo_PWR_Server()) {
      Serial.println("We are now connected to the PWR Server.");
    } else {
      Serial.println("We have failed to connect to the PWR server; there is nothin more we will do.");
    }
    PWRdoConnect = false;
  }
  
 if (HRconnected == true) {

    client.publish( (mqttTopic + "/" + mqttSubTopicHR).c_str(), String(HR).c_str() );
    Serial.print("Heart Rate @ ");
    Serial.print(HR, DEC);
    Serial.println("bpm");    
   
    client.publish( (mqttTopic + "/" + mqttSubTopicBAT).c_str(), String(BAT).c_str() );
    Serial.print("Battery level @ ");
    Serial.print(BAT);
    Serial.println("%");
    
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
  if (PWRconnected == true) {
    
    client.publish( (mqttTopic + "/" + mqttSubTopicPWR).c_str(), String(Power).c_str() );
    client.publish( (mqttTopic + "/" + mqttSubTopicCAD).c_str(), String(Cadence).c_str() );
    
    Serial.println("Cadence: " + String(Cadence) + " Rpm");
    Serial.println("Power " + String(Power) + " Watt");
    
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }

// Every 100 Milliseconds
  if ((millis()-screen_update)>100) { // 100 Milliseconds
    drawOLED();
    screen_update = millis();
  }
  client.loop();

  
/////To reconnect to MQQT after disconnect BLE HRM ////////////
  if (client.loop() == 0) {
    Connect_MQQT();
  }

  
  delay(1000);
} // End of loop

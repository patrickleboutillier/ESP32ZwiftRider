#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>


// fitness machine service uuid, as defined in GATT specifications
#define FITNESS_MACHINE_SERVICE   BLEUUID((uint16_t)0x1826) 

// BLE Characteristics required
BLECharacteristic indoorBike(BLEUUID((uint16_t)0x2AD2), BLECharacteristic::PROPERTY_NOTIFY) ;
BLECharacteristic fitnessMachineControlPoint(BLEUUID((uint16_t)0x2AD9), BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_WRITE);


#define CADENCE     39
#define POWER       36
#define CONNECTED   2

void setup() {
  Serial.begin(115200) ;
  pinMode(POWER, INPUT) ;
  pinMode(CADENCE, INPUT) ;
  pinMode(CONNECTED, OUTPUT) ;
  BLEDevice::init("ESP32ZwiftRider") ; // name of the ble device
  InitBLEServer() ;
}


bool connected = false ;
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Connected") ;
    digitalWrite(CONNECTED, HIGH) ;
    connected = true ;
  } ;
  void onDisconnect(BLEServer* pServer) {
    Serial.println("Disconnected") ;
    digitalWrite(CONNECTED, LOW) ;
    connected = false ;
    BLEDevice::startAdvertising() ;
  } ;
} ;


void InitBLEServer() {
  BLEServer *pServer = BLEDevice::createServer() ;
  pServer->setCallbacks(new MyServerCallbacks()) ;
  BLEService *pFitness = pServer->createService(FITNESS_MACHINE_SERVICE);

  indoorBike.addDescriptor(new BLE2902()) ;
  pFitness->addCharacteristic(&indoorBike) ;

  fitnessMachineControlPoint.addDescriptor(new BLE2902());
  pFitness->addCharacteristic(&fitnessMachineControlPoint);

  pFitness->start() ;

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(FITNESS_MACHINE_SERVICE);
  pAdvertising->setScanResponse(true) ;
  pAdvertising->setMinPreferred(0x06) ;
  BLEDevice::startAdvertising() ;
}


#define MAX_CADENCE 120
#define MAX_POWER   500

int16_t grade = 0 ;
int16_t target_power = -1 ;
int16_t target_cadence = -1 ;
void loop() {
  int cadence = (target_cadence >= 0 ? target_cadence : map(analogRead(CADENCE), 0, 4095, 0, MAX_CADENCE)) ;
  int power = (target_power >= 0 ? target_power : map(analogRead(POWER), 0, 4095, 0, MAX_POWER)) ;
  
  if (connected){
    sendData(power, cadence) ;
    getData() ;
  }
  
  delay(100) ;
}


#define INDOOR_BIKE_DATA_DEF  0b0000000001000100 // flags for indoor bike data characteristics - power and cadence
void sendData(uint16_t power, uint16_t cadence){
  uint8_t indoorBikeData[8] = {
    (uint8_t)(INDOOR_BIKE_DATA_DEF & 0xff),  
    (uint8_t)(INDOOR_BIKE_DATA_DEF >> 8), 
    (uint8_t)0, (uint8_t)0,                                             // Speed
    (uint8_t)((cadence << 1) & 0xff), (uint8_t)((cadence << 1) >> 8),   // Cadence
    (uint8_t)(power & 0xff), (uint8_t)(power >> 8)                      // Power
  } ;
  indoorBike.setValue(indoorBikeData, 8) ;
  indoorBike.notify() ;
}


void getData(){
  std::string msg = fitnessMachineControlPoint.getValue() ;
  if (msg.length() > 0){
    uint8_t resp[3] = {0x80, msg[0], 0x01};
    switch(msg[0]) {
      case 0x80:    // Response
        return ;
      case 0x00:    // Request control
      case 0x01:     // Reset
      case 0x07:    // Start/stop
        break ;
      case 0x05:    // Target power level
        target_power = (msg[2] << 8) + msg[1] ;
        Serial.print("Target Power: ") ;
        Serial.println(target_power) ; 
        break ;
      case 0x11:
        grade = (msg[4] << 8) + msg[3] ;
        Serial.print("Grade: ") ;
        Serial.println(grade) ; 
        break ;
    }
    
    for (int i = 0 ; i < msg.length() ; i++){
      Serial.print(msg[i], HEX) ;
      Serial.print(" ") ;
    }
    Serial.println() ;
    fitnessMachineControlPoint.setValue(resp, 3) ;
    fitnessMachineControlPoint.indicate() ;
  }
}

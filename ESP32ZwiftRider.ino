#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>
#include <BLE2902.h>


// fitness machine service uuid, as defined in gatt specifications
#define FITNESS_MACHINE_SERVICE   BLEUUID((uint16_t)0x1826) 

// BLE Characteristics required
BLECharacteristic indoorBike(BLEUUID((uint16_t)0x2AD2), BLECharacteristic::PROPERTY_NOTIFY) ;
BLECharacteristic fitnessMachineControlPoint(BLEUUID((uint16_t)0x2AD9), BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic fitnessMachineStatus(BLEUUID((uint16_t)0x2ADA), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic fitnessMachineFeature(BLEUUID((uint16_t)0x2ACC), BLECharacteristic::PROPERTY_READ);  


#define CADENCE     39
#define POWER       36
#define CONNECTED   2
#define BUTTON      23

void setup() {
  Serial.begin(115200) ;
  pinMode(POWER, INPUT) ;
  pinMode(CADENCE, INPUT) ;
  pinMode(BUTTON, INPUT) ;
  pinMode(CONNECTED, OUTPUT) ;
  BLEDevice::init("ESP32ZwiftRider") ; // name of the ble device
  Serial.print("Standby...") ;
  while (! digitalRead(BUTTON)){
    delay(50) ;
  }
  Serial.println("ready.") ;
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

  BLE2902* descr = new BLE2902();
  fitnessMachineControlPoint.addDescriptor(descr);
  pFitness->addCharacteristic(&fitnessMachineControlPoint);

  pFitness->addCharacteristic(&fitnessMachineStatus) ;
  fitnessMachineStatus.addDescriptor(new BLE2902());
  
  fitnessMachineFeature.addDescriptor(new BLE2902());
  pFitness->addCharacteristic(&fitnessMachineFeature);

  pFitness->start() ;

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(FITNESS_MACHINE_SERVICE);
  const std::string fitnessData = { 0b00000001, 0b00100000, 0b00000000 } ;
  BLEAdvertisementData advertisementData = BLEAdvertisementData() ;
  advertisementData.setServiceData(FITNESS_MACHINE_SERVICE, fitnessData) ;
  
  pAdvertising->setAdvertisementData(advertisementData) ;
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

  /*
  // Pretend we support all the features so we get all the messages.
  //const uint32_t fitnessMachineFeaturesDef = 0b00000000000000000100000010001010 ; 
  //const uint32_t targetSettingFeaturesDef =  0b00000000000000010010000000001000 ;                        
  const uint32_t fitnessMachineFeaturesDef = 0b00000000000000011111111111111111 ; 
  const uint32_t targetSettingFeaturesDef =  0b00000000000000011111111111111111 ;
  uint8_t fitnessMachineFeatureData[8] = {  // values for setup - little endian order
    (uint8_t)(fitnessMachineFeaturesDef & 0xff),
    (uint8_t)(fitnessMachineFeaturesDef >> 8),
    (uint8_t)(fitnessMachineFeaturesDef >> 16),
    (uint8_t)(fitnessMachineFeaturesDef >> 24),
    (uint8_t)(targetSettingFeaturesDef & 0xff),
    (uint8_t)(targetSettingFeaturesDef >> 8),
    (uint8_t)(targetSettingFeaturesDef >> 16),
    (uint8_t)(targetSettingFeaturesDef >> 24) 
  } ;
  fitnessMachineFeature.setValue(fitnessMachineFeatureData, 8) ; // flags
  fitnessMachineFeature.notify() ;
  */
}


void getData(){
  std::string msg = fitnessMachineControlPoint.getValue() ;
  if (msg.length() > 0){
    uint8_t resp[3] = {0x80, msg[0], 0x01};
    switch(msg[0]) {
      case 0x80:    // Response
        return ;
      case 0x00:    // Request control
      case 0x01: {    // Reset
        uint8_t status[3] = { 0x15, (uint8_t)180, (uint8_t)0 } ;
        fitnessMachineStatus.setValue(status, 3) ;
        fitnessMachineStatus.notify() ;
      }
      case 0x07:    // Start/stop
        break ;
      case 0x05: {  // Target power level
        target_power = (msg[2] << 8) + msg[1] ;
        Serial.print("Target Power: ") ;
        Serial.println(target_power) ; 
        uint8_t status[3] = { 0x08, (uint8_t)msg[1], (uint8_t)msg[2] } ;
        fitnessMachineStatus.setValue(status, 3) ;
        fitnessMachineStatus.notify() ;
        break ;
      }
      case 0x11: {
        grade = (msg[4] << 8) + msg[3] ;
        Serial.print("Grade: ") ;
        Serial.println(grade) ; 
        uint8_t status[7] = { 0x12, (uint8_t)msg[1], (uint8_t)msg[2], (uint8_t)msg[2], (uint8_t)msg[3], (uint8_t)msg[4], (uint8_t)msg[5] } ;
        fitnessMachineStatus.setValue(status, 7) ;
        fitnessMachineStatus.notify() ;
        break ;
      }
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

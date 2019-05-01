// https://webbluetoothcg.github.io/demos/heart-rate-sensor/

// Import libraries (BLEPeripheral depends on SPI)
#include <SPI.h>
#include <BLEPeripheral.h>

#include "nrf.h" // RTC0 check

// define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9
#define BLE_DEVICE_NAME "stefaan-HRM"

bool bleConnected = false;
uint8_t myHeartRate = 100;

unsigned char hrm_manufacturerData[] = "stefaanLLC";

// create peripheral instance, see pinouts above
BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);

// nodig voor bonding
BLEBondStore bleBondStore = BLEBondStore(0); // op nRF51 reserveer 1kB flash page op hoogste adres

// uuid's can be:
//   16-bit: "ffff"
//  128-bit: "19b10010e8f2537e4f6cd104768a1214" (dashed format also supported)

// create one or more services
BLEService svcHeartRate = BLEService("180d"); // assigned UUID!

// heart rate service characteristics
BLECharacteristic charHeartRateMeasurement = BLECharacteristic("2a37", BLENotify,2); // valueSize = 2, we gaan 2 bytes sturen
BLECharCharacteristic charBodySensorLocation = BLECharCharacteristic("2a38", BLERead);
BLECharCharacteristic charHeartRateControlPoint = BLECharCharacteristic("2a39", BLEWrite);

// create one or more descriptors (optional)
//BLEDescriptor descriptor = BLEDescriptor("2901", "value");

// 2902 descriptor wordt blijkbaar door de BLEPeripheral lib afgehandeld
// check NRF51822.cpp
// unsigned char clientconfig[] = {0,0};
// descriptor 2902 is supposed to be 16 bit
//BLEDescriptor descClientCharacteristicConfiguration = BLEDescriptor("2902", clientconfig, 2);

void blePeripheralConnectHandler(BLECentral& central) {
  bleConnected = true;
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  bleConnected = false;
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}

void charHeartRateControlPointEventHandler (BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print(F("hrControlPoint written with value :"));
  Serial.println(characteristic[0]); // opgelet : value is een char*, niet persé een string!
  // TODO do reset expended energy
}

void setup() {
  Serial.begin(115200);
  
  // da werkt nie!
  //bleBondStore.clearData(); // on reboot
  //blePeripheral.setBondStore(bleBondStore);

  blePeripheral.setDeviceName(BLE_DEVICE_NAME);
  //blePeripheral.setAppearance(0x0); // wadisda?

  blePeripheral.setLocalName(BLE_DEVICE_NAME); // optional
  blePeripheral.setManufacturerData(hrm_manufacturerData, sizeof(hrm_manufacturerData)); // required for heartrate profile (??)
  blePeripheral.setAdvertisedServiceUuid(svcHeartRate.uuid()); // optional

  // add attributes (services, characteristics, descriptors) to peripheral
  blePeripheral.addAttribute(svcHeartRate);
  blePeripheral.addAttribute(charHeartRateMeasurement);
  blePeripheral.addAttribute(charBodySensorLocation);
  blePeripheral.addAttribute(charHeartRateControlPoint);

  // set initial value
  unsigned char hrVal[] = {0,100}; // 0 = flags, 100 = heartrate
  charHeartRateMeasurement.setValue(hrVal,2); 
  charBodySensorLocation.setValue(3); // FINGER

  // event handler for the BLE device
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  
  // event handler for the characteristic
  charHeartRateControlPoint.setEventHandler(BLEWritten, charHeartRateControlPointEventHandler);

  // begin initialization
  blePeripheral.begin();

} // setup

uint32_t ntfMillis = 0;

void loop() {

  blePeripheral.poll();

  if (millis() - ntfMillis > 1000)
  {
    // notify central about HR every second
    ntfMillis = millis();
    if ((bleConnected) && (charHeartRateMeasurement.subscribed())) {
      // eventueel canNotify of canIndicate gebruiken
      // deze gaven altijd 'true' van zodra een central connected was
      // maar in nRF51822 wordt gecheckt op buffer
      myHeartRate++; // simulatie
      //charHeartRateMeasurement.broadcast(); // da doet niets
      unsigned char hrVal[] = {0,myHeartRate};
      Serial.print(F("sending HR : ")); Serial.println(myHeartRate);
      bool retval;
      // dit stuurt data naar de central :
      retval = charHeartRateMeasurement.setValue(hrVal,2);
      if (!retval)
        Serial.println(F("setValue failed!"));
    }

    // out of curiosity check if RTC0 is used by BLE
    // -> ja dus!
    // dit verklaart allicht waarom debuggen met BLE niet lukt
    // misschien is er een manier om RTC0 te stoppen tijdens halt?
    // (RTCO.STOP=1 laten uitvoeren door de debugger bij een break)
    // dat zal allicht wel side effects hebben, zoals afbreken van opstaande connecties
    // Serial.print("RTC0.COUNTER = ");Serial.println(NRF_RTC0->COUNTER);


  }

} // loop
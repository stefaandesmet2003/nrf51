// https://webbluetoothcg.github.io/demos/heart-rate-sensor/

// peripheral manipulations
#include "nrf.h"

//ble
#include <SPI.h>
#include <BLEPeripheral.h>

// display output
#include <Wire.h>
#include <U8g2lib.h> 

// define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9
#define BLE_DEVICE_NAME "stefaan-cadence"

#define pinButton     4
#define pinBattery    5 // P0.05
#define pinCrankRev   17 // RX = 0.17

#define DEVICE_SLEEP_TIMEOUT 60000 //ms

// debouncing reed contact
#define DEBOUNCE_IDLE     0
#define DEBOUNCE_START    1
#define DEBOUNCE_DONE     2
#define DEBOUNCE_TIMEOUT  10 // ms

uint16_t myCrankRevs = 0;
uint32_t crankRevLastEventMillis = 0; // last crank event time in millis

uint8_t pinCrankRevDebounceState = DEBOUNCE_IDLE;
uint32_t pinCrankRevDebounceStartMillis;

uint32_t ntfMillis = 0;

bool bleConnected = false;
unsigned char csc_manufacturerData[] = "stefaanLLC";
BLEPeripheral blePeripheral = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);
BLEService svcSpeedCadence = BLEService("1816"); // assigned UUID!
// cadence sensor characteristics
BLECharacteristic charSpeedCadenceMeasurement = BLECharacteristic("2a5b", BLENotify,5); // valueSize = 5, we gaan 5 bytes sturen
BLECharacteristic charSpeedCadenceFeature = BLECharacteristic("2a5c", BLERead,2);

// the oled display
U8G2_SSD1306_64X32_1F_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); 
uint32_t displayMillis;
bool displayOn = false;

void blePeripheralConnectHandler(BLECentral& central) {
  bleConnected = true;
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  bleConnected = false;
}

void charHeartRateControlPointEventHandler (BLECentral& central, BLECharacteristic& characteristic) {
}

void setup() {
  // Serial.begin(115200);

  // 1. BLE setup
  blePeripheral.setDeviceName(BLE_DEVICE_NAME);
  //blePeripheral.setAppearance(0x0); // wadisda?

  blePeripheral.setAdvertisingInterval(500); // 100ms is default
  //can this also be used to save power ?
  // master will contact slave at most every 500ms and at least 1/s
  // that's enough because we want to notify only 1/s
  // tried in connect/disconnect handler -> works sometimes
  //blePeripheral.setConnectionInterval(400,800); // default : 40/80 = 50ms-100ms 

  blePeripheral.setLocalName(BLE_DEVICE_NAME); // optional
  blePeripheral.setManufacturerData(csc_manufacturerData, sizeof(csc_manufacturerData));
  blePeripheral.setAdvertisedServiceUuid(svcSpeedCadence.uuid());

  // add attributes (services, characteristics, descriptors) to peripheral
  blePeripheral.addAttribute(svcSpeedCadence);
  blePeripheral.addAttribute(charSpeedCadenceMeasurement);
  blePeripheral.addAttribute(charSpeedCadenceFeature);

  // set initial value
  unsigned char cadVal[] = {2,1,0,0,0}; // 2 = flags, 1,0 = cumulative crank revs; 0,0 = last crank event time
  unsigned char cscFeatureVal[] = {2,0}; // crank rev data supported
  charSpeedCadenceMeasurement.setValue(cadVal,5); 
  charSpeedCadenceFeature.setValue(cscFeatureVal,2); 

  // event handler for the BLE device
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  
  // event handler for the characteristic
  //charHeartRateControlPoint.setEventHandler(BLEWritten, charHeartRateControlPointEventHandler);

  // begin initialization
  blePeripheral.begin();
  blePeripheral.setTxPower(-40); // -40dB for saving power, instead of 0dB default

  // 2. setup wake-up from button & crankrev
  pinMode (pinButton, INPUT_PULLUP);
  pinMode (pinCrankRev, INPUT_PULLUP);

  NRF_GPIOTE->INTENSET = 0x0; // disable port events for now (default)

  // todo voor gpiote irq GPIOTE_IRQHandler (gcc_startup_nrf51.S)
  NVIC_SetPriority(GPIOTE_IRQn, 15);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_EnableIRQ(GPIOTE_IRQn);

  // display setup
  // for power saving : activate TWI peripheral only when data display is requested
  //Wire.begin();
  //u8g2.begin();
  //u8g2.clearBuffer();
  //u8g2.setFont(u8g2_font_ncenB08_tr);


} // setup

void notifyCrankRevs ()
{
  // notify central about crankrevs every second
  if ((bleConnected) && (charSpeedCadenceMeasurement.subscribed())) {
    // eventueel canNotify of canIndicate gebruiken
    // deze gaven altijd 'true' van zodra een central connected was
    // maar in nRF51822 wordt gecheckt op buffer
    uint16_t crankEventTime;
    unsigned char cadVal[5];
    crankEventTime = crankRevLastEventMillis % 64000;
    crankEventTime = (crankEventTime * 1024) / 1000;
    cadVal[0] = 2; // flags
    cadVal[1] = (unsigned char) (myCrankRevs & 0xFF);
    cadVal[2] = (unsigned char) ((myCrankRevs>>8) & 0xFF);
    cadVal[3] = (unsigned char) (crankEventTime & 0xFF);
    cadVal[4] = (unsigned char) ((crankEventTime>>8) & 0xFF);
    bool retval;
    // dit stuurt data naar de central :
    retval = charSpeedCadenceMeasurement.setValue(cadVal,5);
    if (!retval) {
      // too bad, hopefully better next time
    }
  }
  
} // notifyCrankRevs


void checkCrankRevs() {
  switch(pinCrankRevDebounceState) {
    case DEBOUNCE_IDLE :
      if (digitalRead(pinCrankRev) == LOW) {
        pinCrankRevDebounceState = DEBOUNCE_START;
        pinCrankRevDebounceStartMillis = millis();
      }
      break;
    case DEBOUNCE_START :
      if (digitalRead(pinCrankRev) == HIGH) {
        pinCrankRevDebounceState = DEBOUNCE_IDLE;
      }
      else {
        if ((millis() - pinCrankRevDebounceStartMillis) > DEBOUNCE_TIMEOUT) {
          pinCrankRevDebounceState = DEBOUNCE_DONE;
          // do stuff after debounce
          myCrankRevs++;
          crankRevLastEventMillis = millis();
        }
      }
      break;
    case DEBOUNCE_DONE :
      if (digitalRead(pinCrankRev) == HIGH) {
        pinCrankRevDebounceState = DEBOUNCE_IDLE;
      }
      break;
  } // switch
  
} // checkCrankRevs

void displayPage (uint8_t pageId) {
  Wire.begin();
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  // page0 : batteryVoltage & crankRevs
  if (pageId == 0) {
    // batteryVoltage
    int batVoltage = analogRead(pinBattery);
    u8g2.setCursor(1,10);
    u8g2.print("bat:");
    u8g2.print((float)batVoltage*0.019266); // conversion todo
    u8g2.print("V");
    u8g2.setCursor(1,20);
    u8g2.print("revs:");
    u8g2.print(myCrankRevs);
    u8g2.setCursor(1,30);
    if (bleConnected) u8g2.print("conn");
    else u8g2.print("disconn");
    u8g2.sendBuffer();    
  }
  Wire.end(); // disable TWI peripheral
  
} // displayPage

/*
 * on active ble connection, sensor notifies central every second
 * without connection, sensor stays awake as long as crank revolutions are detected
 * without connection, sensor goes to sleep after 60s if no crank revolutions are detected
 */
void loop() {

  blePeripheral.poll();

  // poll the pinCrankRev for pulses
  checkCrankRevs();

  if (bleConnected) {
    // notify crank revs every second
    if (millis() - ntfMillis > 1000) {
      notifyCrankRevs ();
      ntfMillis = millis();
    }
  }
  else {
    // wait for ble connection
    // after 1 minute without pulses go to sleep
    if ((millis() - crankRevLastEventMillis) > DEVICE_SLEEP_TIMEOUT) {
      // activate button wakeup
      NRF_GPIO->PIN_CNF[pinButton] &= ~GPIO_PIN_CNF_SENSE_Msk;
      NRF_GPIO->PIN_CNF[pinButton] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIO->PIN_CNF[pinCrankRev] &= ~GPIO_PIN_CNF_SENSE_Msk;
      NRF_GPIO->PIN_CNF[pinCrankRev] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
      NRF_GPIOTE->INTENSET = 0x80000000;
      /* using cpu sleep 
      // goto sleep
      __WFI();
      // TODO : moeten we hier ook de radio uitzetten ? JA
      // on wakeup
      // disable sense interrupt
      NRF_GPIOTE->INTENSET = 0;
      myCrankRevs = 0;
      crankRevLastEventMillis = millis();
      */
      /* using system off -> wake up with a reset */
      NRF_POWER->SYSTEMOFF = 0x1UL;
      while(1); 
      // won't come here
    }
  }

  // TODO
  // display : check side button and display pages
  if ((digitalRead(pinButton) == LOW) && ((millis() - displayMillis) > 500)) {
    displayMillis = millis();
    displayOn = true;
    displayPage(0);
  }
  if (displayOn && ((millis() - displayMillis) > 5000)) {
    Wire.begin();
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    Wire.end();
    displayOn = false;
  }  

} // loop


extern "C"{
void GPIOTE_IRQHandler(void)
{
  // events wissen -> dit neemt de int source weg!
  // hoe weet je nu welk event actief is? -> ze 1 voor 1 afgaan
  // maar in dit voorbeeld worden enkel IN[0] en PORTS geactiveerd
  if (NRF_GPIOTE->EVENTS_IN[0]) {
    NRF_GPIOTE->EVENTS_IN[0] = 0;
  }
  if (NRF_GPIOTE->EVENTS_PORT) {
    NRF_GPIOTE->EVENTS_PORT = 0;
  }

} // GPIOTE_IRQHandler

} // extern "C"

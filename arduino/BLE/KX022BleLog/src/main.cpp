/* a sketch to get the most out of BLE speed
  pushing as much accelerometer data as possible over a BLE UART connection
  turns out that this gives a lot of errors in the default BLESerial implementation
  (error returns not handled, leading to buffer overrides etc. )
  use ../test/kx022logger.html to display the data
  not working 100%
*/

#include <Wire.h>
#include <SPI.h>
#include "KX022.h"
#include <U8g2lib.h> // let's show output on the screen
#include <BLEPeripheral.h>
#include "BLESerial.h"

// define pins (varies per shield/board) - not used for nrf51
#define BLE_REQ   BLE_DEFAULT_REQ
#define BLE_RDY   BLE_DEFAULT_RDY
#define BLE_RST   BLE_DEFAULT_RST

// create ble serial instance, see pinouts above
BLESerial BLESerial(BLE_REQ, BLE_RDY, BLE_RST);

// the accelerometer
KX022 acc;
// the oled display
U8G2_SSD1306_64X32_1F_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); 

float xyz[3];

int pin_PIN1 = 13; // PIN1 = P0.13, PIN2 = P0.12, ADDR = P0.15
void KX022_IRQHandler(void);
void KX022_DoInterrupts(void);
void runParser();

volatile bool kx022InterruptFlag = false;

// for test
bool isBleOn = false;

void startBle() {
  // setup ble uart
  // custom services and characteristics can be added as well
  BLESerial.setLocalName("UART");
  BLESerial.begin();
  // hiermee halen we max 130samples/s over ble uart
  BLESerial.setConnectionInterval(6,6); // min 7.5ms, max 22.5ms
}

void stopBle() {
  BLESerial.end();
}

void setup()
{
  Wire.begin();
  
  Serial.begin(115200);
  Serial.println("START!!");

  delay(500); // wait for the accelerometer to power up

  // opgelet KX022.cpp aangepast om dataready ints te krijgen!
  acc.init();
  u8g2.begin();
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0,10,"KX022!");
  u8g2.sendBuffer();					// transfer internal memory to the display

  // de PIN1 int pin van de accelerometer komt op p0.13
  // int active high geconfiged in de kx022 lib
  //pinMode (pin_PIN1,INPUT); 
  attachInterrupt(pin_PIN1,KX022_IRQHandler, RISING);
  // somehow I miss the first rising edge of the PIN1,
  // even if I attachInterrupt before acc.init()
  acc.readRegister(KX022_INT_REL);


  // setup ble uart
  startBle();
  isBleOn = true;

} // setup

int16_t sampleXYZ[3];
float curG = 0.0;
float minG = 2.0;
float maxG = 0.0;
uint32_t sampleCount=0;
uint32_t lastSampleCount=0;

uint32_t displayMillis;
uint32_t errorCount = 0;
bool blinkie = false;

void loop()
{

  // dit is de acc.poll()
  if (kx022InterruptFlag){

    KX022_DoInterrupts();
    // lets calculate the g for every sample
    // maakt allicht niet veel uit om de x²+y²+z² in uint32_t uit te rekenen
    curG += sampleXYZ[0]*sampleXYZ[0];
    curG += sampleXYZ[1]*sampleXYZ[1];
    curG += sampleXYZ[2]*sampleXYZ[2];
    curG = sqrt((float)curG)*4.0/32768.0; // 4.0 = range
    if (curG > maxG) maxG = curG;
    if (curG < minG) minG = curG;

    // ble log
    // write sample on ble uart
    
    if (BLESerial) {
      //BLESerial.print(sampleCount); BLESerial.print(',');
      BLESerial.print(sampleXYZ[0]); BLESerial.print(',');
      BLESerial.print(sampleXYZ[1]); BLESerial.print(',');
      BLESerial.println(sampleXYZ[2]);
      BLESerial.flush();
      // sds : flush aangepast om te wachten op notify buffer
      // hier is iets serieus mis met BLESerial, want 
      // de flush in de library als de buffer vol is, werkt niet consistent
      // en data gaan verloren
    }
  }

  BLESerial.poll(); // keep ble alive

  if ((millis() - displayMillis) > 1000) {
    // update display
    u8g2.clearBuffer();
    u8g2.setCursor(0,10);
    if (blinkie) {
      u8g2.print("g: ");  
    }
    else {
      u8g2.print("g  ");
    }
    u8g2.print(curG);
    blinkie = !blinkie;

    // show samples/s since last display update
    // dit geeft 49samples/s voor ODR 50Hz, >190 voor 200Hz, >380 voor 400Hz
    uint32_t samplesPerSecond = 1000*(sampleCount - lastSampleCount)/(millis() - displayMillis);
    u8g2.print(" ");u8g2.print(samplesPerSecond); 
    //u8g2.print(" ");u8g2.print(errorCount); 
    u8g2.setCursor(0,20);
    u8g2.print("min-g: "); u8g2.print(minG);
    u8g2.setCursor(0,30);
    u8g2.print("max-g: "); u8g2.print(maxG);
    u8g2.sendBuffer();

    displayMillis = millis();
    lastSampleCount = sampleCount;
  }

  runParser();  
} // loop

void KX022_IRQHandler(void) {
  // ISR context -> no i2c access to device here!
  kx022InterruptFlag = true;
  // we can't eliminate the source of the external interrupt yet
  // so disable this irq until we handle the interrupt in KX022_DoInterrupts
  NVIC_DisableIRQ(GPIOTE_IRQn);
} // KX022_IRQHandler

void show(char *str)
{
  u8g2.clearBuffer();
  u8g2.setCursor(0,10);
  u8g2.print(str);
  u8g2.setCursor(0,20);
  u8g2.sendBuffer();
}
void KX022_DoInterrupts(void) {

  // read INS2
  uint8_t ins2 = acc.readRegister(KX022_INS2);
  if (ins2 & INS2_DRDY){
    sampleCount++;
    // read the sample
    acc.getRawXYZ(sampleXYZ);
  }

  // clear int
  acc.readRegister(KX022_INT_REL);

  kx022InterruptFlag = false;
  NVIC_EnableIRQ(GPIOTE_IRQn);  

} // KX022_DoInterrupts


#define PARSER_IDLE         (0)
#define PARSER_HAVE_CMD     (1)
#define PARSER_HAVE_PARAM1  (2)
#define PARSER_CAN_EXECUTE  (4)
#define PARSER_INPUT_ERROR  (5)
#define PARSER_CMD_READ     'r'
#define PARSER_CMD_WRITE    'w'
#define PARSER_CMD_TOGGLEBLE 's'

uint8_t parserState = PARSER_IDLE;
uint8_t parseCommand, parseRegAddress, parseRegParam; // param = regValue(w) or readCount (r)

void runParser() {
  if (Serial.available()) {
    switch (parserState) {
      case PARSER_IDLE : 
      {
        char cmd = Serial.read();
        if ((cmd == 'r') || (cmd == 'R')) {
          parseCommand = PARSER_CMD_READ;
          parserState = PARSER_HAVE_CMD;
        }
        else if ((cmd == 'w') || (cmd == 'W')) {
          parseCommand = PARSER_CMD_WRITE;
          parserState = PARSER_HAVE_CMD;
        }
        else if ((cmd == 's') || (cmd == 'S')) {
          parseCommand = PARSER_CMD_TOGGLEBLE;
          parserState = PARSER_CAN_EXECUTE;
        }
        else if ((cmd == ' ') || (cmd == '\r') || (cmd == '\n'))
        {
          // white space before command -> do nothing
        }
        else {
          parserState = PARSER_INPUT_ERROR;
        }
        break;
      }
      case PARSER_HAVE_CMD :
      {
        parseRegAddress = (uint8_t) Serial.parseInt();
        parserState = PARSER_HAVE_PARAM1;
        break;
      }
      case PARSER_HAVE_PARAM1 :
      {
        char inChar = Serial.peek();
        if ((inChar == '\r') || (inChar == '\n')) {
          // hit eol
          Serial.read(); // consume the character
          if (parseCommand == PARSER_CMD_READ) {
            parseRegParam = 1; // read 1 byte
            parserState = PARSER_CAN_EXECUTE;
          }
          else {
            // missing argument
            parserState = PARSER_INPUT_ERROR;
          }
        }
        else {
          parseRegParam = (uint8_t) Serial.parseInt();
          parserState = PARSER_CAN_EXECUTE;
        }
        break;
      }
      case PARSER_INPUT_ERROR :
      {
        char inChar = Serial.read();
        if ((inChar == '\r') || (inChar == '\n')) {
          parserState = PARSER_IDLE;
         }
        break;
      }
    } // parserState
  }
  // execute
  uint8_t regVal;
  if (parserState == PARSER_CAN_EXECUTE) {
    if (parseCommand == PARSER_CMD_TOGGLEBLE) {
      Serial.print("switching");
      if (isBleOn) {
        stopBle();
        isBleOn = false;
        Serial.println(" off BLE");
      }
      else {
        startBle();
        isBleOn = true;
        Serial.println(" on BLE");

      }
    }
    Serial.print("> "); Serial.print((char)parseCommand); Serial.print(' ');
    Serial.print(parseRegAddress);Serial.print(' '); 
    Serial.print(parseRegParam); Serial.print(" : ");
    if (parseCommand == PARSER_CMD_READ) {
      regVal = acc.readRegister(parseRegAddress);
      Serial.println(regVal,HEX);
    }
    else if (parseCommand == PARSER_CMD_WRITE) {
      acc.writeRegister(parseRegAddress,parseRegParam);
      Serial.println("OK!");
    }

    parserState = PARSER_IDLE;
  }
  /*
  else if (parserState == PARSER_INPUT_ERROR) {
    Serial.println("parse error!");
  }
  */
} // runParser

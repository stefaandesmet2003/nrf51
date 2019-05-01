#include <Wire.h>
#include <SPI.h>
#include "KX022.h"
#include <U8g2lib.h> // let's show output on the screen

KX022 acc;

U8G2_SSD1306_64X32_1F_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); 

float xyz[3];
int pin_PIN1 = 13; // PIN1 = P0.13, PIN2 = P0.12, ADDR = P0.15
void KX022_IRQHandler(void);
void KX022_DoInterrupts(void);
volatile bool kx022InterruptFlag = false;

void setup()
{
  Wire.begin();
  
  Serial.begin(115200);
  Serial.println(__FILE__);

  delay(500); // wait for the accelerometer to power up


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
  
} // setup

#define PARSER_IDLE         (0)
#define PARSER_HAVE_CMD     (1)
#define PARSER_HAVE_PARAM1  (2)
#define PARSER_CAN_EXECUTE  (4)
#define PARSER_INPUT_ERROR  (5)
#define PARSER_CMD_READ     'r'
#define PARSER_CMD_WRITE    'w'

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

void showXYZ() {
    acc.getAccelXYZ(xyz);

    u8g2.clearBuffer();
    u8g2.setCursor(0,10);
    u8g2.print("X: "); u8g2.print(xyz[0]);
    u8g2.setCursor(0,20);
    u8g2.print("Y: "); u8g2.print(xyz[1]);
    u8g2.setCursor(0,30);
    u8g2.print("Z: "); u8g2.print(xyz[2]);
    u8g2.sendBuffer();
} // showXYZ

void loop()
{
  runParser();

  // dit is de acc.poll()
  if (kx022InterruptFlag)
    KX022_DoInterrupts();
  
} // loop

void KX022_IRQHandler(void) {
  // ISR context -> no i2c access to device here!
  kx022InterruptFlag = true;
  // we can't eliminate the source of the external interrupt yet
  // so disable this irq until we handle the interrupt in KX022_DoInterrupts
  NVIC_DisableIRQ(GPIOTE_IRQn);
} // KX022_IRQHandler

uint16_t cntWakeup = 0;
uint16_t cntTap = 0;
uint16_t cntTilt = 0;

void show(char *str)
{
  u8g2.clearBuffer();
  u8g2.setCursor(0,10);
  u8g2.print(str);
  u8g2.setCursor(0,20);
  u8g2.print(cntWakeup);u8g2.print(' ');
  u8g2.print(cntTap);u8g2.print(' ');
  u8g2.print(cntTilt);
  u8g2.sendBuffer();
}
void KX022_DoInterrupts(void) {

  // read INS2
  uint8_t ins2 = acc.readRegister(KX022_INS2);
  if (ins2 & INS2_BFI)
    show("BFI");
  if (ins2 & INS2_WMI)
    show("WMI");
  if (ins2 & INS2_DRDY)
    show("DRDY");
  if (ins2 & INS2_TDTS) {
    if ((ins2 & INS2_TDTS) == INS2_SINGLETAP)
      show("1tap - ");
    if ((ins2 & INS2_TDTS) == INS2_DOUBLETAP)
      show("2tap - ");
    cntTap++;
    // find motion direction
    //uint8_t direction = acc.readRegister(KX022_INS1);
    //Serial.println(direction,HEX);
  }

  if (ins2 & INS2_WUFS) {
    show("wakeup - ");
    cntWakeup++;
    // find motion direction
    //uint8_t direction = acc.readRegister(KX022_INS3);
    //Serial.println(direction,HEX);
  }
  if (ins2 & INS2_TPS) {
    show("tilt - ");
    cntTilt++;
    // find tilt direction
    //uint8_t prevDirection = acc.readRegister(KX022_TSPP);
    //uint8_t curDirection = acc.readRegister(KX022_TSCP);
    //Serial.print(prevDirection,HEX); Serial.print("->");
    //Serial.println(curDirection,HEX);
  }

  // clear int
  acc.readRegister(KX022_INT_REL);

  kx022InterruptFlag = false;
  NVIC_EnableIRQ(GPIOTE_IRQn);  

} // KX022_DoInterrupts




/*
void KX022_DoInterrupts(void) {

  // read INS2
  uint8_t ins2 = acc.readRegister(KX022_INS2);
  if (ins2 & INS2_BFI)
    Serial.println("BFI");
  if (ins2 & INS2_WMI)
    Serial.println("WMI");
  if (ins2 & INS2_DRDY)
    Serial.println("DRDY");
  if (ins2 & INS2_TDTS) {
    if ((ins2 & INS2_TDTS) == INS2_SINGLETAP)
      Serial.print("1tap - ");
    if ((ins2 & INS2_TDTS) == INS2_DOUBLETAP)
      Serial.print("2tap - ");
    // find motion direction
    uint8_t direction = acc.readRegister(KX022_INS1);
    Serial.println(direction,HEX);
  }

  if (ins2 & INS2_WUFS) {
    Serial.print("wakeup - ");
    // find motion direction
    uint8_t direction = acc.readRegister(KX022_INS3);
    Serial.println(direction,HEX);
  }
  if (ins2 & INS2_TPS) {
    Serial.print("tilt - ");
    // find tilt direction
    uint8_t prevDirection = acc.readRegister(KX022_TSPP);
    uint8_t curDirection = acc.readRegister(KX022_TSCP);
    Serial.print(prevDirection,HEX); Serial.print("->");
    Serial.println(curDirection,HEX);
  }

  // clear int
  acc.readRegister(KX022_INT_REL);

  kx022InterruptFlag = false;
  NVIC_EnableIRQ(GPIOTE_IRQn);  

} // KX022_DoInterrupts

*/
/* a completely useless demo using a few features of the b-watch
   while making fun of work
   - display
   - ui based on recognizing taps/double taps by the accelerometer
   - raw accelero data
*/

#include <Wire.h>
#include <U8g2lib.h> // let's show output on the screen
#include "KX022.h"

#define BUZZERPIN   (7)
#define BUTTONPIN   (4)

// de gewone B
#define nmbs_width 32
#define nmbs_height 21
static unsigned char nmbs_bits[] = {
   0x00, 0xf0, 0x0f, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0xc0, 0x01, 0x80, 0x03,
   0x70, 0xc0, 0x01, 0x0e, 0x38, 0xf8, 0x0f, 0x1c, 0x1c, 0x3e, 0x3e, 0x38,
   0x0e, 0x3e, 0x7c, 0x70, 0x06, 0x3c, 0x7c, 0x60, 0x07, 0x3c, 0x7e, 0xe0,
   0x07, 0xfc, 0x3f, 0xe0, 0x07, 0xfc, 0x1f, 0xe0, 0x07, 0x3c, 0x3c, 0xe0,
   0x07, 0x3c, 0x78, 0xe0, 0x06, 0x3c, 0x78, 0x60, 0x0e, 0x3e, 0x78, 0x70,
   0x1c, 0x3e, 0x3c, 0x38, 0x38, 0xf8, 0x1f, 0x1c, 0x70, 0xf0, 0x07, 0x0e,
   0xc0, 0x01, 0x80, 0x03, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0xf0, 0x0f, 0x00 };


// de omgevallen B
#define nmbsContra_width 21
#define nmbsContra_height 32
static unsigned char nmbsContra_bits[] = {
   0x00, 0x1f, 0x00, 0xc0, 0x7f, 0x00, 0xe0, 0xff, 0x00, 0x70, 0xc0, 0x01,
   0x38, 0x80, 0x03, 0x18, 0x00, 0x03, 0x0c, 0x00, 0x06, 0x04, 0x00, 0x04,
   0x06, 0x00, 0x0c, 0x62, 0xc0, 0x08, 0xe2, 0xff, 0x08, 0xf2, 0xff, 0x09,
   0xf9, 0xff, 0x11, 0xf9, 0xff, 0x11, 0x19, 0x0c, 0x13, 0x19, 0x0c, 0x13,
   0x19, 0x0c, 0x13, 0x19, 0x9c, 0x11, 0x39, 0xfe, 0x11, 0xf1, 0xff, 0x11,
   0xf2, 0xff, 0x08, 0xe2, 0xfb, 0x08, 0xc2, 0x71, 0x08, 0x86, 0x20, 0x0c,
   0x84, 0x20, 0x04, 0x0c, 0x00, 0x06, 0x18, 0x00, 0x03, 0x38, 0x80, 0x03,
   0x70, 0xc0, 0x01, 0xe0, 0xff, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x1f, 0x00 };

// the oled display
U8G2_SSD1306_64X32_1F_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); 

// the accelerometer
KX022 acc;

float xyz[3];
int16_t sampleXYZ[3];
float curG = 0.0;
uint8_t displayG[64]; // 1 sample per pixel
int pin_PIN1 = 13; // PIN1 = P0.13, PIN2 = P0.12, ADDR = P0.15
void KX022_IRQHandler(void);
void KX022_DoInterrupts(void);
volatile bool kx022InterruptFlag = false;
bool singleTapped = false;
bool doubleTapped = false;

uint32_t displayMillis;

void setup()
{
  pinMode(BUZZERPIN, OUTPUT);
  pinMode (BUTTONPIN, INPUT_PULLUP);
  digitalWrite (BUZZERPIN,0); // buzzer uit

  Serial.begin(115200);
  Wire.begin();

  u8g2.begin();
  u8g2.clearBuffer();					// clear the internal memory
  //u8g2.setFont(u8g2_font_ncenB08_tr); // u8g2_font_oskool_tr
  u8g2.setFont(u8g2_font_courB08_tr); // u8g2_font_courB08_tr // u8g2_font_6x12_tr  u8g2_font_pcsenior_8r 

  delay(500); // wait for the accelerometer to power up

  // opgelet KX022.cpp aangepast om dataready ints te krijgen!
  acc.init();
  // de PIN1 int pin van de accelerometer komt op p0.13
  // int active high geconfiged in de kx022 lib
  //pinMode (pin_PIN1,INPUT); 
  attachInterrupt(pin_PIN1,KX022_IRQHandler, RISING);
  // somehow I miss the first rising edge of the PIN1,
  // even if I attachInterrupt before acc.init()
  acc.readRegister(KX022_INT_REL);

  Serial.println("START!!");

} // setup

// threshold 0..1023
uint8_t getRandomMask(uint16_t threshold) {
  uint8_t rMask;
  uint8_t maskBit;
  
  rMask = 0;
  for (int i=0; i<8; i++) {
    maskBit = (random(1024)> threshold) ? 1 : 0;
    rMask = (rMask << 1) | maskBit;
  }
  return rMask;
} // getRandomMask

uint8_t fromImage[256]; // 64x32 pixels
uint8_t toImage[256]; // 64x32 pixels
uint8_t disappearStep = 0;
uint8_t appearStep = 0;
uint8_t contrasAnimationStatus = 0;

// return : true : finished
bool animateDisappear (uint8_t *image, uint8_t totalSteps, uint8_t *step) {
  uint8_t *frameBufferPtr = u8g2.getBufferPtr();

  if (*step >= totalSteps) return true;
  if (*step == 0) {
    memcpy(frameBufferPtr,image,256);
  }
  else {
    for (int i=0;i<256;i++) {
      uint8_t rMask = getRandomMask(5*(*step));
      frameBufferPtr[i] &= rMask;
    }
  }
  u8g2.sendBuffer();
  *step = *step + 1;

  if (*step >= totalSteps) return true;
  else return false;

} // animateDisappear

// return : true : finished
bool animateAppear (uint8_t *image, uint8_t totalSteps, uint8_t *step) {
  uint8_t *frameBufferPtr = u8g2.getBufferPtr();

  if (*step >= totalSteps) return true;
  if (*step == 0) {
    u8g2.clearBuffer();
  }
  else {
    for (int i=0;i<256;i++) {
      uint8_t rMask = getRandomMask(5*(*step));
      frameBufferPtr[i] |= (image[i] & (~rMask));
    }
  }

  u8g2.sendBuffer();
  *step = *step + 1;

  if (*step >= totalSteps) return true;
  else return false;

} // animateAppear

void doAnimationSamenContras () {
  bool animationAllDone = false;
  // 30 Hz animation framerate
  if ((millis() - displayMillis) > 33) {
    if (contrasAnimationStatus == 0) {
      uint8_t *frameBufferPtr = u8g2.getBufferPtr();
      // setup from image
      u8g2.clearBuffer();
      u8g2.setCursor(35,10);
      u8g2.print("SAMEN");
      u8g2.setCursor(40,20);
      u8g2.print("PROS");
      u8g2.drawXBM(0, 0, nmbs_width, nmbs_height, nmbs_bits);
      memcpy (fromImage, frameBufferPtr,256);

      // setup to image
      u8g2.clearBuffer();
      u8g2.setCursor(24,10);
      u8g2.print("SAMEN");
      u8g2.setCursor(21,30);
      u8g2.print("CONTRAS");
      u8g2.drawXBM(0, 0, nmbsContra_width, nmbsContra_height, nmbsContra_bits);
      memcpy (toImage, frameBufferPtr,256);
      disappearStep = 0;
      appearStep = 0;
      contrasAnimationStatus = 1;
    }
    else if (contrasAnimationStatus == 1) {
      // disappear stage
      if (animateDisappear (fromImage,75,&disappearStep)) {
        contrasAnimationStatus = 2; // go to appear stage
      }
    }
    else if (contrasAnimationStatus == 2) {
      // appear stage
      if (animateAppear (toImage,75,&appearStep)) {
        animationAllDone = true;
      }
    }
    displayMillis = millis();
  } // 30Hz
  if (!animationAllDone) {
    // eliminate occasional taps before animation is complete
    singleTapped = false;
    doubleTapped = false;
  }
} // doAnimationSamenContras

void doIntro () {
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawFrame(0,0,64,32);
  u8g2.drawXBM(16, 6, nmbs_width, nmbs_height, nmbs_bits);
  u8g2.sendBuffer();					// transfer internal memory to the display

} // doIntro

void doPlanning () {
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawFrame(0,0,64,32);
  u8g2.drawStr (4,10,"Planning");
  u8g2.drawStr (20,20,"4.0");
  u8g2.drawStr (18,30,"DEMO");
  u8g2.sendBuffer();					// transfer internal memory to the display

} // doPlanning

bool tc22done = false;
void doTC22 () {
  if (tc22done)
    return;
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawFrame(0,0,64,32);
  u8g2.drawStr (25,10,"SAP ");
  u8g2.drawStr (2,20,"Transactie :");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"T");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"TC");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"TC2");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"TC22");
  u8g2.sendBuffer();
  tc22done = true;

} // doTC22

bool tc21done = false;
void doTC21 () {
  if (tc21done)
    return;
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawFrame(0,0,64,32);
  u8g2.drawStr (25,10,"SAP ");
  u8g2.drawStr (2,20,"Transactie :");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"T");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"TC");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"TC2");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr (20,30,"TC21");
  u8g2.sendBuffer();
  tc21done = true;

} // doTC21

int32_t budget;
void doBudget()
{
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.drawFrame(0,0,64,32);
  u8g2.drawStr (4,10,"Live");
  u8g2.drawStr (20,20,"Budget");

  if ((singleTapped) && (budget)) {
    budget = 10000000;
    singleTapped = false;
    u8g2.setCursor(10,30);
    u8g2.print(budget);
    u8g2.sendBuffer();
    delay(500);
  }
  u8g2.setCursor(10,30);
  u8g2.print(budget);
  u8g2.sendBuffer();

  budget -= 19427;
  if (budget < 0) budget = 0;

} // doBudget

void doLiveImmo () {
  singleTapped = false; 
  u8g2.clearBuffer();
  u8g2.drawFrame(0,0,64,32);
  u8g2.drawStr (4,8,"Live Immo");
  u8g2.setCursor(10,20);
  for (int i=0;i<64;i++) {
    u8g2.drawPixel(i,displayG[i]);
  }
  u8g2.sendBuffer();
} // doLiveImmo

uint32_t stiptheidMillis;
uint8_t stiptheidState = 0;
void doStiptheid () {
  switch (stiptheidState) {
    case 0 :
      // countdown
      u8g2.clearBuffer();
      u8g2.drawFrame(0,0,64,32);
      u8g2.drawStr (5,10,"Stiptheid");
      u8g2.drawStr (10,20,"09:29:54");
      u8g2.sendBuffer();
      delay(1000);
      u8g2.drawStr (10,20,"           ");
      u8g2.sendBuffer();
      u8g2.drawStr (10,20,"09:29:55");
      u8g2.sendBuffer();
      delay(1000);
      u8g2.drawStr (10,20,"           ");
      u8g2.sendBuffer();
      u8g2.drawStr (10,20,"09:29:56");
      u8g2.sendBuffer();
      delay(1000);
      u8g2.drawStr (10,20,"           ");
      u8g2.sendBuffer();
      u8g2.drawStr (10,20,"09:29:57");
      u8g2.sendBuffer();
      delay(1000);
      u8g2.drawStr (10,20,"           ");
      u8g2.sendBuffer();
      u8g2.drawStr (10,20,"09:29:58");
      u8g2.sendBuffer();
      delay(1000);
      u8g2.drawStr (10,20,"           ");
      u8g2.sendBuffer();
      u8g2.drawStr (10,20,"09:29:59");
      u8g2.sendBuffer();
      delay(1000);
      u8g2.drawStr (10,20,"           ");
      u8g2.sendBuffer();
      u8g2.drawStr (10,20,"09:30:00");
      u8g2.sendBuffer();
      delay(2000);
      stiptheidState = 1;
      stiptheidMillis = millis();
      digitalWrite(BUZZERPIN,1);
      singleTapped = false;
      doubleTapped = false;
      break;
    case 1 :
      if ((millis()-stiptheidMillis) > 1000) {
        digitalWrite(BUZZERPIN,1);
        u8g2.clearBuffer();
        u8g2.drawFrame(0,0,64,32);
        u8g2.drawStr (5,10,"Stiptheid");
        u8g2.drawStr (4,20,"SYNCHRO !!!");
        u8g2.sendBuffer();
        stiptheidState = 2;
        stiptheidMillis = millis();
      }
      break;
    case 2 :
      if ((millis()-stiptheidMillis) > 1000) {
        digitalWrite(BUZZERPIN,0);
        u8g2.clearBuffer();
        u8g2.drawFrame(0,0,64,32);
        u8g2.drawStr (5,10,"Stiptheid");
        u8g2.drawStr (4,20,"SYNCHRO");
        u8g2.sendBuffer();
        stiptheidState = 1;
        stiptheidMillis = millis();
      }
      break;
  }
} // doStiptheid

bool buttonHasToggled = true;
uint8_t idPage = 0;

void loop()
{
  // check the button
  if (digitalRead(BUTTONPIN) == 0)
  {
    if (buttonHasToggled) {
      idPage++;
      buttonHasToggled = false;
    }
  }
  else {
      buttonHasToggled = true;
  }


  // handle accelerometer interrupts
  // dit is de acc.poll()
  if (kx022InterruptFlag){

    KX022_DoInterrupts();
    // lets calculate the g for every sample
    // maakt allicht niet veel uit om de x²+y²+z² in uint32_t uit te rekenen
    curG += sampleXYZ[0]*sampleXYZ[0];
    curG += sampleXYZ[1]*sampleXYZ[1];
    curG += sampleXYZ[2]*sampleXYZ[2];
    curG = sqrt((float)curG)*4.0/32768.0; // 4.0 = range - niet juist!! moet 32768 zijn!!

    // g's in een arrayke opslaan voor display bij live immo
    for (int i=0;i<63;i++) {
      displayG[i] = displayG[i+1];
    }
    displayG[63] = 41 - (int) (20*curG); // float->int cast = truncate goed genoeg
  }

  if (idPage > 7) idPage = 0;
  if (idPage != 6) digitalWrite(BUZZERPIN,0); // enkel buzzer bij stiptheid

  if ((millis() - displayMillis) > 30) {
    switch (idPage) {
      case 0 :
        doIntro();
        budget = 10000000;
        tc22done = false;
        tc21done = false;
        stiptheidState = 0;
        contrasAnimationStatus = 0; // go back to init
        break;
      case 1 :
        doPlanning();
        break;
      case 2 :
        doTC22();
        break;
      case 3 :
        doBudget();
        break;
      case 4 :
        doTC21();
        break;
      case 5 : 
        doLiveImmo();
        break;
      case 6 : 
        doStiptheid();
        break;
      case 7 : 
        doAnimationSamenContras();
        break;
    } // idPage

  // check taps if the pages haven't handled them yet
  if (singleTapped || doubleTapped) {
    idPage++;
    singleTapped = false;
    doubleTapped = false;
  }

  } // display refresh

} // loop

void KX022_IRQHandler(void) {
  // ISR context -> no i2c access to device here!
  kx022InterruptFlag = true;
  // we can't eliminate the source of the external interrupt yet
  // so disable this irq until we handle the interrupt in KX022_DoInterrupts
  NVIC_DisableIRQ(GPIOTE_IRQn);
} // KX022_IRQHandler

void KX022_DoInterrupts(void) {

  // read INS2
  uint8_t ins2 = acc.readRegister(KX022_INS2);
  if (ins2 & INS2_DRDY){
    // read the sample
    acc.getRawXYZ(sampleXYZ);
  }
  /*
  if (ins2 & INS2_BFI)
    show("BFI");
  if (ins2 & INS2_WMI)
    show("WMI");
  */
  if (ins2 & INS2_TDTS) {
    if ((ins2 & INS2_TDTS) == INS2_SINGLETAP)
      singleTapped = true;
    if ((ins2 & INS2_TDTS) == INS2_DOUBLETAP)
      doubleTapped = true;
    // find motion direction
    //uint8_t direction = acc.readRegister(KX022_INS1);
    //Serial.println(direction,HEX);
  }
  /*
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
  */

  // clear int
  acc.readRegister(KX022_INT_REL);

  kx022InterruptFlag = false;
  NVIC_EnableIRQ(GPIOTE_IRQn);  

} // KX022_DoInterrupts
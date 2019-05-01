
//EC11 geeft wel systematisch 4 pulsen voor elke click
// met debounce filter aan/uit maakt niet uit
// met sample period 6 of 7 en debounce filter aan, geen pulsen

// test van de qdec
// op de 2 pinnen die we nog over hebben (tx & rx)
#include <Arduino.h>
#include "nrf.h"
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_64X32_1F_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft maar een half scherm

// we kunnen dus geen serial gebruiken voor deze sketch 
int pinA = 17; // rx
int pinB = 18; // tx
int pinButton = 4; // for switching sample period of qdec

int32_t pulses = 0;
int32_t badPulses = 0;
int samplePeriod = 3;
bool hasToggled = true;

// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(pinButton, INPUT_PULLUP);
  // eerst gpio pins als input configureren vooraleer qdec te enablen!
  // PULLUP nodig, met PULLDOWN krijgt ge geen pulsen
  // zonder pull krijgt ge spurious signals (pulses & bad pulses)
  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP);

  NRF_QDEC->PSELA = pinA;
  NRF_QDEC->PSELB = pinB;
  NRF_QDEC->PSELLED = 0xFFFFFFFF; // unused
  NRF_QDEC->SAMPLEPER = 0x3; // 1024us sample period, met EC11 beste keus
  NRF_QDEC->DBFEN = 0x0UL; // 0x1 = enable debouce filter

  NRF_QDEC->ENABLE = 1; //enable the peripheral
  NRF_QDEC->TASKS_START = 0x1UL; // start the qdec
  NRF_QDEC->TASKS_READCLRACC = 0x1UL; // clear ACC

  // display init
  u8g2.begin();
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
  u8g2.drawStr(0,10,"QDEC test!");
  u8g2.sendBuffer();					// transfer internal memory to the display

} // setup

void displayUpdate () {
  u8g2.clearBuffer();
  u8g2.setCursor(0,10);
  u8g2.print(pulses);
  u8g2.setCursor(0,20);
  u8g2.print(samplePeriod);
  u8g2.sendBuffer();					// transfer internal memory to the display

}
// the loop function runs over and over again forever
void loop() {
  if ((NRF_QDEC->ACC) || (NRF_QDEC->ACCDBL)) {
    NRF_QDEC->TASKS_READCLRACC = 0x1UL; //clear
    // now read the ACC and DBL from the backup register
    pulses += NRF_QDEC->ACCREAD;
    badPulses += NRF_QDEC->ACCDBLREAD;
    displayUpdate();
  }
  if (digitalRead(pinButton) == 0)
  {
    if (hasToggled) {
      samplePeriod++;
      if (samplePeriod >= 8) {
        samplePeriod = 0;
      }
      NRF_QDEC->SAMPLEPER = samplePeriod; 
      displayUpdate();
      hasToggled = false;
    }
  }
  else {
      hasToggled = true;
  }  

} // loop


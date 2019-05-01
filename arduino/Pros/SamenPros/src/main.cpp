/*

after a couple of tests we figured out the correct u8g2 display configuration required

*/

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//U8G2_SSD1306_64X32_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1);

//U8G2_SSD1306_64X32_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft ruis
//U8G2_SSD1306_64X32_NONAME_1_2ND_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft niets
//U8G2_SSD1306_64X32_1F_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft maar een paar lijnen
//U8G2_SSD1306_64X32_1F_1_2ND_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft niets
//U8G2_SSD1306_64X32_NONAME_2_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft ruis
//U8G2_SSD1306_64X32_NONAME_2_2ND_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft niets
//U8G2_SSD1306_64X32_1F_2_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft maar een half scherm
// End of constructor list
//U8G2_SSD1306_64X32_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft ruis
//U8G2_SSD1306_64X32_NONAME_F_2ND_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); //geeft niets

U8G2_SSD1306_64X32_1F_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 2, /* dc=*/ 0, /* reset=*/ 1); 


void setup(void) {
  u8g2.begin();
}

void loop(void) {
  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_ncenB10_tr);	// choose a suitable font
  u8g2.drawStr(10,11,"Samen");	// write something to the internal memory
  u8g2.drawStr(10,25,"PROS!");  // write something to the internal memory
  u8g2.sendBuffer();					// transfer internal memory to the display
  delay(1000);  
}


/* a bunch of experiments :
  - SYSTEM OFF and WFI cpu sleep 
  - internal temperature sensor
  - watchdog function
*/
/* nrf.h needed for register manipulation */

#include <Arduino.h>
#include "nrf.h"

int pinBuzzer = 7;
int pinButton = 4;

uint32_t tempMeasMillis;

float readTemperature ();
void printResetReason();
void watchdogSetup();
void testSystemOffAndWatchdog();


void setup() {
  pinMode(pinBuzzer, OUTPUT);
  pinMode (pinButton, INPUT_PULLUP);
  digitalWrite (pinBuzzer,0); // buzzer uit
  Serial.begin(115200);
  Serial.println("hello bwatch!");

  // print reset reason
  printResetReason();

  //watchdogSetup();

  // setup gpiote int from button
  // methode 1 : via gpiote -> en EVENT_IN[0] genereert de gpiote int
  // hier wordt de pinMode INPUT_PULLUP ongedaan gemaakt
  // want de pin wordt overgenomen door GPIOTE
  //NRF_GPIOTE->CONFIG[0] = 0x20401;
  //NRF_GPIOTE->INTENSET = 0x1;

  // methode 2 : pin blijft eigendom via gpio
  // maar we gaan de 'sense' (sense for low)activeren op de pin.
  // als pin laag gaat, geeft dit een rising edge op DETECT
  // de detect pulse genereert een int via PORT EVENT :
  NRF_GPIO->PIN_CNF[pinButton] &= ~GPIO_PIN_CNF_SENSE_Msk;
  NRF_GPIO->PIN_CNF[pinButton] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
  NRF_GPIOTE->INTENSET = 0x80000000;

  Serial.print("button level : "); Serial.println(digitalRead(pinButton));

  // todo voor gpiote irq GPIOTE_IRQHandler (gcc_startup_nrf51.S)
  NVIC_SetPriority(GPIOTE_IRQn, 15);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_EnableIRQ(GPIOTE_IRQn);  

} // setup

void printResetReason() {
  // read the register
  uint32_t resetReason = NRF_POWER->RESETREAS;

  Serial.print("reset by : ");

  if (resetReason & POWER_RESETREAS_DIF_Msk)
    Serial.print(" Debug interface");
  if (resetReason & POWER_RESETREAS_LPCOMP_Msk)
    Serial.print(" ANADETECT");
  if (resetReason & POWER_RESETREAS_OFF_Msk)
    Serial.print(" DETECT");
  if (resetReason & POWER_RESETREAS_LOCKUP_Msk)
    Serial.print(" LOCKUP");
  if (resetReason & POWER_RESETREAS_SREQ_Msk)
    Serial.print(" core");
  if (resetReason & POWER_RESETREAS_DOG_Msk)
    Serial.print(" watchdog");
  Serial.println();

  //reset the register
  NRF_POWER->RESETREAS = 0xFFFFFFFF;

} // printResetReason

uint32_t cpuSleepMillis;
uint8_t cpuSleepCountDown = 5;
uint32_t event = 0;
bool eventFlag = false;

void loop() {
  // testSystemOffAndWatchdog();

  if (eventFlag) {
    Serial.print("event happened : ");Serial.println(event,HEX);
    eventFlag = false;
    // er zit blijkbaar een debounce condensator op de button
    // want die blijft nog 1 à 2 seconden 0 lezen nadat de knop is gereleased
    // in die periode kan je geen events krijgen
    //while (digitalRead(pinButton) == 0) Serial.print('0');
  }


  if ((millis() - cpuSleepMillis) > 1000)
  {
     Serial.print("CPU sleep in "); Serial.print(cpuSleepCountDown); Serial.println(" seconds!");

    if (cpuSleepCountDown == 0) {
      // cpu goes to sleep
      __WFI();
      // continue when we wake up ..
      cpuSleepCountDown = 5;
    }
    
    cpuSleepCountDown--;
    cpuSleepMillis = millis();

  }  

  // temperature measurements
  float dieTemperature;
   if ((millis() - tempMeasMillis) > 3000) {
    dieTemperature = readTemperature();
    Serial.print("die temperature = "); Serial.print(dieTemperature); Serial.println("°C");
    tempMeasMillis = millis();
  }
} // loop

// godverdomme, 2de keer dat ik mij hieraan laat vangen!!
// don't forget extern "C" !
extern "C"{
void GPIOTE_IRQHandler(void)
{
  eventFlag = true;
  // events wissen -> dit neemt de int source weg!
  // hoe weet je nu welk event actief is? -> ze 1 voor 1 afgaan
  // maar in dit voorbeeld worden enkel IN[0] en PORTS geactiveerd
  if (NRF_GPIOTE->EVENTS_IN[0]) {
    event = 1;
    NRF_GPIOTE->EVENTS_IN[0] = 0;
  }
  if (NRF_GPIOTE->EVENTS_PORT) {
    event = 2;
    NRF_GPIOTE->EVENTS_PORT = 0;
  }

} // GPIOTE_IRQHandler

} // extern "C"

int countDownSystemOff = 20;
int countDownWatchdog = 5;
uint32_t countDownMillis;

void watchdogSetup()
{
  NRF_WDT->POWER = 0x1UL;
  NRF_WDT->RREN = 1; // is eigenlijk al de default
  NRF_WDT->CONFIG = 0; // voorlopig wdt pause while cpu sleeping
  NRF_WDT->CRV = 5*32768; // 5s
  NRF_WDT->TASKS_START = 1; // start the watchdog
} // watchdogSetup

void testSystemOffAndWatchdog()
{
  if ((millis() - countDownMillis) > 1000)
  {
    Serial.print("shutting down in ");
    Serial.print(countDownSystemOff); Serial.println(" seconds!");

    if (countDownSystemOff == 0) {
      // going to system off
      // enable wakeup from button low
      NRF_GPIO->PIN_CNF[pinButton] &= ~GPIO_PIN_CNF_SENSE_Msk;
      NRF_GPIO->PIN_CNF[pinButton] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);

      // enter system off
      NRF_POWER->SYSTEMOFF = 0x1UL;
      while(1); 
      // won't come here
    }
    // watchdog warning !
    Serial.print("watchdog reset in ");
    Serial.print(countDownWatchdog); Serial.println(" seconds!");

    countDownSystemOff--;
    countDownWatchdog--;
    countDownMillis = millis();
  }

  // feed the watchdog
  int button = digitalRead(pinButton);
  if (button == 0) {
    // feed as long as button is pressed
    // no need to check button release for now
    NRF_WDT->RR[0] = 0x6E524635; // dog likes this particular cookie
    countDownWatchdog = 5;
  }

} // testSystemOffAndWatchdog

float readTemperature () {
  // power register niet gedocumenteerd in refman??
  // stond in de praktijk al enable in on-mode
  // niet in SW gebeurd, dus is dit een default??
  // geen idee of dit altijd zo is.
  NRF_TEMP->POWER = 1; 

  // laten we eens meten hoe lang dit duurt
  uint32_t i =0;
  NRF_TEMP->EVENTS_DATARDY = 0; // clear outstanding events
  NRF_TEMP->TASKS_START = 1;
  while (NRF_TEMP->EVENTS_DATARDY == 0){i++;}

  //Serial.print("measurement took "); Serial.print(i); Serial.println(" cycles");
  // read temperature from NRF->TEMP
  return ((float) (NRF_TEMP->TEMP / 4.0));

} // readTemperature

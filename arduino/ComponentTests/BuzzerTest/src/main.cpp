/* basic b-watch hardware test
   press the button to toggle the vibrator
   display slightly dimmed because target is powered from the st-link
   optionally connect a serial-to-usb converter to catch serial output
*/

#include <Arduino.h>

int pinBuzzer = 7;
int pinButton = 4;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(pinBuzzer, OUTPUT);
  pinMode (pinButton, INPUT_PULLUP);
  digitalWrite (pinBuzzer,0); // buzzer uit
  Serial.begin(115200);
  Serial.println("hello world");
}
bool hasToggled = true;

void loop() {
  if (digitalRead(pinButton) == 0) {
    if (hasToggled) {
      digitalWrite(pinBuzzer, !digitalRead(pinBuzzer));
      Serial.println("toggle buzzer!");
      hasToggled = false;
    }
  }
  else {
      hasToggled = true;
  }
}

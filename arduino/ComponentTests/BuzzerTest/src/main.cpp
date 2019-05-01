
// druk op 't knoppeke en de vibrator gaat aan!
// licht gaat een beetje uit op het display 
// target wel gevoed via st-link 
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
// the loop function runs over and over again forever
void loop() {
  
  if (digitalRead(pinButton) == 0)
  {
    if (hasToggled) {
      digitalWrite(pinBuzzer, !digitalRead(pinBuzzer));
      Serial.println("toggle buzzer!");
      hasToggled = false;
    }
  }
  else {
      hasToggled = true;
  }
/*  
  digitalWrite(pinTX, HIGH);    // turn the LED off by making the voltage HIGH
  delay(1000);                       // wait for a second
  digitalWrite(pinTX, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  */
}

// compiles, maar upload lukt niet omdat er een firmware.hex ontbreekt
// dit heeft iets te maken met de combinatie van softdevice & userfirmware.hex die 
// slecht geconfiged is
// de firmware.elf werkt wel, maar de nrf51822.ld file plaatst die op een offset
// die rekening houdt met een softdevice (S110 of S130) afhankelijk van de targets.json
// als je de firmware.elf met openocd oplaadt en de softdevice zit al geflasht, 
// dan werkt het
// de huidige mbed-build omgeving merget de userfirmware.hex niet met een softdevice
// zoals dit met arduino framework wel gebeurt

#include "mbed.h"
//#include "SoftSerial.h"
// druk op 't knoppeke en de vibrator gaat aan!

//DigitalOut myled(LED1);
//DigitalOut myled(p30);
DigitalIn button(P0_4,PullUp);
DigitalOut buzzer(P0_7);


// default = 9600baud
Serial pc(P0_18,P0_17,115200); //tx=0.18, rx=0.17
//SoftSerial pc(p18,p17); -> werkt nie

bool hasToggled = true;

int main() {
    buzzer = 0; // buzzer uit
    pc.printf("button and buzzer!\n");
    while (1) {
        if (button == 0)
        {
            if (hasToggled) {
                buzzer = !buzzer;
                hasToggled = false;
                pc.printf("toggled!\n");
            }
        }
        else {
          hasToggled = true;
        }    
    } 
}
/*
    Global stuff
    SD card - works, 50
    GPS - works, RX to 17, TX to 16
    Lights - works, A1=Red, A2=Orange, A3=Green
    Piezo - works, 48
    Servo Motor - works, 6
    DC Motor - ???
    Sonar - works, Trig 3, Echo 4
    Line - works, 7
    Potetiometer - works, A0
    Button 1(Crash) - works, 8
    Button 2(closer to battery pack) - works, 9
    Button 3 - works, 10
*/

// DC Motor & Motor Module - L298N
#include <L298N.h>

// Create one motor instance
L298N motor(11, 12);

void setup() {
  // DC Motor & Motor Module - L298N
  motor.setSpeed(70);
  motor.forward();
}


void loop() {
}

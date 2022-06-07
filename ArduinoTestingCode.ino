/* Global stuff 
 *  SD card works, 50
 *  GPS works, RX to 17, TX to 16
 *  Lights works, A1=Red, A2=Orange, A3=Green
 *  Piezo - works, 48
 *  Servo Motor - 
 *  DC Motor - 
 *  Sonar - 
 *  Line - 
 *  Potetiometer - 
 *  Button 1(big) - 
 *  Button 2(closer to battery pack) - 
 *  Button 3 -
 *  
*/

// DC Motor & Motor Module - L298N
#include <L298N.h>

// Pin definition
const unsigned int IN3 = 22;
const unsigned int IN4 = 23;

// Create one motor instance
L298N motor(IN3, IN4);

void setup() {
  // DC Motor & Motor Module - L298N
    motor.setSpeed(70);
    motor.forward();
}


void loop(){
  
}

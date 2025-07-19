/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>
const int servoPins[] = {4,5,6,7,8,9,10,11};
const int numServos = sizeof(servoPins) / sizeof(servoPins[0]);

Servo servos[numServos];
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);

  for(int i=0; i<numServos;i++){
    servos[i].attach(servoPins[i]);
  }
  
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    for (int i=0;i<numServos;i++){
      servos[i].write(pos);             
    }
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    for (int i=0;i<numServos;i++){
      servos[i].write(pos);             
    }        
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

#include <Servo.h>

// Define pins
const int hallSensorPin = 12;  // Hall sensor input (D12)
const int servoPins[] = {3, 4, 5, 6, 8, 9, 10, 11};  // Servo outputs D3 to D11
const int numServos = sizeof(servoPins) / sizeof(servoPins[0]);

Servo servos[numServos];  // Create an array of Servo objects

void setup() {
  pinMode(hallSensorPin, INPUT);  // Hall sensor input
  Serial.begin(9600);

  // Attach all servos to their respective pins
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
  }
}

void loop() {
  int sensorState = digitalRead(hallSensorPin);
  Serial.println(sensorState);
  if (sensorState == LOW) {  // Magnet detected
    Serial.println("Magnet detected! Signal high");
    // activate slave signal on pin2 and pin7
    digitalWrite(2,HIGH);
    delayMicroseconds(1800);
    digitalWrite(2,LOW);
    delay(18);
    delayMicroseconds(200);

    digitalWrite(7,HIGH);
    delayMicroseconds(1800);
    digitalWrite(7,LOW);
    delay(18);
    delayMicroseconds(200); // fine-tune to hit ~ 20 ms total

  } else {
    Serial.println("No magnet. Signal low");
    // use master signal
    digitalWrite(2,HIGH)
    delayMicroseconds(1500);
    digitalWrite(2,LOW);
    delay(18);
    delayMicroseconds(500);

    digitalWrite(7,HIGH)
    delayMicroseconds(1500);
    digitalWrite(7,LOW);
    delay(18);
    delayMicroseconds(500);

    // Neutral 1500us PWM to all servos
    for (int i = 0; i < numServos; i++) {
      servos[i].writeMicroseconds(1500);    
    }
  }

  delay(20);  // 50Hz servo update rate (~20ms)
}

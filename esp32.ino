#include <ESP32Servo.h>

Servo gripperServo1;
Servo gripperServo2;

void setup() {
  Serial.begin(115200);  // ESP32 typically uses 115200 baud rate
  
  // Allow allocation of all timers for servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  gripperServo1.attach(18);  // GPIO 18 (good PWM pin for ESP32)
  gripperServo1.write(35);   // Start in closed position
  
  gripperServo2.attach(19);  // GPIO 19 (good PWM pin for ESP32)
  gripperServo2.write(95);   // Start in closed position

  Serial.println("ESP32 Servo Gripper Control");
  Serial.println("Type 'J' to open, 'K' to close the gripper.");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'J' || command == 'j') {
      gripperServo1.write(35);     // Open gripper
      Serial.println("Gripper opened.");
    } 
    else if (command == 'K' || command == 'k') {
      gripperServo1.write(90);    // Close gripper
      Serial.println("Gripper closed.");
    }
    else if (command == 'B' || command == 'B') {
      gripperServo2.write(110);    // Close gripper
      Serial.println("Gripper closed.");
    }
    else if (command == 'V' || command == 'v') {
      gripperServo2.write(95);    // Close gripper
      Serial.println("Gripper closed.");
    }
  }
}
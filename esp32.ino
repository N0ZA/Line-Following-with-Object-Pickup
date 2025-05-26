#include <ESP32Servo.h>

Servo gripperServo;

void setup() {
  Serial.begin(115200);  // ESP32 typically uses 115200 baud rate
  
  // Allow allocation of all timers for servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  gripperServo.attach(18);  // GPIO 18 (good PWM pin for ESP32)
  gripperServo.write(35);   // Start in closed position
  
  Serial.println("ESP32 Servo Gripper Control");
  Serial.println("Type 'J' to open, 'K' to close the gripper.");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'J' || command == 'j') {
      gripperServo.write(35);     // Open gripper
      Serial.println("Gripper opened.");
    } 
    else if (command == 'K' || command == 'k') {
      gripperServo.write(90);    // Close gripper
      Serial.println("Gripper closed.");
    }
  }
}
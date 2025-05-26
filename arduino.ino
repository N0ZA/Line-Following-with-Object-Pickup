// 5 Channel IR Sensor Connection
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4

// Cytron MDDS30 Motor control pins
#define M1_PWM 9   // Motor 1 Speed (PWM)
#define M1_DIR 5   // Motor 1 Direction
#define M2_PWM 6   // Motor 2 Speed (PWM)
#define M2_DIR 7   // Motor 2 Direction

// PID Constants
float Kp = 6;
float Ki = 0.01;
float Kd = 3;

// PID Variables
float error = 0, lastError = 0;
float P = 0, I = 0, D = 0;
float PID_value = 0;

// Motor speeds
int baseSpeed = 100;
int leftMotorSpeed = 0, rightMotorSpeed = 0;

int TURN_DELAY_LEFT = 1100;   // milliseconds
int TURN_DELAY_RIGHT = 1100;

void setup() {
  Serial.begin(9600);
  Serial.println("Line Follower Robot - Cytron MDDS30 PID");

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  stopMotors();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "F") {
      executeForwardCommand();
      Serial.println("DONE");
    } else if (command == "L") {
      turnLeft();
      Serial.println("DONE");
    } else if (command == "R") {
      turnRight();
      Serial.println("DONE");
    } else if (command == "S") {
      stopMotors();
      Serial.println("DONE");
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }
}

void executeForwardCommand() {
  bool junctionDetected = false;
  error = 0; lastError = 0; I = 0; D = 0; // Reset PID variables

  while (!junctionDetected) {
    int s1 = digitalRead(ir1);
    int s2 = digitalRead(ir2);
    int s3 = digitalRead(ir3);
    int s4 = digitalRead(ir4);
    int s5 = digitalRead(ir5);

    // Check junction condition
    if ((s3 == 0 && s4 == 0 && s5 == 0) || 
        (s3 == 1 && s4 == 0 && s5 == 0) || 
        (s1 == 0 && s2 == 0 && s3 == 0) || 
        (s1 == 0 && s2 == 0 && s3 == 1) || 
        (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)) {
      junctionDetected = true;
      delay(500);
      stopMotors();
      return;
    }

    // Calculate PID
    int weights[5] = {-2, -1, 0, 1, 2};
    int sensors[5] = {s1, s2, s3, s4, s5};
    int sum = 0, activeSensors = 0;

    for (int i = 0; i < 5; i++) {
      if (sensors[i] == 0) {
        sum += weights[i];
        activeSensors++;
      }
    }

    if (activeSensors != 0) error = (float)sum / activeSensors;
    else error = lastError;

    P = error;
    I += error;
    D = error - lastError;
    PID_value = (Kp * P) + (Ki * I) + (Kd * D);
    lastError = error;

    leftMotorSpeed = baseSpeed + PID_value;
    rightMotorSpeed = baseSpeed - PID_value;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
    delay(10);
  }
}

// Motor control functions remain the same
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, rightSpeed);
  analogWrite(M2_PWM, leftSpeed);
}

void turnLeft() {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, 180);
  analogWrite(M2_PWM, 0);
  delay(TURN_DELAY_LEFT);
  stopMotors();
}

void turnRight() {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 180);
  delay(TURN_DELAY_RIGHT);
  stopMotors();
}

void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
}
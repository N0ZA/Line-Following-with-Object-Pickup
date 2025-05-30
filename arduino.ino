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
float Kp = 20;
float Ki = 0.005;
float Kd = 1.5;

// PID Variables
float error = 0, lastError = 0;
float P = 0, I = 0, D = 0;
float PID_value = 0;

// Motor speeds
int baseSpeed = 100;
int leftMotorSpeed = 0, rightMotorSpeed = 0;

int TURN_DELAY_LEFT = 1000;   // milliseconds
int TURN_DELAY_RIGHT = 1000;

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
    } else if (command == "U") {
      turnUTurn();
      Serial.println("DONE");
    } else if (command == "S") {
      stopMotors();
    } else if (command == "C") {
      complexMovement();
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
      delay(200);
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


void complexMovement() {
  // Execute the complex movement 5 times
  for (int iteration = 1; iteration <= 5; iteration++) {
    Serial.print("Complex Movement - Iteration: ");
    Serial.println(iteration);
    
    // Step 1: Move forward and stop 300ms after detecting a line
    error = 0; lastError = 0; I = 0; D = 0;
    bool lineDetected = false;

    while (!lineDetected) {
      int s1 = digitalRead(ir1);
      int s2 = digitalRead(ir2);
      int s3 = digitalRead(ir3);
      int s4 = digitalRead(ir4);
      int s5 = digitalRead(ir5);

      // Junction/line condition
      if ((s3 == 0 && s4 == 0 && s5 == 0) || 
          (s3 == 1 && s4 == 0 && s5 == 0) || 
          (s1 == 0 && s2 == 0 && s3 == 0) || 
          (s1 == 0 && s2 == 0 && s3 == 1) || 
          (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)) {
        lineDetected = true;
        delay(300);
        stopMotors();
        break;
      }

      // PID forward logic
      int weights[5] = {-2, -1, 0, 1, 2};
      int sensors[5] = {s1, s2, s3, s4, s5};
      int sum = 0, activeSensors = 0;

      for (int i = 0; i < 5; i++) {
        if (sensors[i] == 0) {
          sum += weights[i];
          activeSensors++;
        }
      }

      error = (activeSensors != 0) ? (float)sum / activeSensors : lastError;

      P = error;
      I += error;
      D = error - lastError;
      PID_value = (Kp * P) + (Ki * I) + (Kd * D);
      lastError = error;

      leftMotorSpeed = constrain(baseSpeed + PID_value, 0, 255);
      rightMotorSpeed = constrain(baseSpeed - PID_value, 0, 255);

      setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
      delay(10);
    }

    // Step 2: Rotate right (M1 forward, M2 off) until IR3 detects a line, start sensing IR3 after 200ms
    digitalWrite(M2_DIR, HIGH);
    digitalWrite(M1_DIR, LOW);
    analogWrite(M2_PWM, 150);
    analogWrite(M1_PWM, 150);

    delay(300);  // Delay before IR3 sensing starts

    while (digitalRead(ir3) == 1) {
      // waiting to detect the line
      delay(10);
    }
    stopMotors();

    // Step 3: Stay in place for 2 seconds
    delay(4000);

    // Step 4: Rotate left (M1 off, M2 forward) until IR3 detects a line
    // Skip this step in the last iteration (iteration 5)
    if (iteration < 5) {
      digitalWrite(M2_DIR, LOW);
      digitalWrite(M1_DIR, HIGH);
      analogWrite(M2_PWM, 150);
      analogWrite(M1_PWM, 150);

      delay(300);

      while (digitalRead(ir3) == 1) {
        delay(10);
      }
      stopMotors();
    } else {
      Serial.println("Skipping Step 4 in final iteration");
    }
    
    // Small delay between iterations
    delay(100);
  }
  
  Serial.println("Complex Movement completed all 5 iterations");
}

void turnLeft() {
  // Step 1: Initiate the turn to leave the line
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, 150);
  analogWrite(M2_PWM, 0);
  
  // Step 2: Wait until the center sensor leaves the line (s3 == 1)
  while (digitalRead(ir3) == 0);  // Wait to leave the line
  delay(400);
  // Step 3: Now re-align using s3 feedback
  while (digitalRead(ir3) == 1) {
    int s3 = digitalRead(ir3);
    float error = 1.0; // s3 still hasn't hit line
    float slowDown = Kp * error;

    int rightSpeed = constrain(150 - slowDown, 80, 150);
    analogWrite(M1_PWM, rightSpeed);
    analogWrite(M2_PWM, 0);
    delay(10);
  }

  stopMotors();
}

void turnRight() {
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 150);

  while (digitalRead(ir3) == 0);  // Wait to leave the line
  delay(400);
  while (digitalRead(ir3) == 1) {
    int s3 = digitalRead(ir3);
    float error = 1.0;
    float slowDown = Kp * error;

    int leftSpeed = constrain(150 - slowDown, 80, 150);
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, leftSpeed);
    delay(10);
  }

  stopMotors();
}



void turnUTurn() {
  // Step 1: Initiate the U-turn to leave the line
  // M1 HIGH, M2 LOW for U-turn (spinning in place or tight turn)
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, LOW);  // Reverse direction for M2
  analogWrite(M1_PWM, 150);
  analogWrite(M2_PWM, 150);   // Same PWM for both motors
  
  // Step 2: Wait until the center sensor leaves the line (s3 == 1)
  while (digitalRead(ir3) == 0);  // Wait to leave the line
  delay(700);  // Longer delay for U-turn to complete more rotation
  
  // Step 3: Continue turning until we find the line again using PID control
  while (digitalRead(ir3) == 1) {
    int s3 = digitalRead(ir3);
    float error = 1.0; // s3 still hasn't hit line
    float slowDown = Kp * error;

    int motorSpeed = constrain(150 - slowDown, 80, 150);
    digitalWrite(M1_DIR, HIGH);
    digitalWrite(M2_DIR, LOW);
    analogWrite(M1_PWM, motorSpeed);
    analogWrite(M2_PWM, motorSpeed);
    delay(10);
  }

  stopMotors();
}

void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
}
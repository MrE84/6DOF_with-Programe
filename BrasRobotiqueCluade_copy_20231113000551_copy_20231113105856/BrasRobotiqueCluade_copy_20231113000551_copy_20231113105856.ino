#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD display
LiquidCrystal_I2C lcd(0x27,16,4);

// Initialize servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Initialize Bluetooth module
SoftwareSerial bt1(2,3); /* (Rx,Tx) */

// Define servo motor parameters
const int MIN_PULSE_WIDTH = 500;
const int MAX_PULSE_WIDTH = 2500;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50;
const float FREQUENCY_SCALE = (float)FREQUENCY * 4096 / 1000000;

// Function to calculate pulse width for a given angle
int pulseWidth(int angle) {
    int pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    int analog_value = int(pulse_wide * FREQUENCY_SCALE);
    Serial.println(analog_value);
    return analog_value;
}

// Define buttons and potentiometers
int leftButton = 7;
int rightButton = 8;
int potentiometer1 = A0;
int potentiometer2 = A1;
int potentiometer3 = A2;
int potentiometer4 = A3;
int potentiometer5 = A4;
int potentiometer6 = A5;

// Define record section
int buttonStartRecord = 9;
int buttonStopRecord = 10;
int buttonPlayRecord = 11;
int ledStopRecord = 12;
int ledStartRecord = 13;

bool isRecord = false;
bool isPlay = false;
int indexRecord = 0;

// Define maximum moves and servos
const int moveCount = 10;
const int servoNumber = 6;


int movesServos[moveCount][servoNumber]; //max of 10 moves

void setup() {
    Serial.begin(9600);
    bt1.begin(9600);

    // Initialize LCD display
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.begin(16, 4);      // set up the LCD's number of columns and rows
    lcd.setCursor(0, 0);
    lcd.print("2023 GDIP F1 :");
    lcd.setCursor(0, 1);
    lcd.print("6 DOF ROBOTIC ARM ");
    lcd.setCursor(0, 2);
    lcd.print("PROTOTYPE 1");

    // Initialize servo driver
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

    // Move servo motors to initial positions
    pwm.setPWM(1,0,pulseWidth(125));
    pwm.setPWM(2,0,pulseWidth(180));
    pwm.setPWM(3,0,pulseWidth(190));
    pwm.setPWM(4,0,pulseWidth(180));
    pwm.setPWM(5,0,pulseWidth(180));
    pwm.setPWM(6,0,pulseWidth(80));

    pinMode(leftButton, INPUT);
    pinMode(rightButton, INPUT);

  //Record section
    pinMode(buttonStartRecord, INPUT);
    pinMode(buttonStopRecord, INPUT);
    pinMode(buttonPlayRecord, INPUT);

    pinMode(ledStartRecord, OUTPUT);
    pinMode(ledStopRecord, OUTPUT);


}

void loop() {

    String command = "";

    // Read commands from serial port or Bluetooth module
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');
        command.trim(); // Trim any newline or carriage return characters
        Serial.println("Received: " + command); // Echo the command back to the serial monitor
    } else if (bt1.available()) {
        command = bt1.readStringUntil('\n');
        command.trim(); // Trim any newline or carriage return characters
        Serial.println("Received via BT: " + command); // Echo the command from BT
    }

    // Execute command if available
    if (command != "") {
        executeCommand(command);
    }
  
  if (!isPlay) {
    int valPotentiometer1 = analogRead(potentiometer1);
    int valPotentiometer2 = analogRead(potentiometer2);
    int valPotentiometer3 = analogRead(potentiometer3);
    int valPotentiometer4 = analogRead(potentiometer4);
    int valPotentiometer5 = analogRead(potentiometer5);
    int valPotentiometer6 = analogRead(potentiometer6);

    int valServo1 = map(valPotentiometer1, 0, 1023, 10, 170);
    int valServo2 = map(valPotentiometer2, 0, 1023, 10, 170);
    int valServo3 = map(valPotentiometer3, 0, 1023, 10, 170);
    int valServo4 = map(valPotentiometer4, 0, 1023, 10, 170);
    int valServo5 = map(valPotentiometer5, 0, 1023, 10, 170);
    int valServo6 = map(valPotentiometer5, 0, 1023, 10, 170);

   
    pwm.setPWM(1, 0, pulseWidth(valServo1));
    pwm.setPWM(1, 0, pulseWidth(valServo1));
    pwm.setPWM(1, 0, pulseWidth(valServo1));
    pwm.setPWM(1, 0, pulseWidth(valServo1));
    pwm.setPWM(1, 0, pulseWidth(valServo1));
    pwm.setPWM(1, 0, pulseWidth(valServo1));
    pwm.setPWM(1, 0, pulseWidth(valServo1));
    

    int leftButtonState = digitalRead(leftButton);
    int rightButtonState = digitalRead(rightButton);

    if (leftButtonState == HIGH && rightButtonState == HIGH) {
      clawOpen();
    } else {
      if (leftButtonState == HIGH) {
        clawOpen();
      }
      if (rightButtonState == HIGH) {
        clawClose();
      }
    }
  }
  
  record();

  delay(5);
}




void record() {
  int buttonStartRecordState =  digitalRead(buttonStartRecord);
  int buttonRecordMoveState =  digitalRead(buttonStopRecord);
  int buttonPlayRecordState =  digitalRead(buttonPlayRecord);

  if (buttonStartRecordState == HIGH) {
    if (!isRecord) {
      isRecord = true;
      digitalWrite(ledStopRecord, LOW);
      digitalWrite(ledStartRecord, HIGH);
      Serial.println("----START Record----");
      indexRecord = 0;
      memset(movesServos, 0, sizeof(movesServos));
    } else {
      isRecord = false;
      digitalWrite(ledStopRecord, LOW);
      digitalWrite(ledStartRecord, LOW); 
    }
    delay(300);
  }

  if (buttonRecordMoveState == HIGH && isRecord) {
    digitalWrite(ledStopRecord, LOW);
    digitalWrite(ledStartRecord, LOW);
    recordMove();
    delay(300);
    digitalWrite(ledStopRecord, LOW);
    digitalWrite(ledStartRecord, HIGH);

  } else if (buttonRecordMoveState == HIGH && isPlay) {
    isPlay = false;
    digitalWrite(ledStopRecord, LOW);
    digitalWrite(ledStartRecord, LOW);
  }
  if (buttonPlayRecordState == HIGH) {

    digitalWrite(ledStopRecord, HIGH);
    digitalWrite(ledStartRecord, LOW);
    isPlay = true;
  }

  if (isPlay) {
    Serial.println("Play");
    playRecord();

    delay(300);
  }
}

void recordMove() {
  if (indexRecord < moveCount) {
    int servoRead1 =  pwm.getPWM(1);
    int servoRead2 =  pwm.getPWM(2);
    int servoRead3 =  pwm.getPWM(3);
    int servoRead4 =  pwm.getPWM(4);
    int servoRead5 =  pwm.getPWM(5);
    int servoRead6 =  pwm.getPWM(6);
    

    movesServos[indexRecord][0] = servoRead1;
    movesServos[indexRecord][1] = servoRead2;
    movesServos[indexRecord][2] = servoRead3;
    movesServos[indexRecord][3] = servoRead4;
    movesServos[indexRecord][4] = servoRead5;
    movesServos[indexRecord][5] = servoRead6;

   

    indexRecord++;
  } else {
    Serial.println("Max array");
  }

}


void playRecord() {

  for(int i = 0; i < indexRecord; i++) {

    pwm.setPWM(1, 0, movesServos[i][0]);
    // Set channels to stored widths  

    if(movesServos[i][4] == 1) {
      clawOpen();
    } else {
      clawClose();
    }
  }
  
}

void moves() {

  for (int i = 0; i < indexRecord - 1; i++) {

    for (int j = 0; j < servoNumber; j++) {

      if (j >= 3) {
        // Gripper servo 
        if (movesServos[i + 1][j] == 1) {
          clawOpen();
        } else {
          clawClose();
        }

      } else {
        // Joint servos
        
        int startWidth = movesServos[i][j]; 
        int endWidth = movesServos[i + 1][j];

        setServoPosition(j+1, startWidth, endWidth);  
      }
    }
  }
}





void goToStartPosition() {

  for (int j = 0; j < 5; j++) {
    
    if (j >= 3) {
      if (movesServos[0][j] == 1) {
        clawOpen();
      } else {
        clawClose();
      }
      
    } else {
      int startWidth = pwm.getPWM(j+1);
      int endWidth = movesServos[0][j];
      setServoPosition(j+1, startWidth, endWidth); 
    }
  }

}


void setServoPosition(int channel, int startWidth, int endWidth) {

  int step;
  
  if (startWidth > endWidth) {
    // Move servo clockwise
    step = -1;  
  } else {
    // Move servo counter-clockwise 
    step = 1;
  }

  // Update pulse width from start to end
  for(int i = startWidth; i != endWidth; i += step) {
    pwm.setPWM(channel, 0, i);
    delay(30);
  }

}

void clawOpen() {
  
  pwm.setPWM(6,0,pulseWidth(80));
}

void clawClose() {
pwm.setPWM(6,0,pulseWidth(120));
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
void MoveToPick() {
    // Define the sequence of angles for each servo in the pick-up motion
    int angles[10][servoNumber] = {
        // HIP, WAIST, SHOULDER, ELBOW, WRIST, CLAW
        {125, 180, 190, 180, 190, 80},    // Initial position
        {150, 110, 100, 190, 90, 80}, // Move to above the object
        {150, 50, 180, 150, 180, 80}, // Lower towards the object
        {150, 50, 180, 150, 180, 140},// Close claw to grab the object
        {150, 90, 100, 150, 90, 140},// Lift the object
        {125, 180, 190, 180, 190, 120},    // Return to initial position with object
        {150, 110, 100, 190, 90, 80}, // Move the object
        {150, 40, 180, 150, 180, 80}, // Lower  the object
        {150, 40, 180, 150, 180, 80}, // OPEN claw to RELEASE the object
        {125, 180, 190, 180, 190, 120}    // Return to initial position with object
    };

    // Execute the sequence
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < servoNumber; j++) {
            pwm.setPWM(j + 1, 0, pulseWidth(angles[i][j]));
        }
        delay(2000); // Wait for 1 second between each step for smooth movement
    }

    // Optionally, open the claw to release the object at the end
    // pwm.setPWM(6, 0, pulseWidth(0)); // Open the claw
}

void executeCommand(String command) {

   command.trim(); // Trim whitespace
    command.toUpperCase(); // Convert to upper case for case-insensitive comparison

    // Define servo commands
    struct ServoCommand {
        const char* name;
        int channel;
    };

    ServoCommand commands[] = {
        {"HIP ", 1},
        {"WAIST ", 2},
        {"SHOULDER ", 3},
        {"ELBOW ", 4},
        {"WRIST ", 5},
        {"CLAW ", 6}
    };

    // Check for the MOVE_TO_PICK command
    if (command == "MOVE_TO_PICK") {
        MoveToPick();
        Serial.println("Executing MoveToPick sequence");
        return;
    }

    // Parse and execute other servo commands
    for (const auto& cmd : commands) {
        if (command.startsWith(cmd.name)) {
            int angle = command.substring(strlen(cmd.name)).toInt();
            pwm.setPWM(cmd.channel, 0, pulseWidth(angle));
            Serial.println(String(cmd.name) + "moved to " + String(angle) + " degrees");
            return;
        }
    }

    // Unknown command
    Serial.println("Unknown command: " + command);
}

#include <Arduino.h>
#include <Freenove_WS2812_Lib_for_ESP32.h>
#include <functional>

/*  "We do these things not because they are easy, but because we thought they were GOING to be easy."  */

//End of travel can be detected because the encoder will stop recieving updates.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// General Pins
#define P_VREF 4      // 3.3V Reference voltage

// Encoder Pins
#define P_ENCA 22     // Encoder A input
#define P_ENCB 23     // Encoder B input

// Driver Pins
#define P_DSPD 21     // H-Bridge driver PWM speed output
#define P_DM1 19      // H-Bridge driver IN1 output
#define P_DM2 18      // H-Bridge driver IN2 output

// Control Pins
#define P_C_UP 26     // UP button input
#define P_C_DOWN 25   // DOWN button input
#define P_C_HOMES 33  // HOMES button input
#define P_C_F1 32     // Function 1 button input
#define P_C_F2 35     // Function 2 button input
#define P_C_F3 34     // Function 3 button input
#define P_CLED 13     // Control LED data output

// Led indices
#define LED_UP 0      // Up button LED index
#define LED_DOWN 1    // Down button LED index
#define LED_HOMES 2   // Homes button LED index
#define LED_F1 3      // Function 1 button LED index
#define LED_F2 4      // Function 2 button LED index
#define LED_F3 5      // Function 3 button LED index

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS   //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// What is the period of the main process loop (buttons, lights, PID compute,etc.) in ms.
const int PROCESS_PERIOD = 20;

// What is the period of the encoder scanning loop in µs.
const int ENC_SAMPLE_PERIOD = 1000;

// What is the minimum PWM speed (0-255) that the motor will operate at. (Values lower will be replaced by this value)
const int MIN_SPEED = 30;

// What is the maximum operating PWM speed (0-255) during normal operation.  Note: internet advises using less than the 95% maximum possible speed.
const int MAX_SPEED = 200;

// What is the maximum operating PWM speed (0-255) during homing operations. Should be kept low to prevent blowing fuses.
const int MAX_SPEED_HOMING = 35;

// Maximum allowed acceleration in PWM units per PROCESS_PERIOD.
const float MAX_ACCEL_PER_SECOND = 250.0F;

// Maximum allowable windup in the integral term of the PID loop.
const float MAX_INT_WINDUP = 5.0F;

// Position in encoder ticks that is the lower end of travel.
const int EOT_BOT = 0;

// Position in encoder ticks that is the upper end of travel.
const int EOT_TOP = 4100;

// Distance in encoder ticks from EOT_TOP where you are prevented from further travel towards the EOT.
const int EOT_BUFFER_TOP = 100;

// Distance in encoder ticks from EOT_BOTTOM where you are prevented from further travel towards the EOT.
const int EOT_BUFFER_BOT = 200;

// How many ms must an active motor not cause an encoder tick before it's considered a stall.
const int STALL_CUTOUT_MS = 250;

// How many ms must a button be pressed to be considered a tap.
const int BUTTON_TAP_THRESHHOLD = 250;

// How many ms must the button NOT be pressed to be considered idle.
const int BUTTON_IDLE_THRESHHOLD = 500;

int COL_DEFAULT[] = {40,40,40};
int COL_LOCKED[] = {255,0,0};
int COL_CONT_MOVE[] = {0,255,0};
int COL_HOLD_MOVE[] = {0,0,255};
int COL_UNAVAILABLE[] = {255,90,0};
int COL_SPECIAL[] = {80,0,255};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES  CLASSES    //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PIDController {
public:

  float setpoint;
  float integral;
  float prevError;
  float kp, ki, kd;

  PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), setpoint(0) {
    integral = 0;
    prevError = 0;
  }

  // Compute PID control output
  float compute(float currentValue) {
    // Calculate error
    float error = setpoint - currentValue;

    // Update integral term
    integral += error;

    // Calculate derivative term
    float derivative = error - prevError;

    // PID control output
    float outP = kp * error;
    float outI = min(max(-MAX_INT_WINDUP,ki * integral),MAX_INT_WINDUP);
    float outD = kd * derivative;
    float output = outP + outI + outD;

    // Serial.print("    targ: ");
    // Serial.print(setpoint);
    // Serial.print("    curr: ");
    // Serial.print(currentValue);
    // Serial.print("     err: ");
    // Serial.print(error);
    // Serial.print("    outP: ");
    // Serial.print(outP);
    // Serial.print("    outI: ");
    // Serial.print(outI);
    // Serial.print("    outD: ");
    // Serial.print(outD);
    // Serial.print("     out: ");
    // Serial.print(output);
    // Serial.println();

    // Save current error for the next iteration
    prevError = error;

    return output;
  }
  
  // Modify setpoint
  void changeSetpoint(float newSetpoint) {
    setpoint = newSetpoint;
  }

  // Reset PID Controller
  void reset() {
    integral = 0;
    prevError = 0;
  }
};

class MotorController {
public:

  int dir = 0;
  int mag = 0;

  // Constructor
  MotorController(int pinPWM, int pinM1, int pinM2) : pinPWM(pinPWM), pinM1(pinM1), pinM2(pinM2) {}

  void setSpeed(int in){
    if (in > 0){
      dir = 1;
    } else if (in < 0){
      dir = -1;
    } else {
      dir = 0;
    }
    
    if (dir == 0) {
      mag = 0;
    } else {
      mag = min(max(MIN_SPEED,abs(in)),MAX_SPEED);
    }
  }

  void apply(){
    if ((dir!=oldDir)||(mag!=oldMag)){
      analogWrite(pinPWM,mag);
      // Direction 1
      if(dir == -1){
        digitalWrite(pinM1,HIGH);
        digitalWrite(pinM2,LOW);
      }
      // Direction 2
      else if (dir == 1){
        digitalWrite(pinM1,LOW);
        digitalWrite(pinM2,HIGH);
      }
      // Locked
      else if (dir == 0){
        digitalWrite(pinM1,LOW);
        digitalWrite(pinM2,LOW);
      } else {
      // Float
        digitalWrite(pinM1,HIGH);
        digitalWrite(pinM2,HIGH);
      }
      oldMag = mag;
      oldDir = dir;
    }
  }

private:
  int pinPWM;
  int pinM1;
  int pinM2;
  int oldMag = 0;
  int oldDir = 0;
};

class Button {
public:
  bool isPressed;
  
  // Constructor
  Button(uint8_t pin, std::function<void()> func1, std::function<void()> func2, std::function<void()> func3, std::function<void(int)> func4)
  : buttonPin(pin), functionHold(func1), functionHoldRelease(func2), functionPressInstant(func3), functionMultiTap(func4) {
    isPressed = 0;
    timeSincePress = 0;
    timeSinceRelease = 0;
    isHeld = false;
    multiTapNotSent = true;
  }

  // Check the state of the button
  void scan() {

    // Press event
    if (!isPressed && digitalRead(buttonPin)) {
      isPressed = true;
      timeSincePress = 0;
      functionPressInstant();
    }

    // Release event
    else if (isPressed && !digitalRead(buttonPin)) {
      isPressed = false;
      multiTapNotSent = true;
      timeSinceRelease = 0;

      if (timeSincePress < BUTTON_TAP_THRESHHOLD) {
        timesTapped++;
      }

      if (isHeld) {
        isHeld = false;
        functionHoldRelease();
      }
    }

    // Held
    else if (isPressed && digitalRead(buttonPin)) {
      if (timeSincePress < 10000) {
        timeSincePress += PROCESS_PERIOD;
      }

      if (timeSincePress > BUTTON_TAP_THRESHHOLD && !isHeld) {
        functionHold();
        isHeld = true;
      }
    }

    // Idle
    else if (!isPressed && !digitalRead(buttonPin)) {
      if (timeSinceRelease < 10000) {
        timeSinceRelease += PROCESS_PERIOD;
      }
      
      if (timeSinceRelease > BUTTON_IDLE_THRESHHOLD && multiTapNotSent) {
        functionMultiTap(timesTapped);
        timesTapped = 0;
        multiTapNotSent = false;
      }
    }
  }

private:
  uint8_t buttonPin;
  uint32_t timeSinceRelease;
  uint32_t timeSincePress;
  uint8_t timesTapped;
  bool isHeld;
  bool multiTapNotSent;

  std::function<void()> functionHold;
  std::function<void()> functionHoldRelease;
  std::function<void()> functionPressInstant;
  std::function<void(int)> functionMultiTap;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES  VARIABLES   //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//===oOo==={   General Purpose    }===oOo===//

byte opType = 0; // 0 = motor disabled; 1 = velocity PID; 2 = position PID; 3 = ramping down
bool isHomed = false;

//===oOo==={       Encoder        }===oOo===//
// Encoder logic
//volatile long curEncTime = 0;
//volatile long lastEncTime = 0;
long long loopTime = 0;
uint8_t encState = 0;
uint8_t prevEncState = 0;

//===oOo==={    Motionstates     }===oOo===//

int encPos = 0;
float rotWorm = 0.0f;
float rotAxle = 0.0f;
float zPos = 0.0f;
int rotRawLast = 0;
float MAX_ACCEL_PER_PERIOD = (MAX_ACCEL_PER_SECOND/1000)*PROCESS_PERIOD;

// Conversion factors
const float ratioEncToWorm = .25F;
const float ratioWormToAxle = 0.0256410256F;
const float ratioAxleToZ = 19.685F; //in millimeters

//===oOo==={   PID Control Loop   }===oOo===//

int targetSpeed = 0;
MotorController motor(P_DSPD,P_DM1,P_DM2);

// PID Control Parameters
const float KPpos = .4;
const float KIpos = 0.000;
const float KDpos = 16;

const float KPvel = 200.0F;
const float KIvel = 0.0F;
const float KDvel = 0.0F;

PIDController PIDpos(KPpos,KIpos,KDpos);
PIDController PIDvel(KPvel,KIvel,KDvel);

//===oOo==={   Buttons and LEDs   }===oOo===//

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(6, P_CLED, 0);

bool panelLocked = false;
bool continuousUp = false;
bool continuousDown = false;

// Stall Detection
uint32_t msStalled = 0;
bool overrideEnabled = false;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS   //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function Declarations //
void checkEncoder();
void standardProcess();

void changeOpType(int);
void changeLEDcolor(int,int[]);
void changeLEDcolorAll(int[]);

void buttonUpHold();
void buttonUpHoldRelease();
void buttonUpPressInstant();
void buttonUpMultiTap(int);

void buttonDownHold();
void buttonDownHoldRelease();
void buttonDownPressInstant();
void buttonDownMultiTap(int);

void toggleLock();
void checkForStall();

void runToBottom();
void runToTop();

/*=============================================================================================================================================================*/

Button buttonUp(P_C_UP,buttonUpHold, buttonUpHoldRelease, buttonUpPressInstant, buttonUpMultiTap);
Button buttonDown(P_C_DOWN,buttonDownHold, buttonDownHoldRelease, buttonDownPressInstant, buttonDownMultiTap);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);

  pinMode(P_VREF, OUTPUT);
  digitalWrite(P_VREF,HIGH);

  pinMode(P_ENCA,INPUT);
  pinMode(P_ENCB,INPUT);

  pinMode(P_DSPD,OUTPUT);
  pinMode(P_DM1,OUTPUT);
  pinMode(P_DM2,OUTPUT);

  pinMode(P_C_UP,INPUT);
  pinMode(P_C_DOWN,INPUT);
  pinMode(P_C_HOMES,INPUT);
  pinMode(P_C_F1,INPUT);
  pinMode(P_C_F2,INPUT);
  pinMode(P_C_F3,INPUT);
  pinMode(P_CLED,OUTPUT);

  strip.begin();
  strip.setAllLedsColor(100,100,100);

  Serial.print("Max accel (units per period) is ");Serial.print(MAX_ACCEL_PER_PERIOD);Serial.println(" u/P.");
  Serial.println("Begin.");
}

/*=============================================================================================================================================================*/

unsigned long startTimeEnc = micros();
unsigned long startTimeStd = millis();

void loop() {
  
  //~2us completion time
  if(micros() - startTimeEnc > ENC_SAMPLE_PERIOD) {
    checkEncoder();
    startTimeEnc = micros();
  }
  
  //~32us completion time, but LEDs break if run too quickly (1ms is too fast, 20ms is adequate)
  if(millis() - startTimeStd > PROCESS_PERIOD) {
    //Serial.print(encState);Serial.print(" ");Serial.print(prevEncState);Serial.print(" ");Serial.print(totalState);Serial.print(" ");Serial.println(encPos);
    standardProcess();
    startTimeStd = millis();
  }
}

void checkEncoder() {
  prevEncState = encState;
  encState = (digitalRead(P_ENCA) << 1) + digitalRead(P_ENCB);
  
  //state takes the form of 4 bit number: [PIN A STATE] [PIN B STATE] [DID A CHANGE] [DID B CHANGE]
  switch ((encState<<2)+(encState^prevEncState)) {
    case 0:   // 0000; AB = 00; No Change
    case 4:   // 0100; AB = 01; No Change
    case 8:   // 1000; AB = 10; No Change
    case 12:  // 1100; AB = 11; No Change
      break;
    case 1:   // 0001; AB = 00; Changed from 01; Up
    case 6:   // 0110; AB = 01; Changed from 11; Up
    case 10:  // 1010; AB = 10; Changed from 00; Up
    case 13:  // 1101; AB = 11; Changed from 10; Up
      encPos++;
      break;
    case 2:   // 0010; AB = 00; Changed from 10; Down
    case 5:   // 0101; AB = 01; Changed from 00; Down
    case 9:   // 1001; AB = 10; Changed from 11; Down
    case 14:  // 1110; AB = 11; Changed from 01; Down
      encPos--;
      break;
    case 3:   // 0011; AB = 00; Changed from 11 (IMPOSSIBLE)
    case 7:   // 0111; AB = 01; Changed from 10 (IMPOSSIBLE)
    case 11:  // 1011; AB = 10; Changed from 01 (IMPOSSIBLE)
    case 15:  // 1111; AB = 11; Changed from 00 (IMPOSSIBLE)
      Serial.println("Impossible Encoder Signal Detected!");
      break;
  }
}

void standardProcess() {

  rotWorm = encPos * ratioEncToWorm;
  rotAxle = rotWorm * ratioWormToAxle;
  zPos = rotAxle * ratioAxleToZ;

  float outputAccel = 0.0F;
  switch (opType) {
    case 0:  // In opType 0, the motor does nothing and no PID loops are calculated.  It's a system idle

      break;

    case 1:  // In opType 1, the PID loop is driven by a desired velocity.
      outputAccel = min(max(-MAX_ACCEL_PER_PERIOD,PIDvel.compute(targetSpeed)),MAX_ACCEL_PER_PERIOD);
      targetSpeed = outputAccel + min(max(-MAX_SPEED,targetSpeed),MAX_SPEED);
      motor.setSpeed(targetSpeed);
      motor.apply();

      if (isHomed && !overrideEnabled) {
        if (motor.dir == 1 && encPos >= (EOT_TOP-EOT_BUFFER_TOP)) {
          Serial.println("Motor is within an EOT buffer. Switching to optype 3 for spindown.");
          changeLEDcolor(LED_UP,COL_UNAVAILABLE);
          changeOpType(3);

        } else if (motor.dir == -1 && encPos <= (EOT_BOT+EOT_BUFFER_BOT)) {
          Serial.println("Motor is within an EOT buffer. Switching to optype 3 for spindown.");
          changeLEDcolor(LED_DOWN,COL_UNAVAILABLE);
          changeOpType(3);
        }
      }
      break;

    case 2:  // In opType 2, the PID loop is driven by a desired position.
      outputAccel = min(max(-MAX_ACCEL_PER_PERIOD,PIDpos.compute(encPos)),MAX_ACCEL_PER_PERIOD);
      targetSpeed = outputAccel + min(max(-MAX_SPEED,targetSpeed),MAX_SPEED);
      motor.setSpeed(targetSpeed);
      motor.apply();

      if (abs(targetSpeed) < 1) {
        Serial.print("Arrived at target. Current position is ["); Serial.print(encPos); Serial.println("]. Switching to optype 0.");
        opType = 0;
        motor.setSpeed(0);
        motor.apply();

        if (continuousUp) {
          continuousUp = false;
          changeLEDcolor(LED_UP,COL_DEFAULT);
        }

        if (continuousDown) {
          continuousDown = false;
          changeLEDcolor(LED_DOWN,COL_DEFAULT);
        }
      }
      break;

    case 3: // In opType 3, the system is ramping down to 0 speed.
      outputAccel = min(max(-MAX_ACCEL_PER_PERIOD,PIDvel.compute(targetSpeed)),MAX_ACCEL_PER_PERIOD);
      targetSpeed = outputAccel + min(max(-MAX_SPEED,targetSpeed),MAX_SPEED);
      motor.setSpeed(targetSpeed);
      motor.apply();
      if (targetSpeed == 0) {opType=0;}
      break;

    default: // All other opType values are assumed to be an error and the system is set to idle.
      opType = 0;  Serial.println("Unknown optype; Switching to opType 0.");
  }
  
  checkForStall();
  rotRawLast = encPos;

  buttonUp.scan();
  buttonDown.scan();

  strip.show();
}

/*=============================================================================================================================================================*/

void buttonUpHold() {

  //Panel is unlocked
  if (!panelLocked) {
    if (isHomed){
      if (encPos < EOT_TOP - EOT_BUFFER_TOP || overrideEnabled) {
        changeLEDcolor(LED_UP,COL_HOLD_MOVE);
        Serial.println("Up Button Held.");
        Serial.println("Switching to optype 1.");
        Serial.print("Target speed set to "); Serial.println(MAX_SPEED);
        changeOpType(1);
        PIDvel.changeSetpoint(MAX_SPEED);
      } else {
        changeLEDcolor(LED_UP,COL_UNAVAILABLE);
        Serial.println("Up Button held.  Cannot travel upwards due to EOT.");
      }
    } else {
      changeLEDcolor(LED_UP,COL_UNAVAILABLE);
    }
  }
};

void buttonUpHoldRelease() {
  if (panelLocked) {changeLEDcolor(LED_UP,COL_LOCKED);} else {changeLEDcolor(LED_UP,COL_DEFAULT);}
  Serial.println("Up Button Hold Released.");
  changeOpType(3);
  Serial.println("Switching to optype 3 for spindown.");
};

void buttonUpPressInstant() {
  Serial.println("Up Button Pressed.");
  if (continuousUp) {
    continuousUp = false;
    changeLEDcolor(LED_UP,COL_DEFAULT);
    Serial.println("Aborted movement.");
    Serial.println("Switching to optype 3 for spindown.");
    changeOpType(3);
  }
  if (continuousDown) {
    continuousDown = false;
    changeLEDcolor(LED_DOWN,COL_DEFAULT);
    Serial.println("Aborted movement.");
    Serial.println("Switching to optype 3 for spindown.");
    changeOpType(3);
  }
};

void buttonUpMultiTap(int timesTapped) {
  Serial.print("Registered Up Button Multitap. ["); Serial.print(timesTapped); Serial.println("].");
  switch (timesTapped) {
    case 2:
      if (!panelLocked) {
        if (isHomed) {
          runToTop();
        } else {
          changeLEDcolor(LED_UP,COL_UNAVAILABLE);
        }
      }
      break;
    case 4:
      toggleLock();
      break;
    case 5:
      if (!panelLocked) {
        Serial.print("Manually setting home location.");
        isHomed = true;
        encPos = 0;
      }
      break;
    case 7:
      if (!panelLocked) {
        overrideEnabled = !overrideEnabled;
        Serial.print("Toggled master override.  Enabled: "); Serial.println(overrideEnabled);
      }
      break;
  }

};

void buttonDownHold() {

  // Panel is unlocked
  if (!panelLocked) {
    if (isHomed) {
      if (encPos > EOT_BOT + EOT_BUFFER_BOT || overrideEnabled) {
        changeLEDcolor(LED_DOWN,COL_HOLD_MOVE);
        Serial.println("Down Button Held.");
        Serial.println("Switching to optype 1.");
        Serial.print("Target speed set to "); Serial.println(-MAX_SPEED);
        changeOpType(1);
        PIDvel.changeSetpoint(-MAX_SPEED);
      } else {
        changeLEDcolor(LED_DOWN,COL_UNAVAILABLE);
        Serial.println("Down Button held.  Cannot travel downwards due to EOT.");
      }
    } else {
        changeLEDcolor(LED_DOWN,COL_SPECIAL);
        Serial.println("Down Button Held.");
        Serial.println("Switching to optype 1.");
        changeOpType(1);
        PIDvel.changeSetpoint(-MAX_SPEED_HOMING);
    }
  // Panel is locked
  }
};

void buttonDownHoldRelease() {
  if (panelLocked) {changeLEDcolor(LED_DOWN,COL_LOCKED);} else {changeLEDcolor(LED_DOWN,COL_DEFAULT);}
  Serial.println("Down Button Hold Release.");
  changeOpType(3);
  Serial.println("Switching to optype 3 for spindown.");
};

void buttonDownPressInstant() {
  Serial.println("Down Button Pressed.");
  if (continuousUp) {
    continuousUp = false;
    changeLEDcolor(LED_UP,COL_DEFAULT);
    Serial.println("Aborted movement.");
    Serial.println("Switching to optype 3 for spindown.");
    changeOpType(3);
  }
  if (continuousDown) {
    continuousDown = false;
    changeLEDcolor(LED_DOWN,COL_DEFAULT);
    Serial.println("Aborted movement.");
    Serial.println("Switching to optype 3 for spindown.");
    changeOpType(3);
  }
};

void buttonDownMultiTap(int timesTapped) {
  Serial.print("Registered Down Button Multitap. ["); Serial.print(timesTapped); Serial.println("].");
  switch (timesTapped) {
    case 2:
      if (!panelLocked) {
        if (isHomed) {
          runToBottom();
        } else {
          changeLEDcolor(LED_DOWN,COL_SPECIAL);
          continuousDown = true;
          Serial.println("Down Autohoming engaged.");
          Serial.println("Switching to optype 1.");
          changeOpType(1);
          PIDvel.changeSetpoint(-MAX_SPEED_HOMING);
        }
      }
      break;
    case 3:
      Serial.print("Current position: ");Serial.print(encPos);Serial.print(" --- zPos: ");Serial.println(zPos);
      break;
    case 4:
      toggleLock();
      break;
  }
};

void changeOpType(int ot) {
  switch (ot) {
    case 0:
      opType = 0;
      break;
    case 1:
      opType = 1;
      break;
    case 2:
      opType = 2;
      break;
    case 3:
      opType = 3;
      PIDvel.changeSetpoint(0.0F);
      break;
    default:
      Serial.println("No such opType exists.");
  }
};

void changeLEDcolor(int LEDindex, int color[]) {
  strip.setLedColorData(LEDindex,color[0],color[1],color[2]);
};

void changeLEDcolorAll(int color[]) {
  strip.setAllLedsColorData(color[0],color[1],color[2]);
};

void toggleLock() {
    panelLocked = !panelLocked;
    if (panelLocked) {changeLEDcolorAll(COL_LOCKED);} else {changeLEDcolorAll(COL_DEFAULT);}
};

void checkForStall() {
  if ((encPos == rotRawLast) && (motor.mag != 0)) {
    msStalled += PROCESS_PERIOD;
  } else if ((encPos != rotRawLast) || (motor.mag == 0)) {
    msStalled = 0;
  }

  if (msStalled >= STALL_CUTOUT_MS) {
    Serial.println("Stall detected.  Killing motor. ");
    Serial.print(msStalled);Serial.print(" ms. ");Serial.print(encPos);Serial.print(" : ");Serial.print(rotRawLast);Serial.print(" : ");Serial.println(motor.mag);
    
    opType = 0;
    if (motor.dir == -1 && !isHomed){
      Serial.println("Assuming this to be the new zero zPos.  Setting as home.");
      isHomed = true;
      encPos = 0;
    }
    targetSpeed = 0;
    motor.setSpeed(0);
    changeLEDcolorAll(COL_DEFAULT);
    motor.apply();
  }
};

void runToBottom() {
  changeLEDcolor(LED_DOWN,COL_CONT_MOVE);
  continuousDown = true;
  Serial.println("Down Button Continuous Engaged.");
  Serial.println("Switching to optype 2 and targeting EOT_BOT.");
  changeOpType(2);
  PIDpos.changeSetpoint(EOT_BOT);
};

void runToTop() {
  changeLEDcolor(LED_UP,COL_CONT_MOVE);
  continuousUp = true;
  Serial.println("Up Button Continuous Engaged.");
  Serial.println("Switching to optype 2 and targeting EOT_TOP.");
  changeOpType(2);
  PIDpos.changeSetpoint(EOT_TOP);
};
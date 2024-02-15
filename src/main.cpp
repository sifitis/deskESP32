#include <Arduino.h>
#include <Freenove_WS2812_Lib_for_ESP32.h>
#include <functional>

/*  "We do these things not because they are easy, but because we thought they were GOING to be easy."  */

//End of travel can be detected because the encoder will stop recieving updates.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  PINS  //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// General Pins
#define P_VREF 4

// Encoder Pins
#define P_ENCA 22
#define P_ENCB 23

// Driver Pins
#define P_DSPD 21
#define P_DM1 19
#define P_DM2 18

// Control Pins
#define P_C1 26
#define P_C2 25
#define P_C3 33
#define P_C4 32
#define P_C5 35
#define P_C6 34
#define P_CLED 13

// Led indices
#define LED_UP 0
#define LED_DOWN 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS  CONSTANTS   //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const int ENC_BOUNCE_THRESHHOLD = 10;
const int PROCESS_PERIOD = 20;
const int ENC_SAMPLE_PERIOD = 100;
const int MIN_SPEED = 25;
const int MAX_SPEED = 200;
const int MAX_SPEED_HOMING = 35;
const float MAX_ACCEL = 5;
float TARGETPOS = 230;
const float MAX_INT_WINDUP = 5.0F;
const float EOT_BUFFER = 15.0F;
const float EOT_BOT = 0.0F;
const float EOT_TOP = 1000.0F;
const int STALL_CUTOUT_MS = 250;

const int BUTTON_TAP_THRESHHOLD = 250;
const int BUTTON_IDLE_THRESHHOLD = 500;

int COL_DEFAULT[] = {40,40,40};
int COL_LOCKED[] = {255,0,0};
int COL_CONT_MOVE[] = {0,255,0};
int COL_HOLD_MOVE[] = {0,0,255};
int COL_UNAVAILABLE[] = {255,90,0};
int COL_SPECIAL[] = {100,0,255};

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
volatile long curEncTime = 0;
volatile long lastEncTime = 0;
long long loopTime = 0;

//===oOo==={    Motionstates     }===oOo===//

volatile int rotRaw = 0;
float rotWorm = 0.0f;
float rotAxle = 0.0f;
float zPos = 0.0f;
int rotRawLast = 0;

// Conversion factors
const float ratioEncToWorm = .25F;
const float ratioWormToAxle = 0.0256410256F;
const float ratioAxleToZ = 19.685F; //in millimeters

//===oOo==={   PID Control Loop   }===oOo===//

int motorSpeed = 0;
MotorController motor(P_DSPD,P_DM1,P_DM2);

// PID Control Parameters
const float KPpos = .1;
const float KIpos = 0.000;
const float KDpos = 15;

const float KPvel = 1.0F;
const float KIvel = 0.0F;
const float KDvel = 0.0F;

PIDController PIDpos(KPpos,KIpos,KDpos);
PIDController PIDvel(KPvel,KIvel,KDvel);

//===oOo==={   Buttons and LEDs   }===oOo===//

Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(6, P_CLED, 0);

bool panelLocked = false;
bool panelDontToggleLock = false;
bool continuousUp = false;
bool continuousDown = false;

// Stall Detection
uint32_t msStalled = 0;
bool overrideEnabled = true;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS  FUNCTIONS   //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function Declarations //
void checkEncoder();
void standardProcess();

void encAChange();
void encBChange();
void encChange(bool);

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

/*=============================================================================================================================================================*/

Button buttonUp(P_C1,buttonUpHold, buttonUpHoldRelease, buttonUpPressInstant, buttonUpMultiTap);
Button buttonDown(P_C2,buttonDownHold, buttonDownHoldRelease, buttonDownPressInstant, buttonDownMultiTap);

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

  pinMode(P_C1,INPUT);
  pinMode(P_C2,INPUT);
  pinMode(P_C3,INPUT);
  pinMode(P_C4,INPUT);
  pinMode(P_C5,INPUT);
  pinMode(P_C6,INPUT);
  pinMode(P_CLED,OUTPUT);

  //attachInterrupt(digitalPinToInterrupt(P_ENCA),encAChange,CHANGE);
  //attachInterrupt(digitalPinToInterrupt(P_ENCB),encBChange,CHANGE);

  strip.begin();
  strip.setAllLedsColor(100,100,100);

  Serial.println("Begin.");
  PIDpos.changeSetpoint(TARGETPOS);
  delay(1000);
}

/*=============================================================================================================================================================*/

unsigned long startTimeEnc = micros();
unsigned long startTimeStd = millis();

void loop() {
  if(micros() - startTimeEnc > ENC_SAMPLE_PERIOD) {
    checkEncoder();
    startTimeEnc = micros();
  }

  if(millis() - startTimeEnc > PROCESS_PERIOD) {
    standardProcess();
    startTimeStd = millis();
  }
  
}

void standardProcess() {
  rotRawLast = rotRaw;

  rotWorm = rotRaw * ratioEncToWorm;
  rotAxle = rotWorm * ratioWormToAxle;
  zPos = rotAxle * ratioAxleToZ;

  float outputAccel = 0.0F;
  switch (opType) {
    case 0:  // In opType 0, the motor does nothing and no PID loops are calculated.  It's a system idle

      break;
    case 1:  // In opType 1, the PID loop is driven by a desired velocity.
      outputAccel = min(max(-MAX_ACCEL,PIDvel.compute(motorSpeed)),MAX_ACCEL);
      motorSpeed = outputAccel + min(max(-MAX_SPEED,motorSpeed),MAX_SPEED);
      motor.setSpeed(motorSpeed);
      motor.apply();
      if (isHomed && !overrideEnabled) {
        if (motor.dir == 1 && rotRaw >= (EOT_TOP-EOT_BUFFER)) {
          Serial.println("Motor is within an EOT buffer. Switching to optype 3 for spindown.");
          changeLEDcolor(LED_UP,COL_UNAVAILABLE);
          changeOpType(3);
        } else if (motor.dir == -1 && rotRaw <= (EOT_BOT+EOT_BUFFER)) {
          Serial.println("Motor is within an EOT buffer. Switching to optype 3 for spindown.");
          changeLEDcolor(LED_DOWN,COL_UNAVAILABLE);
          changeOpType(3);
        }
      }
      break;
    case 2:  // In opType 2, the PID loop is driven by a desired position.
      outputAccel = min(max(-MAX_ACCEL,PIDpos.compute(zPos)),MAX_ACCEL);
      motorSpeed = outputAccel + min(max(-MAX_SPEED,motorSpeed),MAX_SPEED);
      motor.setSpeed(motorSpeed);
      motor.apply();

      if (abs(motorSpeed) < 1) {
        Serial.print("Arrived at target. Current position is ["); Serial.print(zPos); Serial.println("]. Switching to optype 0.");
        opType = 0;
        if (continuousUp) {
          continuousUp = false;
          strip.setLedColorData(0,60,60,60);
        }

        if (continuousDown) {
          continuousDown = false;
          strip.setLedColorData(1,60,60,60);
        }
      }

      break;
    case 3: // In opType 3, the system is ramping down to 0 speed.
      outputAccel = min(max(-MAX_ACCEL,PIDvel.compute(motorSpeed)),MAX_ACCEL);
      motorSpeed = outputAccel + min(max(-MAX_SPEED,motorSpeed),MAX_SPEED);
      motor.setSpeed(motorSpeed);
      motor.apply();
      if (motorSpeed == 0) {opType=0;}
      break;

    default: // All other opType values are assumed to be an error and the system is set to idle.
      opType = 0;  Serial.println("Unknown optype; Switching to opType 0.");
  }
  
  if (!overrideEnabled) {checkForStall();}

  buttonUp.scan();
  buttonDown.scan();

  strip.show();
}

/*=============================================================================================================================================================*/

/* void IRAM_ATTR encAChange(){
  encChange(true);
}


void IRAM_ATTR encBChange(){
  encChange(false);
}

// A generalized function to indetify encoder state and increment/decrement raw rotation value appropriately.
void IRAM_ATTR encChange(bool detectA){
  //curEncTime = (long)millis();
  //long tDelta = curEncTime-lastEncTime;
  //if (tDelta>=ENC_BOUNCE_THRESHHOLD){
    bool A = digitalRead(P_ENCA);
    bool B = digitalRead(P_ENCB); 

    if ((A^B && detectA) || (!(A^B) && !detectA)) {rotRaw++;} else {rotRaw--;}

    //lastEncTime = curEncTime;
  //}
} */

void buttonUpHold() {

  //Panel is unlocked
  if (!panelLocked) {
    if (buttonDown.isPressed) {
      toggleLock();
    } else {
      if (isHomed){
        if (rotRaw < EOT_TOP - EOT_BUFFER || overrideEnabled) {
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
  
  //Panel is locked
  } else {
    if (buttonDown.isPressed) {
      if (!panelDontToggleLock) {
        panelLocked = false;
        changeLEDcolorAll(COL_DEFAULT);
      }
    }
  }
};

void buttonUpHoldRelease() {
  if (panelLocked) {changeLEDcolor(LED_UP,COL_LOCKED);} else {changeLEDcolor(LED_UP,COL_DEFAULT);}
  Serial.println("Up Button Hold Released.");
  if (panelDontToggleLock) {panelDontToggleLock = false;}
  changeOpType(3);
  Serial.println("Switching to optype 3 for spindown.");
};

void buttonUpPressInstant() {
  Serial.println("Up Button Pressed.");
  if (!panelLocked) {
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
  }
};

void buttonUpMultiTap(int timesTapped) {
  Serial.print("Registered Up Button Multitap. ["); Serial.print(timesTapped); Serial.println("].");
  if (!panelLocked) {
    switch (timesTapped) {
      case 2:
        if (isHomed) {
          changeLEDcolor(LED_UP,COL_CONT_MOVE);
          continuousUp = true;
          Serial.println("Up Button Continuous Engaged.");
          Serial.println("Switching to optype 2 and targeting EOT_TOP.");
          changeOpType(2);
          PIDpos.changeSetpoint(EOT_TOP);
        } else {
          changeLEDcolor(LED_UP,COL_UNAVAILABLE);
        }
        break;
      case 10:
        Serial.print("Manually setting home location.");
        isHomed = true;
        rotRaw = 0;
        break;
      case 11:
        overrideEnabled = !overrideEnabled;
        Serial.print("Toggled Stall Check.  Enabled: "); Serial.println(overrideEnabled);
        break;
    }
  }
};


void buttonDownHold() {

  // Panel is unlocked
  if (!panelLocked) {
    if (buttonUp.isPressed) {
      if (!panelDontToggleLock) {
        panelLocked = true;
        changeLEDcolorAll(COL_LOCKED);
      }
    } else {
      if (isHomed) {
        if (rotRaw > EOT_BOT + EOT_BUFFER  || overrideEnabled) {
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
          Serial.println("Target speed set to -255.");
          changeOpType(1);
          PIDvel.changeSetpoint(-MAX_SPEED_HOMING);
      }
    }
  
  // Panel is locked
  } else {
    if (buttonUp.isPressed) {
      if (!panelDontToggleLock) {
        panelLocked = false;
        changeLEDcolorAll(COL_DEFAULT);
      }
    }
  }
};

void buttonDownHoldRelease() {
  if (panelLocked) {changeLEDcolor(LED_DOWN,COL_LOCKED);} else {changeLEDcolor(LED_DOWN,COL_DEFAULT);}
  Serial.println("Down Button Hold Release.");
  if (panelDontToggleLock) {panelDontToggleLock = false;}
  changeOpType(3);
  Serial.println("Switching to optype 3 for spindown.");
};

void buttonDownPressInstant() {
  Serial.println("Down Button Pressed.");
  if (!panelLocked) {
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
  }
};

void buttonDownMultiTap(int timesTapped) {
  Serial.print("Registered Down Button Multitap. ["); Serial.print(timesTapped); Serial.println("].");
  if (!panelLocked) {
    switch (timesTapped) {
      case 2:
        if (isHomed) {
          changeLEDcolor(LED_DOWN,COL_CONT_MOVE);
          continuousDown = true;
          Serial.println("Down Button Continuous Engaged.");
          Serial.println("Switching to optype 2 and targeting EOT_BOT.");
          changeOpType(2);
          PIDpos.changeSetpoint(EOT_BOT);
        } else {
          changeLEDcolor(LED_DOWN,COL_UNAVAILABLE);
        }
    }
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
  if (!panelDontToggleLock) {
    panelLocked = true;
    panelDontToggleLock = true;
    changeLEDcolorAll(COL_LOCKED);
  }    
};

int history[1000];
int historyIndex = 0;

void checkForStall() {
  if ((rotRaw == rotRawLast) && (motor.mag != 0)) {
    msStalled += PROCESS_PERIOD;
    history[historyIndex] = rotRaw;
    historyIndex = ++historyIndex%1000;
  } else if ((rotRaw != rotRawLast) || (motor.mag == 0)) {
    msStalled = 0;
  }

  if (msStalled >= STALL_CUTOUT_MS) {
    Serial.println("Stall detected.  Killing motor. ");
    Serial.print(msStalled);Serial.print(" ms. ");Serial.print(rotRaw);Serial.print(" : ");Serial.print(rotRawLast);Serial.print(" : ");Serial.println(motor.mag);
    
    opType = 0;
    if (motor.dir == -1) {
      Serial.println("Assuming this to be the new zero zPos.  Setting as home.");
      isHomed = true;
      rotRaw = 0;
    }
    motorSpeed = 0;
    motor.setSpeed(0);
    motor.apply();
  }
};
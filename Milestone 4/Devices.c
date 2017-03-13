#include "Milestone4.c"

// state code:

// THE ROBOTS BEHAVIOR IS BASED UPON STATE TRANSITIONS OF TYPE ROBOT_STATE
typedef enum Robot_state
{
  STATE_IDLE = 0,   // THE FIRST STATE
  STATE_SEARCH,     // POINTING VEHICLE TOWARD THE BEACON'S LIGHT
  STATE_SWEEP,     // SWEEP UNTIL BEACON IS LOST TO FIND HIGHEST POINT
  STATE_PINPOINT,     // MOVE TO HIGHEST POINT
  STATE_WALK,       // WAKING TOWARD THE BEACON STRAIGHTLY UNTIL A TRANSITION OCCURS
  STATE_STOP,       // WHEN THE VEHICLE IS IN RANGE OF THE BEACON
} Robot_state;

// control (data) code:

// DATA COMPUTED, PASSED INTO AND POTENTIALLY ALTERED IN VARIOUS STATES
typedef struct {
	bool button1_pushed;     // button is pressed
	bool button2_pushed;
	bool limitLeft_pushed;   // limit switch is pressed
	bool limitRight_pushed;
  bool beaconFound;        // is the guy facing the beacon?
	bool targetClose;        // is the beacon nearby (in range) to the guy
	int deltaLight;					 // the results from monitorLight are stored here, tested in SWEEP
} RobotControl;

// CALLED ONCE AT THE START OF THE PROGRAM
void initializeController(RobotControl & control) {
  control.button1_pushed = false;
  control.button2_pushed = false;
  control.limitLeft_pushed = false;
  control.limitRight_pushed = false;
  control.deltaLight = 0;
}

// claw code:

// SET SPEED OF CLAW TO BE OPENING (POSITIVE) OR CLOSING (NEGATIVE)
// WARNING: Do not pass a high value into this function or risk shorting motor!
void setClawSpeed(short speed) {
	motor[MClaw] = speed;
}

// sonar code:

// RETURN !raw! SENSOR VALUE: -1 if nothing is heard
short SonarValue() {
	return SensorValue[Sonar];
}
// RETURNS TRUE OR FALSE IF THE SONAR VALUE IS PERCEIVED TO BE IN THESE RANGES:
bool SonarGreaterThan(short dist) {
	if (SensorValue[Sonar] > dist || SensorValue[Sonar] == -1) {return true;}
	else {return false;}
}
bool SonarLessThan(short dist) {
	if (SensorValue[Sonar] < dist && SensorValue[Sonar] != -1) {return true;}
	else {return false;}
}
bool SonarGreaterThanEqual(short dist) {
	if (SensorValue[Sonar] >= dist || SensorValue[Sonar] == -1) {return true;}
	else {return false;}
}
bool SonarLessThanEqual(short dist) {
	if (SensorValue[Sonar] <= dist && SensorValue[Sonar] != -1) {return true;}
	else {return false;}
}

// buttons code:

// A FUNCTION TO MONITOR IF THE STATE OF A BUTTON MOVES FROM 0 TO 1
void monitorButtons( RobotControl & control)
{
  if(SensorValue(Button1) && !control.button1_pushed)
    control.button1_pushed = true;

  if(SensorValue(Button2) & !control.button2_pushed)
    control.button2_pushed = true;
}

// limit switches code:

// A FUNCTION TO MONITOR IF THE STATE OF A LIMIT SWITCH MOVES FROM 0 TO 1
void monitorLimitSwitches( RobotControl & control)
{
  if(SensorValue(LimitLeft) & !control.limitLeft_pushed)
    control.limitLeft_pushed = true;

  if(SensorValue(LimitRight) & !control.limitRight_pushed)
    control.limitRight_pushed = true;
}

// MONITOR BOTH THE BUTTONS AND LIMIT SWITCHES
void monitorButtonsAndLimitSwitches( RobotControl & control )
{
  monitorButtons(control);
  monitorLimitSwitches(control);
}

// RETURN TRUE IF ANY LIMIT SWITCH IS PRESSED
bool anyLimitSwitchHit(RobotControl & control) {
	if (control.limitRight_pushed || control.limitLeft_pushed)
    {  return true;  }
	else
    { return false; }
}
// SET THE LIMIT SWITCHES' CONTROL VARIABLES TO FALSE
void setLimitSwitchesFalse(RobotControl & control) {
  control.limitLeft_pushed = false;
  control.limitRight_pushed = false;
}

// L.E.D. code:

// SET ALL THE LEDS
void setLEDs(int light1, int light2, int light3) {
	SensorValue[LED1] = light1;
	SensorValue[LED2] = light2;
	SensorValue[LED3] = light3;
}
// SET LED1 IF PASSED TRUE AS A PARAMETER, ELSE FALSE : LED2 + LED3 AS ETC.
void setLED1If(bool t) {
	if (t) {SensorValue[LED1] = 1;}
	else {SensorValue[LED1] = 0;}
}
void setLED2If(bool t) {
	if (t) {SensorValue[LED2] = 1;}
	else {SensorValue[LED2] = 0;}
}
void setLED3If(bool t) {
	if (t) {SensorValue[LED3] = 1;}
	else {SensorValue[LED3] = 0;}
}

// wheels code:

// CALL TO SET MOTORS WITHOUT CONSIDERING AN OFFSET OR A PID CONTROLLER
void setWheelsManuallyLR(short LSpeed, short RSpeed) {
	motor[MLeft] = LSpeed;
	motor[MRight] = RSpeed;
}

// STOP ALL MOTORS
void stopAllMotors() {
	setWheelsManuallyLR(0,0);
	motor[MClaw] = 0;
}

// encoder code:

// GET LEFT WHEEL ENCODER TICK VALUE
short getLeftWheelEncoder()
{
  return getMotorEncoder(MLeft);
}
// GET RIGHT WHEEL ENCODER TICK VALUE
short getRightWheelEncoder()
{
  return getMotorEncoder(MRight);
}
//ZERO LEFT ENCODER
void zeroLeftWheelEncoder() {
  resetMotorEncoder(MLeft);
}
// ZERO RIGHT ENCODER
void zeroLeftWheelEncoder() {
  resetMotorEncoder(MRight);
}
// RESET BOTH WHEEL MOTOR ENCODERS TO ZERO
void zeroWheelEncoders() {
			resetMotorEncoder(MLeft);
			resetMotorEncoder(MRight);
}

// Proportional controller code:

// THE RIGHT WHEEL IS ESTIMATED TO RUN FIVE TICK
// MASTER POWER IS THE LEFT WHEEL
// SLAVE POWER IS THE RIGHT WHEELS SLOWER SO THERE ISNT A HUGE ERROR IMMEDIATELY
const int roughGuess = 5;
const int kp = 5;
int error = 0;
int slavePower = 0;
int masterPower = 0;
// INITIALIZE P CONTROLLER for forward movement BEFORE ENTERING A STATE THAT USES THE P CONTROLLER
void initializeForwardPController(short speed) {
  masterPower = speed;
  slavePower = speed - roughGuess;
  error = 0;
  zeroWheelEncoders();
  time1[T2] = 0;
  clearTimer(T2);
}

// INITIALIZE P CONTROLLER for turning movement BEFORE ENTERING A STATE THAT USES THE P CONTROLLER
// speed variable should only ever be positive
void initializeTurningPController(bool isTurningRight, short speed) { // + speed is turning right
  if (isTurningRight) // L is + , R is -
    { masterPower = speed;
    	slavePower = -speed + roughGuess; } // - R
  else // R is + , L is -
    { masterPower = -speed;
    	slavePower = speed - roughGuess; } // + R
  error = 0;
  time1[T2] = 0;
  clearTimer(T2);
  zeroWheelEncoders();
}

// RESET P CONTROLLER AFTER LEAVING A STATE THAT USES THE P CONTROLLER
void resetPController() {
  error = slavePower = masterPower = 0;
  zeroWheelEncoders();
  stopAllMotors();
}

// DRIVE COMPLETELY STRAIGHT
void driveStraight() {
  setWheelsManuallyLR(masterPower,slavePower);
  if (time1[T2] > 100 ) {
    error = abs(getLeftWheelEncoder()) - abs(getRightWheelEncoder());
    slavePower += error / kp;
    zeroWheelEncoders();
  }
}

// TURN ON A DIME
int TurnPerfectly() {
  int delta = 0;
  setWheelsManuallyLR(masterPower,slavePower);
  if (time1[T2] > 100 ) {
    error = abs(getLeftWheelEncoder()) - abs(getRightWheelEncoder());
    if (slavePower > 0) { // R is -
      slavePower -= error / kp;
    }
    else if (slavePower < 0) { // R is +
      slavePower += error / kp;
    }
    delta = abs(getLeftWheelEncoder());
    zeroWheelEncoders();
  }
  return delta;
}

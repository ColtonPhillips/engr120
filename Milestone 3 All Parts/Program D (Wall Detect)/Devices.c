#include "Milestone3.c"
// the right motor has to be offset a bit
#define MOTOR_CUSTOM_R_SPEED_OFFSET 7
	#define FASTSPEED 100
	#define MEDIUMSPEED 50
	#define SLOWSPEED 30

// STATE MACHINE T_states
typedef enum T_state
{
  STATE_IDLE = 0,
  STATE_WALK,
  STATE_BACKAWAY,
} T_state;

// DATA PASSED TO AND ALTERED IN VARIOUS STATE
typedef struct {
	bool button1_pushed; // is pressed
	bool button2_pushed;
	bool button1_toggle_state; //these act like on-off switches
	bool button2_toggle_state;
	bool limitLeft_pushed; // is pressed
	bool limitRight_pushed;
} RobotControl;

// OPENING CLAW IS POSITIVE
// CLOSING CLAW (ONTO MAGNET THATS HELD IN PLACE) IS NEGATIVE
void setClaw(int i) {
	motor[MClaw] = i;
}
//SENSOR VALUE
int SonarValue() {
	return SensorValue[Sonar];
}
// HANDLES THE -1 CASE OF SONAR SENSOR
bool SonarGreaterThan(int dist) {
	if (SensorValue[Sonar] > dist || SensorValue[Sonar] == -1) {return true;}
	else {return false;}
}
bool SonarLessThan(int dist) {
	if (SensorValue[Sonar] < dist && SensorValue[Sonar] != -1) {return true;}
	else {return false;}
}
bool SonarGreaterThanEqual(int dist) {
	if (SensorValue[Sonar] >= dist || SensorValue[Sonar] == -1) {return true;}
	else {return false;}
}
bool SonarLessThanEqual(int dist) {
	if (SensorValue[Sonar] <= dist && SensorValue[Sonar] != -1) {return true;}
	else {return false;}
}

// CALL THIS TO ACTUALLY SET WHEEL MOTORS WITHOUT CONSIDERING AN OFFSET
void setWheelsManualLR(int L, int R) {
	motor[MLeft] = L;
	motor[MRight] = R;
}
// WHEELS MOVE IN SAME DIRECTION and POSITIVE VALUES MOVE THE ROBOT FORWARD
// RIGHT WHEEL MOVES FASTER NATURALLY
void setOffsetWheels(int spd) {
	if (spd > MOTOR_CUSTOM_R_SPEED_OFFSET) { 									// positive speed
		setWheelsManualLR(spd,spd-MOTOR_CUSTOM_R_SPEED_OFFSET);	// negative offset
	}
	else if (spd < -MOTOR_CUSTOM_R_SPEED_OFFSET) { 						// negative speed
		setWheelsManualLR(spd,spd+MOTOR_CUSTOM_R_SPEED_OFFSET);	// positive offset
	}
	else { // we shouldn't call this function with low values. set all to zero.
		setWheelsManualLR(0,0);
	}
}
// THIS METHOD GIVES THE FINEST GRAIN CONTROL IF USED PROPERLY
void setWheelsLORO( int Left, int LeftOffset, int Right, int RightOffset) {
	setWheelsManualLR(Left - LeftOffset,Right - RightOffset);
}
// STOP ALL MOTORS (FAULT TOLERANCE)
void setAllMotorsToZero() {
	setWheelsManualLR(0,0);
	motor[MClaw] = 0;
}
// CALLED ONCE AT THE START OF THE PROGRAM
void init_devices(RobotControl & control) {
  control.button1_pushed = false;
  control.button2_pushed = false;
  control.button1_toggle_state  = false;
  control.button2_toggle_state  = false;
  control.limitLeft_pushed = false;
  control.limitRight_pushed = false;
}
// TOGGLE BUTTON STATES 1 AND 2
void flip_1(RobotControl & control) {
	control.button1_toggle_state = !control.button1_toggle_state;
}
void flip_2(RobotControl & control) {
	control.button2_toggle_state = !control.button2_toggle_state;
}
// LIMITLEFT AND LIMIT RIGHT ARE SET TO TRUE HERE
void monitorLimits( RobotControl & control)
{
  if(SensorValue(LimitLeft) & !control.limitLeft_pushed)
    control.limitLeft_pushed = true;

  if(SensorValue(LimitRight) & !control.limitRight_pushed)
    control.limitRight_pushed = true;
}
// BUTTON 1 AND 2, and Limits ARE SET TO TRUE HERE
void monitorInput( RobotControl & control)
{
  if(SensorValue(Button1) && !control.button1_pushed)
    control.button1_pushed = true;

  if(SensorValue(Button2) & !control.button2_pushed)
    control.button2_pushed = true;

	monitorLimits(control);
}
// IF THE LIMIT SWITCHES ARE TRIGGERED
bool anyLimitHit(RobotControl & control) {
	if (control.limitRight_pushed || control.limitLeft_pushed)
		{return true;}
	else {return false;}
}
// SET THE LIMITS TO FALSE (AFTER THEY ARE PRESSED)
void setLimitsFalse(RobotControl & control) {
  control.limitLeft_pushed = false;
  control.limitRight_pushed = false;
}

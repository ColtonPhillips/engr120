#include "Milestone3.c"

// NEW MACROS
#define MOTOR_CUSTOM_R_SPEED_OFFSET 7

// MACROS FROM PREVIOUS BUILDS
/*	#define NINETYDEG_ISH 972
	#define FASTSPEED 90
	#define MEDIUMSPEED 40
	#define SLOWSPEED 20
	#define CLAW_TIME 200 // opening and closing
  #define CLAW_SPEED 30

  #define FORWARD_TIME 2000
  #define BACKWARD_TIME 2400
  #define TURNAWAY_TIME 750 // NOT USED
  #define BACKAWAY_TIME 450
  //*/

	// STATE MACHINE T_states
typedef enum T_state
{
  STATE_IDLE = 0,
} T_state;
// DATA PASSED TO AND ALTERED IN EACH STATE
typedef struct {
	bool button1_pushed; // is pressed
	bool button2_pushed;
	bool button1_toggle_state; //these act like on-off switches
	bool button2_toggle_state;
} RobotControl;

// Helper Functions:
//
// FLIP A TOGGLE FROM FALSE TO TRUE OR TRUE TO FALSE
// OPENING CLAW IS POSITIVE
// CLOSING CLAW (ONTO MAGNET THATS HELD IN PLACE) IS NEGATIVE
void setClaw(int i) {
	motor[MClaw] = i;
}
// WHEELS MOVE IN(SAME DIRECTION)
// POSITIVE VALUES MOVE THE ROBOT FORWARD
// BUG: THE SPEED OFFSET IS STATIC, SHOULD BE DYNAMIC (LINEAR)
void setWheelsSpeed(int spd) {
	motor[MLeft] = spd;
	motor[MRight] = spd - MOTOR_CUSTOM_R_SPEED_OFFSET; // handle offset here!
}
void setWheelsLR( int Left, int Right) {
	motor[MLeft] = Left;
	motor[MRight] = Right - MOTOR_CUSTOM_R_SPEED_OFFSET; // handle offset here!
}
void setWheelsLORO( int Left, int LeftOffset, int Right, int RightOffset) {
	motor[MLeft] = Left - LeftOffset;
	motor[MRight] = Right - RightOffset;
}
// STOP ALL MOTORS (FAULT TOLERANCE)
void setAllMotorsToZero() {
	motor[MLeft] = 0;
	motor[MRight] = 0;
	motor[MClaw] = 0;
}

// Called once at the start of the program
void init_devices(RobotControl & control) {
  control.button1_pushed = false;
  control.button2_pushed = false;
  control.button1_toggle_state  = false;
  control.button2_toggle_state  = false;
}

// ability to toggle state of buttons
void flip_1(RobotControl & control) {
	control.button1_toggle_state = !control.button1_toggle_state;
}
void flip_2(RobotControl & control) {
	control.button2_toggle_state = !control.button2_toggle_state;
}
// BUTTON 1 AND 2 ARE SET TO TRUE HERE
void monitorInput( RobotControl & control)
{
  if(SensorValue(Button1) && !control.button1_pushed)
    control.button1_pushed = true;

  if(SensorValue(Button2) & !control.button2_pushed)
    control.button2_pushed = true;
}

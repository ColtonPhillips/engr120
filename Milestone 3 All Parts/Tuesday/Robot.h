#ifndef HEADER_ROBOT
#define HEADER_ROBOT

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
#endif

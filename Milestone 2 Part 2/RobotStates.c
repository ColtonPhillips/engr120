#include "Robot.h"
#include "Milestone2Claw.c"

void setClaw(int i) {
	motor[MClaw] = i;
}

// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcessStateIdle(RobotControl & control) {
	setClaw(0);
	// BUTTON ONE TRANSITIONS TO OPENING
	if (control.button1_pushed) {
		control.button1_pushed = false;
		return STATE_OPENING;
	}
	// BUTTON TWO TRANSITIONS TO MOVING
	else if (control.button2_pushed) {
			control.button2_pushed = false;
			return STATE_CLOSING;
	}
	return STATE_IDLE;
}
T_state ProcessStateOpening(RobotControl & control) {
	setClaw(10);
	wait1Msec(1000);
	return STATE_IDLE;
}
T_state ProcessStateClosing(RobotControl & control) {
	setClaw(-10);
	wait1Msec(1000);
	return STATE_IDLE;
}

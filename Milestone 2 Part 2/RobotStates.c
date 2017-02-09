#include "Robot.h"
#include "Milestone2Claw.c"

void resetAllEncoders() {
			//resetMotorEncoder(MLeft);
			//resetMotorEncoder(MRight);
			resetMotorEncoder(MClaw);
}
// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcessStateIdle(RobotControl & control) {
	// BUTTON ONE TRANSITIONS TO OPENING
	if (control.button1_pushed) {
		control.button1_pushed = false;
		resetAllEncoders();
		return STATE_OPENING;
	}
	// BUTTON TWO TRANSITIONS TO MOVING
	else if (control.button2_pushed) {
			control.button2_pushed = false;
			resetAllEncoders();
			return STATE_CLOSING;
	}
	return STATE_IDLE;
}
T_state ProcessStateOpening(RobotControl & control) {
	return STATE_IDLE;
}
T_state ProcessStateClosing(RobotControl & control) {
	return STATE_IDLE;
}

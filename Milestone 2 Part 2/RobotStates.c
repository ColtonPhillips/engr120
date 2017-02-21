#include "Robot.h"
#include "Milestone2Claw.c"

void setClaw(int i) {
	motor[MClaw] = i;
}

// SET BOTH WHEEL MOTORS TO M (SAME DIRECTION)
// POSITIVE VALUES MOVE THE ROBOT FORWARD
void setBothMotors( int m) {
	motor[MLeft] = m;
	motor[MRight] = m - MOTOR_SPEED_OFFSET; // handle offset here!
}

void setMotorToZero() {
	motor[MLeft] = 0;
	motor[MRight] = 0;
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
	setClaw(CLAW_SPEED);
	wait1Msec(CLAW_TIME);
	return STATE_FORWARD;
}

T_state ProcessStateClosing(RobotControl & control) {
	setClaw(-CLAW_SPEED);
	wait1Msec(CLAW_TIME);
	return STATE_IDLE;
}

// WHEN THE ROBOT IS MOVING TOWARD THE TARGET
T_state ProcessStateForward(RobotControl & control) {
	setBothMotors(SLOWSPEED);
	wait1Msec(FORWARD_TIME);
	return STATE_BACKWARD;
}

// WHEN THE ROBOT IS MOVING AWAY THE TARGET
T_state ProcessStateBackward(RobotControl & control) {
	setBothMotors(-SLOWSPEED);
	wait1Msec(BACKWARD_TIME);
	setMotorToZero();
	return STATE_IDLE;
}

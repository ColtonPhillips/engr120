// REMEMBER: The Left Wheel's Encoder is NEGATIVE for motor[MLeft]= i, when i is positive.
//					The Right Wheel's Encoder is POSITIVE for motor[MRight]= j, when j is positive.
#include "Robot.h"
#include "Milestone1Move.c"
// SET BOTH WHEEL MOTORS TO M (SAME DIRECTION)
// POSITIVE VALUES MOVE THE ROBOT FORWARD
void setBothMotors( int m) {
	motor[MLeft] = m;
	motor[MRight] = m;
}
// RESET BOTH WHEEL MOTOR ENCODERS TO ZERO
void resetBothEncoders() {
			resetMotorEncoder(MLeft);
			resetMotorEncoder(MRight);
}
// CALLED BEFORE TRANSITIONING TO IDLE
//void setButtonsFalse(RobotControl & control) {
//	control.button1_pushed = false;
//	control.button2_pushed = false;
//}
// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcessStateIdle(RobotControl & control) {
	setBothMotors(0);
	resetBothEncoders();
	// BUTTON ONE TRANSITIONS TO MOVING
	if (control.button1_pushed) {
		control.button1_pushed = false;
		// Reset encoder, Set Motor to move forward, Change State
		resetBothEncoders();
		motor[MLeft] = MEDIUMSPEED;
		motor[MRight] = MEDIUMSPEED - MOTOR_SPEED_OFFSET;
		return STATE_MOVING;
	}
	// BUTTON TWO TRANSITIONS TO MOVING
	else if (control.button2_pushed) {
			control.button2_pushed = false;
			// Reset encoder, Set Left wheel forward, Right wheel backward, Change State
			resetBothEncoders();
			motor[MLeft] = -FASTSPEED;
			motor[MRight] = FASTSPEED - MOTOR_SPEED_OFFSET;
			return STATE_TURNING;
	}
	return STATE_IDLE;
}
// WHEN THE ROBOT IS TURNING, MOTORS MOVING, WAITING FOR ENCODER > NINETYDEG
T_state ProcessStateTurning(RobotControl & control) {
	// PUT LEFT ENCODER AND RIGHT ENCODER INTO PURELY POSITIVE VARIABLES
	int leftenc = getMotorEncoder(MLeft);   // DO NOT NEGATE. MOTOR IS MOVING NEGATIVELY ALREADY, SO POSITIVE ENCODER
	int rightenc = getMotorEncoder(MRight); // ALREADY POSITIVE
	if (leftenc > NINETYDEG || rightenc > NINETYDEG) {
		// WHEN MOVED 90 DEGREES, SET MOTORS TO ZERO, CHANGE STATE TO IDLE
		setBothMotors(0);
		//setButtonsFalse(control);
		//resetBothEncoders();
		return STATE_IDLE;
	}
	return STATE_TURNING;
}
// WHEN THE ROBOT IS MOVING, MOTORS MOVING, WAITING FOR ENCODER > ONEMETRE
T_state ProcessStateMoving(RobotControl & control) {
	// PUT LEFT AND RIGHT ENCODER INTO PURELY POSITIVE VARIABLES
	int leftenc = -getMotorEncoder(MLeft);  // NEGATE. MOTOR IS MOVING POSITIVELY, SO NEGATIVE ENCODER
	int rightenc = getMotorEncoder(MRight); // ALREADY POSITIVE
	if (leftenc > ONEMETRE || rightenc > ONEMETRE) {
		// WHEN MOVED 1 METRE, SET MOTORS TO ZERO, CHANGE STATE TO IDLE
		setBothMotors(0);
		//setButtonsFalse(control);
		//resetBothEncoders();
		return STATE_IDLE;
	}
	return STATE_MOVING;
}

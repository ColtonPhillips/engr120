#include "Robot.h"
#include "Milestone2Claw.c"
// Helper Functions:
//
// BUTTON 1 AND 2 ARE SET TO TRUE HERE
void monitorInput( RobotControl & control)
{
  if(SensorValue(Button1) && !control.button1_pushed)
    control.button1_pushed = true;

  if(SensorValue(Button2) & !control.button2_pushed)
    control.button2_pushed = true;
}
// OPENING CLAW IS POSITIVE
// CLOSING CLAW (ONTO MAGNET THATS HELD IN PLACE) IS NEGATIVE
void setClaw(int i) {
	motor[MClaw] = i;
}
// WHEELS MOVE IN(SAME DIRECTION)
// POSITIVE VALUES MOVE THE ROBOT FORWARD
void setMotorsLR( int Left, int Right) {
	motor[MLeft] = Left;
	motor[MRight] = Right - MOTOR_SPEED_OFFSET; // handle offset here!
}
// STOP ALL MOTORS (FAULT TOLERANCE)
void setAllMotorsToZero() {
	motor[MLeft] = 0;
	motor[MRight] = 0;
	motor[MClaw] = 0;
}
// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcStateIdle(RobotControl & control) {
    monitorInput(control);
	setAllMotorsToZero();
	// BUTTON ONE TRANSITIONS TO: OPENING claw -> moving FORWARD ->
	//
	if (control.button1_pushed) {
		control.button1_pushed = false;
		return STATE_OPENING; // make kissy boys
	}
	// BUTTON TWO TRANSITIONS TO MOVING
	else if (control.button2_pushed) {
			control.button2_pushed = false;
			return STATE_TOGGLECLAW; // debug open or close transition
	}
	return STATE_IDLE;
}

// RIGHT BUTTON
//
// Step 1: Used to open/close the claw
T_state ProcStateToggleClaw(RobotControl & control) {
	if (control.toggle_flag) {
		setClaw(CLAW_SPEED);
	}
	else {
		setClaw(-CLAW_SPEED);
	}
	control.toggle_flag = !control.toggle_flag;
	wait1Msec(CLAW_TIME);
	return STATE_IDLE;
}

//LEFT BUTTON
//
// For our janky purposes, we're going to use Process State Opening
// as the opening of our claw, to actually put the magnet on there,
// I know it's awful but that's how we are gonna role. 2 buttons baby. woo.
// Step 2:
T_state ProcStateOpening(RobotControl & control) {
	setClaw(CLAW_SPEED);
	wait1Msec(CLAW_TIME);
	return STATE_FORWARD;
}
// Step 3
// WHEN THE ROBOT IS MOVING TOWARD THE BEACON TARGET (KISSY BOYS TECHNIQUE)
T_state ProcStateForward(RobotControl & control) {
	setMotorsLR(SLOWSPEED, SLOWSPEED);
	wait1Msec(FORWARD_TIME);
	return STATE_BACKWARD;
}
// Step 4
// at this point we have damn near verifiably made kissy boys
// WHEN THE ROBOT IS MOVING AWAY THE BEACON TARGET
T_state ProcStateBackward(RobotControl & control) {
	setMotorsLR(-SLOWSPEED, -SLOWSPEED);
	wait1Msec(BACKWARD_TIME);
	setAllMotorsToZero();
	return STATE_TURNAWAY;
}
// Step 5
// WHEN THE ROBOT IS TURNING AWAY FROM THE BEACON, HOPEFULLY, MISSING CABLE ;)
T_state ProcStateTurnAway(RobotControl & control) {
	setMotorsLR(-(FASTSPEED+5) , -SLOWSPEED); // LETS SEE THIS PUPPY TURN
	wait1Msec(TURNAWAY_TIME);
	setAllMotorsToZero();
	wait1Msec(300);
	return STATE_BACKAWAY;
}
// Step 6
// move back a bit real fast to impress the judges :)
T_state ProcStateBackAway(RobotControl & control) {
	setMotorsLR(-FASTSPEED , -FASTSPEED);
	wait1Msec(300); // a blink of an eye
	setAllMotorsToZero();
	return STATE_IDLE;
}

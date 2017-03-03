#include "Milestone3.c"
#include "Devices.c"
// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcStateIdle(RobotControl & control) {
  monitorInput(control);
	setAllMotorsToZero();
	if (control.button1_pushed) {
		control.button1_pushed = false;
		flip_1(control);
		return STATE_WALK;
	}
	else if (control.button2_pushed) {
			control.button2_pushed = false;
			flip_2(control);
			return STATE_IDLE;
	}
	return STATE_IDLE;
}
#define SIZEABLE_GAP 25  //in cm.
// WHEN THE ROBOT IS WALKING TOWARD THE WALL AT ANY ANGLE
T_state ProcStateWalk(RobotControl & control) {
	monitorLimits(control);
	setOffsetWheels(MEDIUMSPEED);
	if (SonarLessThanEqual(SIZEABLE_GAP)) {
		return STATE_IDLE;
	}
	else if (anyLimitHit(control)) {
		setLimitsFalse(control);
		return STATE_BACKAWAY;
	}
	return STATE_WALK;
}
// WHEN THE ROBOT RESPONDS TO TOUCHING THE WALL
#define BACKAWAY_TIME 750
T_state ProcStateBackaway(RobotControl & control) {
	setOffsetWheels(-MEDIUMSPEED);
	wait1Msec(BACKAWAY_TIME);
	setOffsetWheels(0);
	return STATE_IDLE;
}

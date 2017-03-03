#include "Milestone3.c"
#include "Devices.c"
#include "LightDetector.c"
// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcStateIdle(RobotControl & control) {
  monitorInput(control);
	setAllMotorsToZero();
	if (control.button1_pushed) {
		control.button1_pushed = false;
		flip_1(control);
		resetBothEncoders();
		return STATE_SEARCH;
	}
	else if (control.button2_pushed) {
			control.button2_pushed = false;
			flip_2(control);
			return STATE_IDLE;
	}
	return STATE_IDLE;
}
#define FOURTYFIVEDEGREES 486
// WHEN THE ROBOT IS SEARCHING FOR THE BEACON THROUGH ROTATION
T_state ProcStateSearch(RobotControl & control) {
	static bool turningRight = true;
	static int turningDistance = FOURTYFIVEDEGREES/4;
	static int spd = SLOWSPEED*1.5;
	if (turningRight) {
		turnRight(spd);
	}	else {
		turnLeft(spd);
	}
	// We sweep back and forth until it is found, going further each time.
	if (turnComplete(turningDistance)) {
		turningDistance *= 2;
		turningRight = !turningRight;
		resetBothEncoders();
	}
	if (control.beaconFound) {
		turningDistance = FOURTYFIVEDEGREES/4;
		turningRight = true;
		resetBothEncoders();
		return STATE_IDLE;
	}
	return STATE_SEARCH;
}

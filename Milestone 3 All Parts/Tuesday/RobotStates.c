#include "Milestone3.c"
#include "Devices.c"
// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcStateIdle(RobotControl & control) {
  monitorInput(control);
	setAllMotorsToZero();
	if (control.button1_pushed) {
		control.button1_pushed = false;
		flip_1(control);
		return STATE_IDLE;
	}
	else if (control.button2_pushed) {
			control.button2_pushed = false;
			flip_2(control);
			return STATE_IDLE;
	}
	return STATE_IDLE;
}
#include "Milestone3.c"
#include "Devices.c"
// WHEN THE ROBOT IS IDLING, WAITING FOR BUTTON PRESSES
T_state ProcStateIdle(RobotControl & control) {
  monitorInput(control);
	setAllMotorsToZero();
	if (control.button1_pushed) {
		resetBothEncoders();
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
T_state ProcStateWalk(RobotControl & control) {
	monitorLimits(control);
	// PID CODE
	static bool canEnter = true;
	static int totalTicks;
	static int slavePower;
	static int masterPower;
	static int error;
	static int kp;
	if (canEnter) {
		totalTicks = 0;
		masterPower = MEDIUMSPEED; // left
		slavePower = masterPower - 5;//MOTOR_CUSTOM_R_SPEED_OFFSET;
		error = 0;
		kp = 10;
		// RESET BOTH WHEEL MOTOR ENCODERS TO ZERO
		resetBothEncoders();
		canEnter = false;
	}
	 //Proportional algorithm to keep the robot going straight.
    motor[MLeft] = masterPower;
    motor[MRight] = slavePower;
   	if (time1[T2] > 100) {
	    error = getMotorEncoder(MLeft) - getMotorEncoder(MRight);
	    slavePower += error / kp;
	    resetBothEncoders();
	  	clearTimer(T2);
	  }
	// END OF PID CODE

//	setOffsetWheels(MEDIUMSPEED);
	if (SonarLessThanEqual(SIZEABLE_GAP)) {
		canEnter = true;
		setLimitsFalse(control);
		return STATE_IDLE;
	}
	else if (anyLimitHit(control)) {
		canEnter = true;
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

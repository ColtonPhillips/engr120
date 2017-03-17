#include "Devices.c"
#include "LightDetector.c"

// THIS BLOCK OF CODE HAPPENS EVERY STEP PRIOR TO ANY PARTICULAR STATES CODE RUNS
void ProcBeforeAnyStateRuns(Robot_state state, RobotControl & control) {
  if (state == STATE_IDLE) {monitorButtonsAndLimitSwitches(control);}
  control.beaconFound = monitorLight(control);
}

// LEFT BUTTON: GOTO: SEARCH STATE
// RIGHT BUTTON: TOGGLE CLAW TO OPEN OR CLOSED
const short searchSpeed = 34;
bool turningRight = true;
Robot_state ProcStateIdle(RobotControl & control) {
  stopAllMotors();
  if (control.button1_pushed) {
		control.button1_pushed = false;
    initializeTurningPController(turningRight, searchSpeed);
		return STATE_SEARCH;
	}
	if (control.button2_pushed) {
		control.button2_pushed = false;
		return STATE_CLAWTOGGLE;
	}
  return STATE_IDLE;
}

// TOGGLE CLAW OPEN OR CLOSED AND GOTO: IDLE STATE
const int ClawSpeed = 20;
const int ClawTime = 400;
Robot_state ProcStateClawToggle(RobotControl & control) {
	if (control.toggle_flag) {
		setClawSpeed(ClawSpeed);
	}
	else {
		setClawSpeed(-ClawSpeed);
	}
	control.toggle_flag = !control.toggle_flag;
	wait1Msec(ClawTime);
	setClawSpeed(0);
	return STATE_IDLE;
}
/*
int distanceToSweepBack = 0;
const short WalkingSpeed = 30;
const int twelveDegreesInTicks = 120;
int distance_sweeped = 0;
const int slowTurningSpeed = 33;
int advance_distance = 0;
int advance_target_distance = 0;
int deltaLightMax = 0;
int encoderAtPinpoint = 0;
const short tooCloseToTarget =  5;
const short closeEnoughToTarget = 20;


Robot_state ProcStateSearch(RobotControl & control) {
	static int distance_turned = 0;
	static int turningDistance = twelveDegreesInTicks;
	distance_turned += TurnPerfectly();
	// We sweep back and forth until it is found, going further each time.
	if (distance_turned > turningDistance) {
    distance_turned = 0;
		turningDistance *= 2;
		turningRight = !turningRight;
		resetPController();
		initializeTurningPController(turningRight, searchSpeed);
	}
	else if (control.beaconFound) {
		distance_turned = 0;
		turningDistance = twelveDegreesInTicks;
		resetPController();
		deltaLightMax = 0;
		distanceToSweepBack = 0;
		distance_sweeped = 0;
		if (SonarGreaterThan(50)) {
			setLEDs(0,1,0);
			initializeForwardPController(WalkingSpeed);
			advance_distance = 0;
			advance_target_distance = SonarValueFiltered()- 65;
			return STATE_ADVANCE;
		}
		else {
			setLEDs(1,1,1);
	    initializeTurningPController(turningRight, slowTurningSpeed);
      return STATE_SWEEP;
		}
  }
	return STATE_SEARCH;
}
/*
const int turnRightNow = 1200;
Robot_state ProcStateAdvance(RobotControl & control) {
  advance_distance += driveStraight();
	if (advance_distance > advance_target_distance) {
		advance_distance = 0;
		resetPController();
		// move out of beacon range manually
		setWheelsManuallyLR(searchSpeed,-searchSpeed);
		wait1Msec(turnRightNow);
		turningRight = false;
		setWheelsManuallyLR(0,0);
		wait1Msec(turnRightNow);
		initializeTurningPController(turningRight, searchSpeed);
		return STATE_SEARCH;
	}
	return STATE_ADVANCE;
}
*/
/*
Robot_state ProcStateSweep(RobotControl & control) {
  distance_sweeped += TurnPerfectly();
  if (control.deltaLight > deltaLightMax) {
		deltaLightMax = control.deltaLight;
		encoderAtPinpoint = distance_sweeped;
	}
	else if (!control.beaconFound) {
		turningRight = !turningRight;
		resetPController();
		initializeTurningPController(turningRight,slowTurningSpeed);
		distanceToSweepBack = distance_sweeped - encoderAtPinpoint;
		distance_sweeped = 0;
		return STATE_PINPOINT;
	}
	else if (SonarLessThanEqual(closeEnoughToTarget)) {
		return STATE_IDLE;
	}
	return STATE_SWEEP;
}
*/
/*
Robot_state ProcStatePinpoint(RobotControl & control) {
	distance_sweeped += TurnPerfectly();
	if (distance_sweeped > distanceToSweepBack) {
		resetPController();
		turningRight = true;
		initializeForwardPController(WalkingSpeed);
		return STATE_WALK;
	}
	else if (SonarLessThanEqual(closeEnoughToTarget)) {
		return STATE_IDLE;
	}
	return STATE_PINPOINT;
}*/

/*
Robot_state ProcStateWalk(RobotControl & control) {
  driveStraight();
	if (SonarLessThanEqual(closeEnoughToTarget)) {
		return STATE_CLAWOPEN;
	}
  if (!control.beaconFound) {
    resetPController();
    initializeTurningPController(turningRight, searchSpeed);
    return STATE_SEARCH;
  }
	return STATE_WALK;
}
*/
// OPEN CLAW (AND TOGGLE CLAW STATE) AND STOP CLAW MOTOR AND GOTO: APPROACH STATE
Robot_state ProcStateClawOpen(RobotControl & control) {
	setClawSpeed(ClawSpeed);
	control.toggle_flag = !control.toggle_flag;
	wait1Msec(ClawTime);
	setClawSpeed(0);
	return STATE_APPROACH;
}

// DRIVE FORWARD AND STOP ALL MOTORS FOR APPROACHTIME MILLISECONDS AND GOTO: BACKUP STATE
const int ApproachTime = 400;
const int ApproachSpeed = 30;
Robot_state ProcStateApproach(RobotControl & control) {
	setWheelsManuallyLR(ApproachSpeed,ApproachSpeed);
	wait1Msec(ApproachTime);
	stopAllMotors();
	return STATE_BACKUP;
}

// BACK UP AND STOP ALL MOTORS FOR BACKUPTIME MILLISECONDS AND GOTO: CLAWCLOSE STATE
const int BackupTime = 600;
const int BackupSpeed = 30;
Robot_state ProcStateBackup(RobotControl & control) {
	setWheelsManuallyLR(-BackupSpeed,-BackupSpeed);
	wait1Msec(BackupTime);
	stopAllMotors();
	return STATE_CLAWCLOSE;
}

// CLOSE THE CLAW AND TOGGLE IT'S CURRENT STATE FOR CLAWTIME MILLISECONDS AND GOTO: IDLE STATE
Robot_state ProcStateClawClose(RobotControl & control) {
	setClawSpeed(-ClawSpeed);
	control.toggle_flag = !control.toggle_flag;
	wait1Msec(ClawTime);
	setClawSpeed(0);
	return STATE_IDLE;
}

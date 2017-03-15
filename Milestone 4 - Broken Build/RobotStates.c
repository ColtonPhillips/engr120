#include "Devices.c"
#include "LightDetector.c"

const short tooCloseToTarget =  15;
const short closeEnoughToTarget = 25;

void ProcBeforeAnyStateRuns(Robot_state state, RobotControl & control) {
	if (state != STATE_CLAWSETUP) {
 	 monitorButtonsAndLimitSwitches(control);
	}
  control.beaconFound = monitorLight(control);
  setLED1If(control.beaconFound);
  setLED2If(control.beaconFound && SonarGreaterThan(closeEnoughToTarget));
  setLED3If(control.beaconFound && SonarLessThan(tooCloseToTarget));
		return;
  }

  const short turningSpeed = 50;
bool turningRight = true;
int deltaLightMax = 0;
int encoderAtPinpoint = 0;

Robot_state ProcStateIdle(RobotControl & control) {
  stopAllMotors();
  if (control.button1_pushed) {
		control.button1_pushed = false;
    initializeTurningPController(turningRight, turningSpeed);
		return STATE_SEARCH;
	}
	if (control.button2_pushed) {
		control.button2_pushed = false;
		return STATE_CLAWSETUP;
	}
  return STATE_IDLE;
}
const int ClawSpeed = 20;
const int ClawTime = 400;
Robot_state ProcStateClawSetup(RobotControl & control) {
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

int distanceToSweepBack = 0;
const short WalkingSpeed = 30;
const int twelveDegreesInTicks = 120;
int distance_sweeped = 0;
const int slowTurningSpeed = 35;
//int distance_turned = 0;
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
		initializeTurningPController(turningRight, turningSpeed);
	}
	else if (control.beaconFound) {
    distance_turned = 0;
		turningDistance = twelveDegreesInTicks;
		resetPController();
    initializeTurningPController(turningRight, slowTurningSpeed);
		deltaLightMax = 0;
		distanceToSweepBack = 0;
		distance_sweeped = 0;
		return STATE_SWEEP;
	}
	return STATE_SEARCH;
}

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
		deltaLightMax = 0;
		return STATE_PINPOINT;
	}
	return STATE_SWEEP;
}

//int distance_sweeped_to_left = 0;
//int distance_sweeped_to_right = 0;
Robot_state ProcStatePinpoint(RobotControl & control) {
	distance_sweeped += TurnPerfectly();
	if (distance_sweeped > distanceToSweepBack) {
		resetPController();
		initializeForwardPController(WalkingSpeed);
		distance_sweeped = 0;
		deltaLightMax = 0;
		turningRight = true;
		return STATE_WALK;
	}
	return STATE_PINPOINT;
}
/*
bool is_right_distance = true;
Robot_state ProcStateFind1(RobotControl & control) {
  distance_sweeped += TurnPerfectly();
  if (control.deltaLight > deltaLightMax) {
		deltaLightMax = control.deltaLight;
		encoderAtPinpoint = distance_sweeped;
		is_right_distance = true;
	}
	else if (!control.beaconFound) {
		turningRight = !turningRight;
		resetPController();
		initializeTurningPController(turningRight,slowTurningSpeed);
		distance_sweeped_to_right = distance_sweeped;
		distance_sweeped = 0;
		encoderAtPinpoint = 0;
		return STATE_FIND2;
	}
	return STATE_FIND1;
}
Robot_state ProcStateFind2(RobotControl & control) {
  distance_sweeped += TurnPerfectly();
  if (control.deltaLight > deltaLightMax) {
		deltaLightMax = control.deltaLight;
		encoderAtPinpoint = distance_sweeped;
		is_right_distance = false;
	}
	else if (!control.beaconFound) {
		turningRight = !turningRight;
		resetPController();
		initializeTurningPController(turningRight,slowTurningSpeed);
		distance_sweeped_to_left = distance_sweeped;
		if (is_right_distance) {
			distanceToSweepBack = distance_sweeped_to_left - distance_sweeped_to_right + encoderAtPinpoint;
		}
		else {
			distanceToSweepBack = distance_sweeped_to_left - encoderAtPinpoint;
		}
		distance_sweeped = 0;
		return STATE_FIND3;
	}
	return STATE_FIND2;
}

Robot_state ProcStateFind3(RobotControl & control) {
	distance_sweeped += TurnPerfectly();
	if (distance_sweeped > distanceToSweepBack) {
		resetPController();
		turningRight = true;
		//initializeForwardPController(WalkingSpeed);
		deltaLightMax = 0;
		encoderAtPinpoint = 0;
		distance_sweeped = 0;
		distanceToSweepBack = 0;
		distance_sweeped_to_right = 0;
		distance_sweeped_to_left = 0;
		return STATE_IDLE;
	}
	return STATE_FIND3;
}
*/
Robot_state ProcStateWalk(RobotControl & control) {
  driveStraight();
	if (SonarLessThanEqual(closeEnoughToTarget)) {
		resetPController();
		/*turningRight = true;
		initializeTurningPController(turningRight, turningSpeed);
		deltaLightMax = 0;
		encoderAtPinpoint = 0;
		distance_sweeped = 0;
		distanceToSweepBack = 0;
		distance_sweeped_to_right = 0;
		distance_sweeped_to_left = 0;*/
		return STATE_IDLE;
	}
  else if (!control.beaconFound) {
    resetPController();
    initializeTurningPController(turningRight, turningSpeed);
    return STATE_SEARCH;
  }
	return STATE_WALK;
}

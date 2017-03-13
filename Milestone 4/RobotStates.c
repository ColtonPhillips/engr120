#include "Devices.c"
#include "LightDetector.c"

const short tooCloseToTarget =  15;
const short closeEnoughToTarget = 25;

void ProcBeforeAnyStateRuns(RobotControl & control) {
  monitorButtonsAndLimitSwitches(control);
  control.beaconFound = monitorLight(control);
  setLED1If(control.beaconFound);
  setLED2If(control.beaconFound && SonarGreaterThan(closeEnoughToTarget));
  setLED3If(control.beaconFound && SonarLessThan(tooCloseToTarget)); //(SensorValue[Sonar] < tooCloseToTarget && SensorValue[Sonar] != -1));
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
  return STATE_IDLE;
}

int distanceToSweepBack = 0;
const short WalkingSpeed = 30;
const int twelveDegreesInTicks = 120;
int distance_sweeped = 0;
const int slowTurningSpeed = 30;

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
		zeroWheelEncoders();
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
		return STATE_PINPOINT;
	}
	else if (SonarLessThanEqual(closeEnoughToTarget)) {
		return STATE_IDLE;
	}
	return STATE_SWEEP;
}

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
}

Robot_state ProcStateWalk(RobotControl & control) {
  driveStraight();
	if (SonarLessThanEqual(closeEnoughToTarget)) {
		return STATE_IDLE;
	}
  if (!control.beaconFound) {
    resetPController();
    initializeTurningPController(turningRight, turningSpeed);
    return STATE_SEARCH;
  }
	return STATE_WALK;
}

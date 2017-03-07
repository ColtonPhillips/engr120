#include "Devices.c"
#include "LightDetector.c"

const short tooCloseToTarget =  15;
const short closeEnoughToTarget = 25;
void ProcBeforeAnyStateRuns(RobotControl & control) {
  monitorButtonsAndLimitSwitches(control);
  control.beaconFound = monitorLight();
  setLED1If(control.beaconFound);
  setLED2If(control.beaconFound && SonarGreaterThan(closeEnoughToTarget));
  setLED3If(control.beaconFound && SonarLessThan(tooCloseToTarget)); //(SensorValue[Sonar] < tooCloseToTarget && SensorValue[Sonar] != -1));
		return;
  }
const short turningSpeed = 50;
bool turningRight = true;
Robot_state ProcStateIdle(RobotControl & control) {
  stopAllMotors();
  if (control.button1_pushed) {
		control.button1_pushed = false;
    initializeTurningPController(turningRight, turningSpeed);
		return STATE_SEARCH;
	}
  return STATE_IDLE;
}

const short WalkingSpeed = 30;
const int sixDegreesInTicks = 60;
Robot_state ProcStateSearch(RobotControl & control) {
  static int distance_turned = 0;
	static int turningDistance = sixDegreesInTicks;
	distance_turned += TurnPerfectly();
	// We sweep back and forth until it is found, going further each time.
	if (distance_turned > turningDistance) {
    distance_turned = 0;
		turningDistance *= 2;
		turningRight = !turningRight;
		resetPController();
		initializeTurningPController(turningRight, turningSpeed);
	}
	if (control.beaconFound) {
    distance_turned = 0;
		turningDistance = sixDegreesInTicks;
		turningRight = true;
		resetPController();
    initializeForwardPController(WalkingSpeed);
		return STATE_WALK;
	}
	return STATE_SEARCH;
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

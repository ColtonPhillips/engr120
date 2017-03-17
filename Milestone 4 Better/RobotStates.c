#include "Devices.c"
#include "Constants.h"

// THIS BLOCK OF CODE HAPPENS EVERY STEP PRIOR TO ANY PARTICULAR STATES CODE RUNS
void ProcBeforeAnyStateRuns(Robot_state state, RobotControl & control) {
  if (state == STATE_IDLE) {monitorButtonsAndLimitSwitches(control);}
  control.beaconFound = monitorLight(control);
  //setLEDs(state & 1,state & (1 << 1),state & (1 << 2));
  Search_state st = control.searchState;
  setLEDs(st & 1,st & (1 << 1),st & (1 << 2));
  setLED1If(control.beaconFound);
}

// LEFT BUTTON: GOTO: SEARCH STATE
// RIGHT BUTTON: TOGGLE CLAW TO OPEN OR CLOSED
Robot_state ProcStateIdle(RobotControl & control) {
  stopAllMotors();
  if (control.button1_pushed) {
		control.button1_pushed = false;
    initializeTurningPController(RIGHT, searchSpeed); // turningRight == true
    searchControllerConstructor(control.searchControl);
		return STATE_SEARCH;
	}
	if (control.button2_pushed) {
		control.button2_pushed = false;
		return STATE_CLAWTOGGLE;
	}
  return STATE_IDLE;
}

// TOGGLE CLAW OPEN OR CLOSED AND GOTO: IDLE STATE
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

int distanceMovedThisStep = 0;
Robot_state ProcStateSearch(RobotControl & control) {
  distanceMovedThisStep = TurnPerfectly();
  control.searchControl.distanceSweeped += distanceMovedThisStep;
  switch(control.searchState) {
    case SEARCH_SEEKING_NO_SIGNAL_GOING_RIGHT:
      // we should always enter this method turningRight
      if (!control.beaconFound) {
        initializeTurningPController(LEFT,searchSpeed);
        searchControllerConstructor(control.searchControl);
        control.searchControl.seekingTurnaroundDistance = fourtyFiveDegreesInTicks;
        control.searchState = SEARCH_SEEKING_SIGNAL_GOING_LEFT;
      }
    break;
    case SEARCH_SEEKING_SIGNAL_GOING_LEFT:
      // we should always enter this method turningLeft
      if (control.beaconFound) {
        // ensure you're in a found beacon range by allowing it to move a teeny bit further
        wait1Msec(10);
        searchControllerConstructor(control.searchControl);
        control.searchState = SEARCH_SCANNING_GOING_LEFT;
      }
      else if (control.searchControl.distanceSweeped > control.searchControl.seekingTurnaroundDistance) {
        control.searchControl.seekingTurnaroundDistance *= 3;
        initializeTurningPController(RIGHT,searchSpeed);
        searchControllerConstructor(control.searchControl);
        control.searchState = SEARCH_SEEKING_SIGNAL_GOING_RIGHT;
      }
    break;
    case SEARCH_SEEKING_SIGNAL_GOING_RIGHT:
      // we should always enter this method turningRight
      if (control.beaconFound) {
        wait1Msec(10);
        searchControllerConstructor(control.searchControl);
        control.searchState = SEARCH_SCANNING_GOING_RIGHT;
      }
      else if (control.searchControl.distanceSweeped > control.searchControl.seekingTurnaroundDistance) {
        control.searchControl.seekingTurnaroundDistance *= 3;
        initializeTurningPController(LEFT,searchSpeed);
        searchControllerConstructor(control.searchControl);
        control.searchState = SEARCH_SEEKING_SIGNAL_GOING_LEFT;
      }
    break;
    case SEARCH_SCANNING_GOING_LEFT:
      // we should always enter this method turningLeft
      if (control.deltaLight > control.searchControl.deltaLightMaxScanned) {
    		control.searchControl.deltaLightMaxScanned = control.deltaLight;
        control.searchControl.encoderAtDeltaLightMax = control.searchControl.distanceSweeped;
    	}
      if (!control.beaconFound) {
        // distance value not reset when constructor called
        control.searchControl.distanceToEncoderAtDeltaLightMax =
          control.searchControl.distanceSweeped -
          control.searchControl.encoderAtDeltaLightMax;
        searchControllerConstructor(control.searchControl);
        initializeTurningPController(RIGHT,searchSpeed);
         control.searchState = SEARCH_MOVE_TO_MAXIMA_GOING_RIGHT;
      }
    break;
    case SEARCH_SCANNING_GOING_RIGHT:
      // we should always enter this method turningRight
      if (control.deltaLight > control.searchControl.deltaLightMaxScanned) {
    		control.searchControl.deltaLightMaxScanned = control.deltaLight;
        control.searchControl.encoderAtDeltaLightMax = control.searchControl.distanceSweeped;
    	}
      if (!control.beaconFound) {
        control.searchControl.distanceToEncoderAtDeltaLightMax = // distance value not reset when constructor called
          control.searchControl.distanceSweeped -
          control.searchControl.encoderAtDeltaLightMax;
        searchControllerConstructor(control.searchControl);
        initializeTurningPController(LEFT,searchSpeed);
         control.searchState = SEARCH_MOVE_TO_MAXIMA_GOING_LEFT;
      }
    break;
    case SEARCH_MOVE_TO_MAXIMA_GOING_RIGHT:
    case SEARCH_MOVE_TO_MAXIMA_GOING_LEFT:
      if (control.searchControl.distanceSweeped > control.searchControl.distanceToEncoderAtDeltaLightMax) {
        control.searchControl.distanceToEncoderAtDeltaLightMax = 0;
        stopAllMotors();
        initializeForwardPController(WalkingSpeed);
        control.searchState = SEARCH_SEEKING_NO_SIGNAL_GOING_RIGHT;
        control.distanceAdvanced = 0;
        control.distanceToAdvanceInTicks = OneCentimeterWalkedInTicks * (SonarValueFiltered() * 0.66);
        return STATE_ADVANCE;
      }
    break;
    default:
  }
  return STATE_SEARCH;
}

Robot_state ProcStateAdvance(RobotControl & control) {
  control.distanceAdvanced += driveStraight();
  if (control.distanceAdvanced > control.distanceToAdvanceInTicks) {
    stopAllMotors();
    initializeTurningPController(RIGHT, searchSpeed);
    return STATE_SEARCH;
  }
  else if (SonarLessThanEqual(closeEnoughToTarget)) {
    stopAllMotors();
    resetPController();
    return STATE_IDLE;
  }
  return STATE_ADVANCE;
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
		initializeTurningPController(turningRight, searchSpeed);
	}
	else if (control.beaconFound) {
		distance_turned = 0;
		turningDistance = twelveDegreesInTicks;
		deltaLightMax = 0;
		distanceToSweepBack = 0;
		distance_sweeped = 0;
		if (SonarGreaterThan(50)) {
			initializeForwardPController(WalkingSpeed);
			advance_distance = 0;
			advance_target_distance = SonarValueFiltered()- 65;
			return STATE_ADVANCE;
		}
		else {
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
Robot_state ProcStateApproach(RobotControl & control) {
	setWheelsManuallyLR(ApproachSpeed,ApproachSpeed);
	wait1Msec(ApproachTime);
	stopAllMotors();
	return STATE_BACKUP;
}

// BACK UP AND STOP ALL MOTORS FOR BACKUPTIME MILLISECONDS AND GOTO: CLAWCLOSE STATE
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

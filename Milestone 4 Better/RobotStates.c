#include "Devices.c"
#include "Constants.h"

// THIS BLOCK OF CODE HAPPENS EVERY STEP PRIOR TO ANY PARTICULAR STATES CODE RUNS
void ProcBeforeAnyStateRuns(Robot_state state, RobotControl & control) {
  if (state == STATE_IDLE) {monitorButtonsAndLimitSwitches(control);}
  control.beaconFound = monitorLight(control);
  //setLEDs(state & 1,state & (1 << 1),state & (1 << 2));
  //Search_state st = control.searchState;
  //setLEDs(st & 1,st & (1 << 1),st & (1 << 2));
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
    control.searchControl.movingToAdvance = true;
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
        stopAllMotors();
        control.searchState = SEARCH_SEEKING_NO_SIGNAL_GOING_RIGHT;
        control.distanceAdvanced = 0;
        control.searchControl.distanceToEncoderAtDeltaLightMax = 0;
        if (control.searchControl.movingToAdvance == true) {
          initializeForwardPController(WalkingSpeed);
          control.distanceToAdvanceInTicks = OneCentimeterWalkedInTicks * (SonarValueFiltered() * 0.66);
          return STATE_ADVANCE;
	      } else { // if not moving to advance, moving to approach (through claw open)
    	    // we want to align one last time before an approach
  	      return STATE_CLAWOPEN;
      	}
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
    initializeTurningPController(RIGHT,searchSpeed);
    control.searchControl.movingToAdvance = false; // moving to appproach through claw open
    return STATE_SEARCH;
  }
  return STATE_ADVANCE;
}

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

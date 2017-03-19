#ifndef PCONTROL_H
#define PCONTROL_H

// Proportional controller code:

// THE RIGHT WHEEL IS ESTIMATED TO RUN FIVE TICKS SLOWER
// MASTER POWER IS THE LEFT WHEEL
// SLAVE POWER IS THE RIGHT WHEELS SLOWER SO THERE ISNT A HUGE ERROR IMMEDIATELY
const int roughGuess = 5;
const int kp = 5;
int error = 0;
int slavePower = 0;
int masterPower = 0;
// RESET P CONTROLLER AFTER LEAVING A STATE THAT USES THE P CONTROLLER
void resetPController() {
  error = slavePower = masterPower = 0;
  zeroWheelEncoders();
  stopAllMotors();
}

// INITIALIZE P CONTROLLER for forward movement BEFORE ENTERING A STATE THAT USES THE P CONTROLLER
void initializeForwardPController(short speed) {
  resetPController();
  masterPower = speed;
  slavePower = speed - roughGuess;
  error = 0;
  zeroWheelEncoders();
  time1[T2] = 0;
  clearTimer(T2);
}

// INITIALIZE P CONTROLLER for turning movement BEFORE ENTERING A STATE THAT USES THE P CONTROLLER
// speed variable should only ever be positive!
void initializeTurningPController(bool isTurningRight, short speed) {
  resetPController();
  if (isTurningRight) // L is + , R is -
    { masterPower = speed;
    	slavePower = -speed + roughGuess; } // - R
  else // R is + , L is -
    { masterPower = -speed;
    	slavePower = speed - roughGuess; } // + R
  error = 0;
  time1[T2] = 0;
  clearTimer(T2);
  zeroWheelEncoders();
}


// DRIVE COMPLETELY STRAIGHT
int driveStraight() {
	int dist = 0;
  setWheelsManuallyLR(masterPower,slavePower);
  if (time1[T2] > 100 ) {
    error = abs(getLeftWheelEncoder()) - abs(getRightWheelEncoder());
    slavePower += error / kp;
    dist = abs(getLeftWheelEncoder());
    zeroWheelEncoders();
  }
  return dist;
}

// TURN ON A DIME
int TurnPerfectly() {
  int delta = 0;
  setWheelsManuallyLR(masterPower,slavePower);
  if (time1[T2] > 50 ) {
    error = abs(getLeftWheelEncoder()) - abs(getRightWheelEncoder());
    if (slavePower > 0) { // R is -
      slavePower -= error / kp;
    }
    else if (slavePower < 0) { // R is +
      slavePower += error / kp;
    }
    delta = abs(getLeftWheelEncoder());
    zeroWheelEncoders();
  }
  return delta;
}
#endif

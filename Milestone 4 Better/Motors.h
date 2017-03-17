#ifndef MOTORS_H
#define MOTORS_H

// claw code:

// SET SPEED OF CLAW TO BE OPENING (POSITIVE) OR CLOSING (NEGATIVE)
// WARNING: Do not pass a high value into this function or risk shorting motor!
void setClawSpeed(short speed) {
	motor[MClaw] = speed;
}

// wheels code:

// CALL TO SET MOTORS WITHOUT CONSIDERING AN OFFSET OR A PID CONTROLLER
void setWheelsManuallyLR(short LSpeed, short RSpeed) {
	motor[MLeft] = LSpeed;
	motor[MRight] = RSpeed;
}

// STOP ALL MOTORS
void stopAllMotors() {
	setWheelsManuallyLR(0,0);
	motor[MClaw] = 0;
}

// encoder code:

// GET LEFT WHEEL ENCODER TICK VALUE
short getLeftWheelEncoder()
{
  return getMotorEncoder(MLeft);
}
// GET RIGHT WHEEL ENCODER TICK VALUE
short getRightWheelEncoder()
{
  return getMotorEncoder(MRight);
}
//ZERO LEFT ENCODER
void zeroLeftWheelEncoder() {
  resetMotorEncoder(MLeft);
}
// ZERO RIGHT ENCODER
void zeroLeftWheelEncoder() {
  resetMotorEncoder(MRight);
}
// RESET BOTH WHEEL MOTOR ENCODERS TO ZERO
void zeroWheelEncoders() {
			resetMotorEncoder(MLeft);
			resetMotorEncoder(MRight);
}

#endif

#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  Button1,        sensorTouch)
#pragma config(Sensor, dgtl2,  Button2,        sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           MLeft,         tmotorVex393_HBridge, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port2,           MClaw,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          MRight,        tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
// WIRING DETAILS
// RIGHT WHEEL: PIN 10
// LEFT WHEEL: PIN 1
// IPC1: 								RIGHT WHEEL=POSITIVE
// ICP2(DAISY CHAINED):  LEFT WHEEL=NEGATIVE
// CLAW MOTOR: MOTOR PIN 2
// BUTTON1: DIGITAL 1
// BUTTON2: DIGITAL 2
#include "Devices.c"
#include "RobotStates.c"
// PROGRAM STARTS HERE
task main()
{
  // ROBOT BEGINS IN IDLE STATE
  T_state state;
  state  = STATE_IDLE;
  // ROBOT BEGINS WITH THESE DATA FIELDS SET
  RobotControl control;
   while (true)
  {
    switch(state) // STATE MACHINE
    {
      case STATE_IDLE:
      	state = ProcStateIdle(control);
      break;
    	default: // We should never be in this state.
    }
  }
}
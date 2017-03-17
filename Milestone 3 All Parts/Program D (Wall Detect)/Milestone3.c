#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    IRSensor,       sensorReflection)
#pragma config(Sensor, dgtl1,  Button1,        sensorTouch)
#pragma config(Sensor, dgtl2,  Button2,        sensorTouch)
#pragma config(Sensor, dgtl3,  LimitLeft,      sensorTouch)
#pragma config(Sensor, dgtl4,  LimitRight,     sensorTouch)
#pragma config(Sensor, dgtl8,  Sonar,          sensorSONAR_cm)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           MLeft,         tmotorVex393_HBridge, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port2,           MClaw,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          MRight,        tmotorVex393_HBridge, openLoop, encoderPort, None)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// WIRING DETAILS
// IR SENSOR: ANALOG PIN 1
// RIGHT WHEEL: PIN 10
// LEFT WHEEL: PIN 1
// IPC1: 								RIGHT WHEEL=POSITIVE
// ICP2(DAISY CHAINED):  LEFT WHEEL=NEGATIVE
// CLAW MOTOR: MOTOR PIN 2
// BUTTON1: DIGITAL 1
// BUTTON2: DIGITAL 2
// SONAR: DIGITAL 8 and 9
// Limit Left and Limit Right: 3 and 4
#include "Devices.c"
#include "RobotStates.c"
// PROGRAM STARTS HERE
// wall detection program
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
      case STATE_WALK:
      	state = ProcStateWalk(control);
      break;
      case STATE_BACKAWAY:
      	state = ProcStateBackaway(control);
      break;
    	default: // We should never be in this state.
    }
  }
}
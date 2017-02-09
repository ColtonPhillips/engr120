#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, dgtl1,  Button1,        sensorTouch)
#pragma config(Sensor, dgtl2,  Button2,        sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           MLeft,         tmotorVex393_HBridge, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port2,           MClaw,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          MRight,        tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "Robot.h"
#include "RobotStates.c"
// BUTTON 1 AND 2 ARE SET TO TRUE HERE
void monitorInput( RobotControl & control)
{
  if(SensorValue(Button1) && !control.button1_pushed)
    control.button1_pushed = true;

  if(SensorValue(Button2) & !control.button2_pushed)
    control.button2_pushed = true;
}
// PROGRAM STARTS HERE
task main()
{
  // ROBOT BEGINS IN IDLE STATE
  T_state state;
  state  = STATE_IDLE;
  // ROBOT BEGINS WITH THESE DATA FIELDS SET
  RobotControl control;
  control.button1_pushed = false;
  control.button2_pushed = false;
  while (true)
  {
  	// POLL BUTTON 1 AND BUTTON 2

    switch(state) // STATE MACHINE
    {
      case(STATE_IDLE):
      	monitorInput(control);
      	state = ProcessStateIdle(control);
      break;

      case(STATE_OPENING):
      	state = ProcessStateOpening(control);
    	break;

      case(STATE_CLOSING):
      	state = ProcessStateClosing(control);
    	break;

    	default: // We should never be in this state.
    }
  }
}

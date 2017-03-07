#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    IRSensor,       sensorReflection)
#pragma config(Sensor, dgtl1,  Button1,        sensorTouch)
#pragma config(Sensor, dgtl2,  Button2,        sensorTouch)
#pragma config(Sensor, dgtl3,  LimitLeft,      sensorTouch)
#pragma config(Sensor, dgtl4,  LimitRight,     sensorTouch)
#pragma config(Sensor, dgtl5,  LED1,           sensorDigitalOut)
#pragma config(Sensor, dgtl8,  Sonar,          sensorSONAR_cm)
#pragma config(Sensor, dgtl10, LED2,           sensorDigitalOut)
#pragma config(Sensor, dgtl11, LED3,           sensorDigitalOut)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           MLeft,         tmotorVex393_HBridge, openLoop, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port2,           MClaw,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          MRight,        tmotorVex393_HBridge, openLoop, encoderPort, None)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "Devices.c"
#include "RobotStates.c"
// PROGRAM STARTS HERE
task main()
{
  // ROBOT BEGINS IN IDLE STATE
  Robot_state state;
  state  = STATE_IDLE;
  // ROBOT BEGINS WITH THESE DATA FIELDS SET
  RobotControl control;
  initializeController(control);
   while (true)
  {
    ProcBeforeAnyStateRuns(control);
    switch(state) // STATE MACHINE
    {
      case STATE_IDLE:
      	state = ProcStateIdle(control);
      break;
      case STATE_SEARCH:
        state = ProcStateSearch(control);
      break;
      case STATE_WALK:
      	state = ProcStateWalk(control);
      break;
      default: // We should never be in this state.
    }
  }
}

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

// AUTHOR : COLTON PHILLIPS
//   THE MAIN TASK CONTROLS THE STATE MACHINE THAT DEFINES THE PROGRAM'S BEHAVIOR OVER TIME

// MACRO meta p r o g r a m m i n g WIZARDRY:
	//WARNING. TOTALLY COOL. (IT'S PRONOUNCED A "SWITCH TO" DESIGN PATTERN)
#define __to__(__X__, __Y__) case (__X__):\
	state = __Y__(control);\
	break

#include "RobotStates.c"

// INPUT: ROBOT_STATE. ROBOT_CONTROL.
// 	GOTO: PRE-PROCESSING TASKS
//	GOTO: STATE-SPECIFIC PROCESSING TASKS
// OUTPUT: CHANGED ROBOT_STATE AND ROBOT_CONTROL
task main()
{
  // GET A ROBOT STATE OBJECT
  Robot_state state = robotStateFactory();
  // GET A ROBOTIC CONTROLLER OBJECT
  RobotControl control; robotControlConstructor(control);
   while (true)
  {
    ProcBeforeAnyStateRuns(state, control);
    switch(state)
    {
      __to__(STATE_IDLE,		ProcStateIdle);
      __to__(STATE_CLAWTOGGLE,		ProcStateClawToggle);
      __to__(STATE_SEARCH,		ProcStateSearch);
      __to__(STATE_ADVANCE,		ProcStateAdvance);
      __to__(STATE_WALL,		ProcStateWall);
      __to__(STATE_APPROACH,		ProcStateApproach);
      default: // We should never be in this state.
    }
  }
}

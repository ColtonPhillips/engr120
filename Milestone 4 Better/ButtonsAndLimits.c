#include "RobotController.h"
// buttons code:

// A FUNCTION TO MONITOR IF THE STATE OF A BUTTON MOVES FROM 0 TO 1
void monitorButtons( RobotControl & control)
{
  if(SensorValue(Button1) && !control.button1_pushed)
    control.button1_pushed = true;

  if(SensorValue(Button2) & !control.button2_pushed)
    control.button2_pushed = true;
}

// limit switches code:

// A FUNCTION TO MONITOR IF THE STATE OF A LIMIT SWITCH MOVES FROM 0 TO 1
void monitorLimitSwitches( RobotControl & control)
{
  if(SensorValue(LimitLeft) & !control.limitLeft_pushed)
    control.limitLeft_pushed = true;

  if(SensorValue(LimitRight) & !control.limitRight_pushed)
    control.limitRight_pushed = true;
}

// MONITOR BOTH THE BUTTONS AND LIMIT SWITCHES
void monitorButtonsAndLimitSwitches( RobotControl & control )
{
  monitorButtons(control);
  monitorLimitSwitches(control);
}

// RETURN TRUE IF ANY LIMIT SWITCH IS PRESSED
bool anyLimitSwitchHit(RobotControl & control) {
	if (control.limitRight_pushed || control.limitLeft_pushed)
    {  return true;  }
	else
    { return false; }
}
// SET THE LIMIT SWITCHES' CONTROL VARIABLES TO FALSE
void setLimitSwitchesFalse(RobotControl & control) {
  control.limitLeft_pushed = false;
  control.limitRight_pushed = false;
}

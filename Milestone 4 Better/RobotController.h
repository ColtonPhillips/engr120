#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H
struct ROBOTCONTROLLER;

// DATA COMPUTED, PASSED INTO AND POTENTIALLY ALTERED IN VARIOUS STATES
typedef struct ROBOTCONTROLLER {
	bool button1_pushed;     // button is pressed
	bool button2_pushed;
	bool toggle_flag; // if claw is opening or closing
	bool limitLeft_pushed;   // limit switch is pressed
	bool limitRight_pushed;
  bool beaconFound;        // is the guy facing the beacon?
	bool targetClose;        // is the beacon nearby (in range) to the guy

  int lightLevel; //  THE AMOUNT OF LIGHT
  int maxLight; // HIGHEST LIGHT IN PAST 100 MILLISECONDS
  int minLight; // LOWEST LIGHT IN PAST 100 MILLISECONDS
	int deltaLight;	// MAXLIGHT - MINLIGHT

} RobotControl, * ROBOTCONTROLLER_PTR;

// CALLED ONCE AT THE START OF THE PROGRAM
void robotControlConstructor(RobotControl &control) {
  control.button1_pushed = false;
  control.button2_pushed = false;
  control.toggle_flag = false;
  control.limitLeft_pushed = false;
  control.limitRight_pushed = false;
  // these will be set to good values if we just leave the robot for 100 ms
  control.deltaLight = 0;
  control.lightLevel = 0;
  control.maxLight = 0;
  control.minLight = 0;
}

#endif

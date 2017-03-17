#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H
struct ROBOTCONTROLLER;

// DATA COMPUTED, PASSED INTO AND POTENTIALLY ALTERED IN VARIOUS STATES
typedef enum SearchState_tag
{
  SEARCH_SEEKING_NO_SIGNAL_GOING_RIGHT = 0,
  SEARCH_SEEKING_SIGNAL_GOING_LEFT,
  SEARCH_SEEKING_SIGNAL_GOING_RIGHT,
  SEARCH_SCANNING_GOING_LEFT,
  SEARCH_SCANNING_GOING_RIGHT,
  SEARCH_MOVE_TO_MAXIMA_GOING_RIGHT,
  SEARCH_MOVE_TO_MAXIMA_GOING_LEFT,
} Search_state;

typedef struct SEARCHCONTROLLER {
  // reset using Constructor function
	int distanceSweeped;
  int encoderAtRightSide;
  int deltaLightMaxScanned;
  int encoderAtDeltaLightMax;

  // manually reset
  int distanceToEncoderAtDeltaLightMax;
  int seekingTurnaroundDistance;
  bool movingToAdvance;

} SearchControl, * SEARCHCONTROLLER_PTR;

void searchControllerConstructor(SearchControl & sControl) {
  sControl.distanceSweeped = 0;
  sControl.encoderAtRightSide = 0;
  sControl.deltaLightMaxScanned = 0;
  sControl.encoderAtDeltaLightMax = 0;
}

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
	Search_state searchState; // substates within the search routine
  SearchControl searchControl; // state within a state
  int distanceAdvanced; // how far have we walked after search has been made
  int distanceToAdvanceInTicks;
} RobotControl, * ROBOTCONTROLLER_PTR;

// CALLED ONCE AT THE START OF THE PROGRAM
void robotControlConstructor(RobotControl &control) {
  control.button1_pushed = false;
  control.button2_pushed = false;
  control.toggle_flag = false;
  control.limitLeft_pushed = false;
  control.limitRight_pushed = false;
  control.distanceAdvanced = 0;
  control.distanceToAdvanceInTicks = 0;
  // these will be set to good values if we just leave the robot for 100 ms
  control.deltaLight = 0;
  control.lightLevel = 0;
  control.maxLight = 0;
  control.minLight = 0;
  // init the search state machine for the search state machine so you can search while you search
  control.searchState = SEARCH_SEEKING_NO_SIGNAL_GOING_RIGHT;
  searchControllerConstructor(control.searchControl);
}

#endif

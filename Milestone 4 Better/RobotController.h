#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H
// THE SEARCH STATE HAS IT'S OWN ENTIRE STATE MACHINE
// IT SHOULD ALWAYS BE ENTERED USING THE 0TH STATE (SEEKING NO SIGNAL RIGHT)
typedef enum SearchState_tag
{
  // SEEKING MOVES THE NEEDLE (ROBOT) INTO THE RIGHT POSITION TO SCAN
  SEARCH_SEEKING_NO_SIGNAL_GOING_RIGHT = 0,
  SEARCH_SEEKING_SIGNAL_GOING_LEFT,
  SEARCH_SEEKING_SIGNAL_GOING_RIGHT,
  // SCANNING WILL RECORD THE IR LIGHT DELTA MAX FOR THE ENTIRE RANGE OF FOUND BEACON LIGHT
  SEARCH_SCANNING_GOING_LEFT,
  SEARCH_SCANNING_GOING_RIGHT,
  // OUT OF THE SCAN, MOVE TO THE BEST SPOT!
  SEARCH_MOVE_TO_MAXIMA_GOING_RIGHT,
  SEARCH_MOVE_TO_MAXIMA_GOING_LEFT,
  // TRANSITION TO ADVANCE OR APPROACH DEPENDING ON THE VALUE OF movingToAdvance
} Search_state;

// THIS CONTROL OBJECT IS USED IN SEARCH STATE.
// SOME OF THE VARIABLES ARE REUSED IN VARIOUS SUBSTATES. (A)
// SOME OF THE VARIABLES ARE COMPUTED ONCE AND PASSED ON TO FURTHER SUBSTATES (B)
typedef struct SEARCHCONTROLLER {
  // (A) reset these values often using Constructor function
	int distanceSweeped; // re-used function for any time the search controller is turning
  int deltaLightMaxScanned; // what is the max IR light change (see project description) when sweeping
  int encoderAtDeltaLightMax; // what encoder value was recorded for that highest scanned point

  // (B) (re)set these values manually when appropriate
  int distanceToEncoderAtDeltaLightMax; // how far from the starting point is the highest IR light delta at
  int seekingTurnaroundDistance; // how far does the robot have to turn to get to the best spot
  bool movingToAdvance; // the search state can transition to the STATE_ADVANCE if this is true
                        // otherwise it will move to STATE_APPROACH

} SearchControl, * SEARCHCONTROLLER_PTR;

//
void searchControllerConstructor(SearchControl & sControl) {
  sControl.distanceSweeped = 0;
  sControl.deltaLightMaxScanned = 0;
  sControl.encoderAtDeltaLightMax = 0;
}

// THIS IS THE DATA STRUCTURE PASSED AROUND TO EACH STATE
struct ROBOTCONTROLLER;
typedef struct ROBOTCONTROLLER {
	bool button1_pushed;         // Left button is pressed
	bool button2_pushed;         // right button is pressed
	bool claw_toggle_flag;       // if claw is opening vs is closing
	bool limitLeft_pushed;       // Left limit switch is pressed
	bool limitRight_pushed;      // Left limit switch is pressed

// COMPUTED IN MONITOR_LIGHT
  int lightLevel;             //  THE AMOUNT OF LIGHT (NOT USED)
  int maxLight;               // HIGHEST LIGHT IN PAST 100 MILLISECONDS (NOT USED)
  int minLight;               // LOWEST LIGHT IN PAST 100 MILLISECONDS (NOT USED)
	int deltaLight;	            // MAXLIGHT - MINLIGHT
  bool beaconFound;           // is the robot facing the beacon

  // STATE_SEARCH STUFF
	Search_state searchState; // substates within the search routine
  SearchControl searchControl; // state within a state

  // STATE_ADVANCE STUFF
  int distanceAdvanced; // how far have we walked after search has been made
  int distanceToAdvanceInTicks;
} RobotControl, * ROBOTCONTROLLER_PTR;

// CALLED ONCE AT THE START OF THE PROGRAM
void robotControlConstructor(RobotControl &control) {
  control.button1_pushed = false;
  control.button2_pushed = false;
  control.claw_toggle_flag = false;
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

const int light_threshold = 80;

// Perform processing of measurements.
// Should be called with rate of at least 20 Hertz for proper detection of puck.
bool monitorLight()
{
	static int minLevelIR1 = 4096;	// Minimum light level seen by IR sensor 1
	static int maxLevelIR1 = 0;			// Maximum light level seen by IR sensor 1
	static int diffLevelIR1 = 0;		// Delta between maximum and minimum seen in last 0.1 seconds

	int lightLevel1 = SensorValue[IRSensor];
	bool returnValue;

	// Check if 100 msecs have elapsed.
	if ( time1[T1] > 100 )  {

	  // 100 msecs have elapsed.  Compute delta of light level.
		diffLevelIR1 = maxLevelIR1 - minLevelIR1;

		// Reset calculation for next 100 msecs.
		maxLevelIR1 = 0;
		minLevelIR1 = 4096;
		clearTimer(T1);

	} else {

	  // Check for new minimum/maximum light levels.
	  if ( lightLevel1 < minLevelIR1 ) {
	  	minLevelIR1 = lightLevel1;
	  } else if ( lightLevel1 > maxLevelIR1 ) {
	    maxLevelIR1 = lightLevel1;
	  }
	}

	// Check if light level difference over threshold.
	if ( diffLevelIR1 > light_threshold ) {
		returnValue = true;
	} else {
	  returnValue = false;
	}

	return(returnValue);
}
/*
task main()
{
	T_State robot_state = LIGHT_OFF;
	bool beaconVisible;

	while( true ) {

	  // Update sensor values (must be called at least 20 times a second for proper performance).
		beaconVisible = monitorLight();

		switch( robot_state ) {
			case LIGHT_OFF:
				SensorValue[StateLED] = 0;
				if ( beaconVisible ) {
					robot_state = LIGHT_ON;
				} else {
				  robot_state = LIGHT_OFF;
				}
				break;
			case LIGHT_ON:
				SensorValue[StateLED] = 1;
				if ( beaconVisible ) {
					robot_state = LIGHT_ON;
			  } else {
			    robot_state = LIGHT_OFF;
			  }
			  break;
			default:
				// This should never happen.
			  robot_state = LIGHT_OFF;

		} // switch( robot_state)

	}  // while(true)

}

*/

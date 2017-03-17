const int light_threshold = 50;

// Perform processing of measurements.
// Should be called with rate of at least 20 Hertz for proper detection of puck.
bool monitorLight(RobotControl & control)
{
	static int minLevelIR1 = 4096;	// Minimum light level seen by IR sensor 1
	static int maxLevelIR1 = 0;			// Maximum light level seen by IR sensor 1
	static int diffLevelIR1 = 0;		// Delta between maximum and minimum seen in last 0.1 seconds

	int lightLevel1 = SensorValue[IRSensor];
	control.lightLevel = lightLevel1;
	bool returnValue;

	// Check if 100 msecs have elapsed.
	if ( time1[T1] > 100 )  {

	  // 100 msecs have elapsed.  Compute delta of light level.
		diffLevelIR1 = maxLevelIR1 - minLevelIR1;
		control.deltaLight = diffLevelIR1;
		control.maxLight = maxLevelIR1;
		control.minLight = minLevelIR1;

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

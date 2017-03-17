// sonar code:

// RETURN !raw! SENSOR VALUE: -1 if nothing is heard
short SonarValue() {
	return SensorValue[Sonar];
}
// RETURN !raw! SENSOR VALUE: -1 if nothing is heard
short SonarValueFiltered() {
	short s = SensorValue[Sonar];
	if (s == -1 ) s = 100;
	return s;
}
// RETURNS TRUE OR FALSE IF THE SONAR VALUE IS PERCEIVED TO BE IN THESE RANGES:
bool SonarGreaterThan(short dist) {
	if (SensorValue[Sonar] > dist || SensorValue[Sonar] == -1) {return true;}
	else {return false;}
}
bool SonarLessThan(short dist) {
	if (SensorValue[Sonar] < dist && SensorValue[Sonar] != -1) {return true;}
	else {return false;}
}
bool SonarGreaterThanEqual(short dist) {
	if (SensorValue[Sonar] >= dist || SensorValue[Sonar] == -1) {return true;}
	else {return false;}
}
bool SonarLessThanEqual(short dist) {
	if (SensorValue[Sonar] <= dist && SensorValue[Sonar] != -1) {return true;}
  else {return false;}
}

// L.E.D. code:

// SET ALL THE LEDS
void setLEDs(int light1, int light2, int light3) {
	SensorValue[LED1] = light1;
	SensorValue[LED2] = light2;
	SensorValue[LED3] = light3;
}
// SET LED1 IF PASSED TRUE AS A PARAMETER, ELSE FALSE : LED2 + LED3 AS ETC.
void setLED1If(bool t) {
	if (t) {SensorValue[LED1] = 1;}
	else {SensorValue[LED1] = 0;}
}
void setLED2If(bool t) {
	if (t) {SensorValue[LED2] = 1;}
	else {SensorValue[LED2] = 0;}
}
void setLED3If(bool t) {
	if (t) {SensorValue[LED3] = 1;}
	else {SensorValue[LED3] = 0;}
}


configuration RHSensorC
{
	provides interface RHSensor;
}
implementation
{
	components RHSensorP,LedsC;//StdOutC;

	RHSensor = RHSensorP;
	RHSensorP.Leds -> LedsC;
//	RHSensorP.StdOut -> StdOutC;
}
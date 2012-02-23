

interface RHSensor
{

	command uint16_t readTemp();
	command uint16_t readHumidity();
	event error_t readTempDone(error_t result,uint16_t temp);
	event error_t readHumidityDone(error_t result,uint16_t humi);
	command void RH_Init();

}
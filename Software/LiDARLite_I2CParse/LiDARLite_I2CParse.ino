#include <Margay.h>

String Header = ""; //Information header
uint8_t I2CVals[0] = {}; 
unsigned long UpdateRate = 300; //Number of seconds between readings 

Margay Logger(Model_2v0);

void setup() {
	// Header = Header + RH.GetHeader();
	Logger.begin(I2CVals, sizeof(I2CVals), Header); //Pass header info to logger
	Init();
}

void loop() {
	Logger.Run(Update, UpdateRate);

}

String Update()
{
	Init();
	delay(200); //DEBUG!
	return ReadI2CData();
}

void Init() 
{
	Wire.begin(); 
	Wire.beginTransmission(0x40);
	Wire.write(0x01);
	Wire.write(0x00); //Set for high sensitivity
	Wire.endTransmission();
}


String ReadI2CData()
{
	const unsigned long Timeout = 200; //Wait up to 200ms for a new reading
	unsigned long LocalTime = millis(); //Keep track of local time for timeout

	uint8_t Data1 = 0;
	uint8_t Data2 = 0;
	uint8_t ADR = 0x40; //FIX! Make global const! 
	
	int16_t DataSet[4] = {0}; //Used to store set of data read in from I2C

	for(int i = 0; i < 4; i++) {
		Wire.beginTransmission(ADR);
		Wire.write(2*i + 2);
		Wire.endTransmission();
		Wire.requestFrom(ADR, 1);
		Data1 = Wire.read();

		Wire.beginTransmission(ADR);
		Wire.write(2*i + 3);
		Wire.endTransmission();
		Wire.requestFrom(ADR, 1);
		Data2 = Wire.read();

		DataSet[i] = ((Data2 << 8) | Data1); //Store data in array
	}
	
	return String(DataSet[0]) + "," + String(DataSet[1]) + "," + String(DataSet[2]) + "," + String(DataSet[3]) + ","; 
}

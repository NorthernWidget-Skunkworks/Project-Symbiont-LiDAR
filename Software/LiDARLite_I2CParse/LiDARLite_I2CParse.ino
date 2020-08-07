#include <Margay.h>

String Header = ""; //Information header
uint8_t I2CVals[1] = {0x50};
unsigned long UpdateRate = 300; //Number of seconds between readings

Margay Logger(Model_2v0);

void setup() {
	Header = Header + "Range [cm], Pitch [deg], Roll [deg]";
	Logger.begin(I2CVals, sizeof(I2CVals), Header); //Pass header info to logger
	Init();
}

void loop() {
	Logger.Run(Update, UpdateRate);

}

String Update()
{
	Init();
	// delay(1000); //DEBUG!
	delay(200);
	return ReadI2CData();
}

void Init()
{
	Wire.begin();
	Wire.beginTransmission(0x50);
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
	uint8_t ADR = 0x50; //FIX! Make global const!

	int16_t DataSet[7] = {0}; //Used to store set of data read in from I2C

	for(int i = 0; i < 7; i++) {
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

  // Takes up memory but makes code easier to read. Could be more efficient.
  // Perhaps use pointers, or just comments?
  float Range = DataSet[0]

  // Convert g vals to floats for math
	float ValX = DataSet[1];
	float ValY = DataSet[2];
	float ValZ = DataSet[3];

	float OffsetX = DataSet[4];
	float OffsetY = DataSet[5];
	float OffsetZ = DataSet[6];

	float Pitch = 0;
	float Roll = 0;

	if(OffsetX == OffsetY && OffsetX == OffsetZ && OffsetX == 0) {  //Do not include angle calc for offset when offset is 0 (Results in NAN value)
		// Pitch = atan(-ValX/(sqrt(pow(ValY, 2) + pow(ValZ, 2))))*(180.0/3.14);
		Pitch = atan(-ValX/ValZ)*(180.0/3.14);
		Roll = atan(ValY/(sqrt(pow(ValX, 2) + pow(ValZ, 2))))*(180.0/3.14);
	}
	else {
		// Pitch = atan(-ValX/(sqrt(pow(ValY, 2) + pow(ValZ, 2))))*(180.0/3.14) - (atan(-OffsetX/(sqrt(pow(OffsetY, 2) + pow(OffsetZ, 2))))*(180.0/3.14));
		Pitch = atan(-ValX/ValZ)*(180.0/3.14) - (atan(-OffsetX/OffsetZ)*(180.0/3.14));
		Roll = atan(ValY/(sqrt(pow(ValX, 2) + pow(ValZ, 2))))*(180.0/3.14) - (atan(OffsetY/(sqrt(pow(OffsetX, 2) + pow(OffsetZ, 2))))*(180.0/3.14));
	}

	if(DataSet[0] < 0) DataSet[0] = -9999; //Check for invalid range
	if(DataSet[1] == DataSet[2] && DataSet[1] == DataSet[3] && DataSet[1] == -1) {  //If all values are -1 set pitch and roll out of range
		Pitch = -9999;
		Roll = -9999;
	}


	// return String(DataSet[0]) + "," + String(DataSet[1]) + "," + String(DataSet[2]) + "," + String(DataSet[3]) + ",";
	return String(DataSet[0]) + "," + String(Pitch) + "," + String(Roll) + ",";
}

#include <Margay.h>

String Header = ""; //Information header
uint8_t I2CVals[0] = {}; 
unsigned long UpdateRate = 60; //Number of seconds between readings 

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
	return ReadSerialData();
}

void Init() 
{
	Serial1.begin(9600); //Turn on auxilary serial at 9600
}

void ReadSerialData(float *Output)
{
	const unsigned long Timeout = 200; //Wait up to 200ms for a new reading
	unsigned long LocalTime = millis(); //Keep track of local time for timeout
	char Val = 0; //Val used to check for begining of data
	while(Val != 'R' && (millis() - LocalTime < Timeout)) Val = Serial1.read();  //Wait for new string of data or timeout
	if(Val == 'R') { //If good read, not timeout
		String Range = Serial1.readStringUntil('\n');
		String XAxis = Serial1.readStringUntil('\n');
		String YAxis = Serial1.readStringUntil('\n');
		String ZAxis = Serial1.readStringUntil('\n');

		XAxis = XAxis.substring(1);  //Trim leading character from strings before float conversion
		YAxis = YAxis.substring(1);
		ZAxis = ZAxis.substring(1);

		Output[0] = Range.toFloat();  //Set values
		Output[1] = XAxis.toFloat();
		Output[2] = YAxis.toFloat();
		Output[3] = ZAxis.toFloat();
	}
	else {
		Output[0] = -9999;  //Set failure condition value if timeout
		Output[1] = -9999;
		Output[2] = -9999;
		Output[3] = -9999;
	}
	
	
}

String ReadSerialData()
{
	const unsigned long Timeout = 200; //Wait up to 200ms for a new reading
	unsigned long LocalTime = millis(); //Keep track of local time for timeout
	char Val = 0; //Val used to check for begining of data
	while(Val != 'R' && (millis() - LocalTime < Timeout)) Val = Serial1.read();  //Wait for new string of data or timeout
	if(Val == 'R') { //If good read, not timeout
		String Range = Serial1.readStringUntil('\n');
		String XAxis = Serial1.readStringUntil('\n');
		String YAxis = Serial1.readStringUntil('\n');
		String ZAxis = Serial1.readStringUntil('\n');

		XAxis = XAxis.substring(1);  //Trim leading character from strings before float conversion
		YAxis = YAxis.substring(1);
		ZAxis = ZAxis.substring(1);

		return Range + "," + XAxis + "," + YAxis + "," + ZAxis + ",";
	}
	else {
		return "-9999,-9999,-9999,-9999"; //Return failure otherwise //FIX! Find more clean method
	}
	
	
}
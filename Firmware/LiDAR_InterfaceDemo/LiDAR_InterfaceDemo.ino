//Dyson_Driver_ShortWave.ino
#include "SlowSoftI2CMaster.h"
#include "WireS.h"
// #include <EEPROM.h> //DEBUG!
//Commands
#define CTRL_REG1_ADR 0x20
#define CTRL_REG2_ADR 0x21
#define CTRL_REG3_ADR 0x22
#define CTRL_REG4_ADR 0x23
#define TEMP_CFG_REG_ADR 0x1F
#define OUT_X_ADR 0x28 //Low byte
#define OUT_Y_ADR 0x2A
#define OUT_Z_ADR 0x2C

#define POWER_SW 2

// #define ACCEL_ADR 0x18
const int ACCEL_ADR = 0x18; //DEBUG!

#define LIDAR_ADR 0x62

#define READ 0x01
#define WRITE 0x00

#define BUF_LENGTH 64 //Length of I2C Buffer, verify with documentation 

// #define ADR_ALT 0x41 //Alternative device address


volatile uint8_t ADR = 0x40; //Use arbitraty address, change using generall call??
// const uint8_t ADR_Alt = 0x41; //Alternative device address  //WARNING! When a #define is used instead, problems are caused

unsigned int Config = 0; //Global config value

uint8_t Reg[10] = {0}; //Initialize registers
bool StartSample = true; //Flag used to start a new converstion, make a conversion on startup
const unsigned int UpdateRate = 5; //Rate of update

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_A2, PIN_A3, true);  //Initialize software I2C

volatile bool StopFlag = false; //Used to indicate a stop condition 
volatile uint8_t RegID = 0; //Used to denote which register will be read from
volatile bool RepeatedStart = false; //Used to show if the start was repeated or not

void setup() {
  // Serial.begin(115200); //DEBUG!
  // Serial.println("begin"); //DEBUG!
  // pinMode(ADR_SEL_PIN, INPUT_PULLUP);
  // if(!digitalRead(ADR_SEL_PIN)) ADR = ADR_Alt; //If solder jumper is bridged, use alternate address //DEBUG!
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  pinMode(POWER_SW, OUTPUT);
	digitalWrite(POWER_SW, HIGH); //Turn off output power //FIX??
  Wire.begin(ADR);  //Begin slave I2C
  Serial.begin(9600);
  Serial.println("START"); //DEBUG!
  // EEPROM.write(0, ADR);

  //Setup I2C slave
	Wire.onAddrReceive(addressEvent); // register event
	Wire.onRequest(requestEvent);     // register event
	Wire.onReceive(receiveEvent);
	Wire.onStop(stopEvent);

	
	pinMode(7, INPUT); //DEBUG! 
	pinMode(11, INPUT);
	pinMode(13, INPUT);
	delay(10);
	digitalWrite(POWER_SW, LOW); //Turn on power
  si.i2c_init(); //Begin I2C master
  InitAccel();
  InitLiDAR();
  digitalWrite(14, LOW);

}

void loop() {
	// static unsigned int Count = 0; //Counter to determine update rate
	// if(StartSample == true) {
	// 	//Read new values in
	// 	AutoRange_Vis();  //Run auto range
	// 	delay(800); //Wait for new sample
	// 	SplitAndLoad(0x0B, GetALS()); //Load ALS value
	// 	SplitAndLoad(0x0D, GetWhite()); //Load white value
	// 	SplitAndLoad(0x02, long(GetUV(0))); //Load UVA
	// 	SplitAndLoad(0x07, long(GetUV(1))); //Load UVB
	// 	SplitAndLoad(0x10, GetLuxGain()); //Load lux multiplier 
	// 	SplitAndLoad(0x13, GetADC(0));
	// 	SplitAndLoad(0x15, GetADC(1));
	// 	SplitAndLoad(0x17, GetADC(2));

	// 	StartSample = false; //Clear flag when new values updated  
	// }
	// if(Count++ == UpdateRate) {  //Fix update method??
	// 	StartSample = true; //Set flag if number of updates have rolled over 
	// 	Count = 0;
	// }

	// for(int i = 0; i < 128; i++) {
	// 	si.i2c_start((i << 1) | WRITE);
	// 	Serial.print(i, HEX);
	// 	Serial.print('\t');
	// 	Serial.println(si.i2c_write(0xFF)); //Write MSB
	// 	si.i2c_stop();
	// }
	// while(digitalRead(7), LOW); //Wait for updated values //DEBUG!
	// ReadByte(ACCEL_ADR, 0x27);
	// ReadWord(ACCEL_ADR, OUT_X_ADR);
	uint8_t Stat1 = ReadByte(ACCEL_ADR, 0x27); 
	uint8_t Stat2 = ReadByte(ACCEL_ADR, 0x07);
	// while(((Stat1 & 0x08) >> 3) != 1 || ((Stat2 & 0x08) >> 3) != 1 || ((Stat2 & 0x80) >> 7) != 1) {
	while(((Stat1 & 0x08) >> 3) != 1 || Stat2 != 0xFF) {
		Stat1 = ReadByte(ACCEL_ADR, 0x27);
		Stat2 = ReadByte(ACCEL_ADR, 0x07);
	}
	// while(((ReadByte(ACCEL_ADR, 0x27) & 0x08) >> 3) != 1 || (digitalRead(7) == LOW)); //Wait for updated values

	// Serial.println("START"); //DEBUG!
	// Serial.println(Stat1, BIN); //DEBUG!	
	// Serial.println(Stat2, BIN); //DEBUG!
	// Serial.print("\n\n"); //Newline return
	Serial.print('R'); //Preceed range value
	Serial.println(GetRange()); 
	GetG();
	// Serial.println(ReadByte(LIDAR_ADR, 0x0E)); //DEBUG! //READ RSSI

	// for(int i = 0; i < 3; i++) {
	// 	Serial.println(GetG(i));
	// }
	// Serial.println(ReadByte(ACCEL_ADR, 0x27), BIN); //DEBUG!	
	delay(1000);
}

// float GetAngle(uint8_t Axis)
// {
// 	float ValX = GetG(0); //Used to get g values
// 	float ValY = GetG(1);
// 	float ValZ = GetG(2);
//   float Val = 0;
//   switch(Axis) {
//     case(0):
//       Val = asin(ValX); 
//       break;
//     case(1):
//       Val = asin(ValY);
//       break;
//     case(2):
//       Val = acos(ValZ);
//       break;
//     case(3):
//       Val = atan(ValX/(sqrt(pow(ValY, 2) + pow(ValZ, 2))))*(180.0/3.14); //Return pitch angle
//       break;
//     case(4):
//       Val = atan(ValY/(sqrt(pow(ValX, 2) + pow(ValZ, 2))))*(180.0/3.14); //Return roll angle
//       break;
//   }
//   if(ValX == ValY && ValX == ValZ) Val = -9999; //Return error value is all vals are the same (1 in 6.87x10^10 likelyhood of occouring without error)
//   return Val; 
// }

uint8_t InitAccel() 
{
	// WriteByte(ACCEL_ADR, CTRL_REG1_ADR, 0x07);
	WriteByte(ACCEL_ADR, CTRL_REG1_ADR, 0x77); //Set for 100Hz output data rate //FIX! Set to low power initally??
	WriteByte(ACCEL_ADR, CTRL_REG4_ADR, 0x88); //Turn on high resolution mode //FIX! Setup to use self text
	WriteByte(ACCEL_ADR, CTRL_REG3_ADR, 0x10);
	WriteByte(ACCEL_ADR, TEMP_CFG_REG_ADR, 0x80);
}

float GetG()
{ 
	// uint8_t AxisADR = OUT_X_ADR + 2*Axis; //Add appropriate offset
	// int16_t Data = ReadWord(ACCEL_ADR, AxisADR);
	// return Data*(4.0/4096.0); //FIX! Make fixed integer! 
	// Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
	int16_t Axis[3] = {0}; //Initalize variables for x,y,z values

	bool Error = SendCommand(ACCEL_ADR, OUT_X_ADR | 0x80);
	si.i2c_stop(); 

	uint8_t Data[6] = {0}; //Init data
	si.i2c_start((ACCEL_ADR << 1) | READ);
	for(int i = 0; i < 6; i++) {
		Data[i] = si.i2c_read(false);
	}
	si.i2c_stop();
	for(int i = 0; i < 3; i++) {
		Axis[i] = (((int16_t)(Data[2*i + 1] << 8) | (int16_t)Data[2*i]) >> 4);
	}
	Serial.print('X'); Serial.println(Axis[0]);
	Serial.print('Y'); Serial.println(Axis[1]);
	Serial.print('Z'); Serial.println(Axis[2]);

	SplitAndLoad(0x04, Axis[0]);
	SplitAndLoad(0x06, Axis[1]);
	SplitAndLoad(0x08, Axis[2]);


	// return Data;
}

uint8_t InitLiDAR() 
{
	WriteByte(LIDAR_ADR, 0x02, 0x80);
	WriteByte(LIDAR_ADR, 0x04, 0x08);
	WriteByte(LIDAR_ADR, 0x12, 0x05);
	WriteByte(LIDAR_ADR, 0x1C, 0x00);
}	

float GetRange()
{
	WriteByte(LIDAR_ADR, 0x00, 0x01);
	// si.i2c_start((LIDAR_ADR << 1) | WRITE);
	// si.i2c_write(0x00); 
	// si.i2c_stop();
	// si.i2c_start((LIDAR_ADR << 1) | WRITE);
	// si.i2c_write(0x01); //Command to take measurment WITH correction bias 
	// si.i2c_stop();

	while((ReadByte(LIDAR_ADR, 0x01) & 0x01) == 1);
	int16_t Data = ReadWord_LE(LIDAR_ADR, 0x0F);
	SplitAndLoad(0x02, Data);
	return Data;

}

uint8_t SendCommand(uint8_t Adr, uint8_t Command)
{
    si.i2c_start((Adr << 1) | WRITE);
    bool Error = si.i2c_write(Command);
    return 1; //DEBUG!
}

uint8_t WriteWord(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
	si.i2c_start((Adr << 1) | WRITE);
	si.i2c_write(Command); //Write Command value
	si.i2c_write(Data & 0xFF); //Write LSB
	uint8_t Error = si.i2c_write((Data >> 8) & 0xFF); //Write MSB
	si.i2c_stop();
	return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteByte(uint8_t Adr, uint8_t Command, uint8_t Data)  //Writes value to 16 bit register
{
	Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
	si.i2c_start((Adr << 1) | WRITE);
	si.i2c_write(Command); //Write Command value
	uint8_t Error = si.i2c_write((Data) & 0xFF); //Write MSB
	si.i2c_stop();
	return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteWord_LE(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
	si.i2c_start((Adr << 1) | WRITE);
	si.i2c_write(Command); //Write Command value
	si.i2c_write((Data >> 8) & 0xFF); //Write MSB
	si.i2c_write(Data & 0xFF); //Write LSB
	si.i2c_stop();
	// return Error;  //Invert error so that it will return 0 is works
}

// uint8_t WriteConfig(uint8_t Adr, uint8_t NewConfig)
// {
// 	si.i2c_start((Adr << 1) | WRITE);
// 	si.i2c_write(CONF_CMD);  //Write command code to Config register
// 	uint8_t Error = si.i2c_write(NewConfig);
// 	si.i2c_stop();
// 	if(Error == true) {
// 		Config = NewConfig; //Set global config if write was sucessful 
// 		return 0;
// 	}
// 	else return -1; //If write failed, return failure condition
// }

int ReadByte(uint8_t Adr, uint8_t Command, uint8_t Pos) //Send command value, and high/low byte to read, returns desired byte
{
	bool Error = SendCommand(Adr, Command);
	si.i2c_rep_start((Adr << 1) | READ);
	uint8_t ValLow = si.i2c_read(false);
	uint8_t ValHigh = si.i2c_read(false);
	si.i2c_stop();
	Error = true; //DEBUG!
	if(Error == true) {
		if(Pos == 0) return ValLow;
		if(Pos == 1) return ValHigh;
	}
	else return -1; //Return error if read failed

}

int ReadByte(uint8_t Adr, uint8_t Command) //Send command value, and high/low byte to read, returns desired byte
{
	Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
	bool Error = SendCommand(Adr, Command);
	si.i2c_stop(); //DEBUG!
	si.i2c_start((Adr << 1) | READ);
	uint8_t Val = si.i2c_read(false);
	// uint8_t ValHigh = si.i2c_read(false);
	si.i2c_stop();
	Error = true; //DEBUG!
	if(Error == true) {
		return Val; //DEBUG!
	// 	if(Pos == 0) return ValLow;
	// 	if(Pos == 1) return ValHigh;
	}
	else return -1; //Return error if read failed

}

int16_t ReadWord(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
	// Command |= 0x80; //turn on auto increment //FIX!!! Remove for other I2C transactions 
	bool Error = SendCommand(Adr, Command);
	si.i2c_stop(); 
	// Serial.print("Error = "); Serial.println(Error); //DEBUG!
	// uint8_t Data[6] = {0}; //Init data
	si.i2c_start((Adr << 1) | READ);

	uint8_t ByteLow = si.i2c_read(false);  //Read in high and low bytes (big endian)
	uint8_t ByteHigh = si.i2c_read(false);
	si.i2c_stop();
	// if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
	// else return -1; //Return error if read failed
	return ((int16_t)(ByteHigh << 8) | (int16_t)ByteLow); //DEBUG!  //FIX! Right shift?? 
}

int ReadWord_LE(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
	bool Error = SendCommand(Adr, Command);
	si.i2c_stop();
	si.i2c_start((Adr << 1) | READ);
	uint8_t ByteHigh = si.i2c_read(false);  //Read in high and low bytes (big endian)
	uint8_t ByteLow = si.i2c_read(false);
	si.i2c_stop();
	// if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
	// else return -1; //Return error if read failed
	return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

void SplitAndLoad(uint8_t Pos, int16_t Val) //Write 16 bits
{
	uint8_t Len = sizeof(Val);
	for(int i = Pos; i < Pos + Len; i++) {
		Reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
	}
}

void SplitAndLoad(uint8_t Pos, long Val)  //Write 32 bits
{
	uint8_t Len = sizeof(Val);
	for(int i = Pos; i < Pos + Len; i++) {
		Reg[i] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
	}
}

boolean addressEvent(uint16_t address, uint8_t count)
{
	RepeatedStart = (count > 0 ? true : false);
	return true; // send ACK to master
}

void requestEvent()
{	
	//Allow for repeated start condition 
	if(RepeatedStart) {
		for(int i = 0; i < 2; i++) {
			Wire.write(Reg[RegID + i]);
		}
	}
	else {
		Wire.write(Reg[RegID]);
	}
}

void receiveEvent(int DataLen) 
{
    //Write data to appropriate location
    if(DataLen == 2){
	    //Remove while loop?? 
	    while(Wire.available() < 2); //Only option for writing would be register address, and single 8 bit value
	    uint8_t Pos = Wire.read();
	    uint8_t Val = Wire.read();
	    //Check for validity of write??
	    Reg[Pos] = Val; //Set register value
	}

	if(DataLen == 1){
		RegID = Wire.read(); //Read in the register ID to be used for subsequent read
	}
}

void stopEvent() 
{
	StopFlag = true;
	//End comunication
}


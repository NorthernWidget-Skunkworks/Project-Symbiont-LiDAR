#include <Wire.h>
#include <Margay.h>

Margay Logger(Model_2v0);

void setup() {
  // put your setup code here, to run once:
  Logger.PowerAux(ON);
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("START");
  uint8_t Data1 = 0;
  uint8_t Data2 = 0;

  for(int i = 0; i < 4; i++) {
    Wire.beginTransmission(0x50);
    Wire.write(2*i + 2);
    Wire.endTransmission();
    Wire.requestFrom(0x50, 1);
    Data1 = Wire.read();

    Wire.beginTransmission(0x50);
    Wire.write(2*i + 3);
    Wire.endTransmission();
    Wire.requestFrom(0x50, 1);
    Data2 = Wire.read();

    Serial.println((Data2 << 8) | Data1);
  }
  delay(1000);
}

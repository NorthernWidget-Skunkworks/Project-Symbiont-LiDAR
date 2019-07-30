void setup() {
  // put your setup code here, to run once:
  pinMode(14, OUTPUT);
  pinMode(8, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(8)) digitalWrite(14, HIGH);
  else(digitalWrite(14, LOW));
}

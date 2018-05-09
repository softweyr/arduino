
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(230400);
  while (!Serial);
  Serial.println("started");
}

int readings[2][256];
void loop() 
{
  static uint8_t count = 0;
  
  readings[0][count] = analogRead(A4);
  readings[1][count] = analogRead(A5);

  Serial.print(readings[0][count]);
  Serial.print(" : ");
  Serial.println(readings[1][count]);
}

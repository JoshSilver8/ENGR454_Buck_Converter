void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // read the input on analog pin 0:
  float sensorValue1 = 4.7*analogRead(A0)/(1023);
  float sensorValue2 = 4.7*analogRead(A2)/(1023);
  float sensorValue3 = 4.7*analogRead(A6)/(1023);
  // print out the value you read:
  Serial.print(sensorValue1);
  Serial.print("\t");
  Serial.print(sensorValue2);
  Serial.print("\t");
  Serial.print(sensorValue3);
  Serial.println();
  delay(1);        // delay in between reads for stability
}

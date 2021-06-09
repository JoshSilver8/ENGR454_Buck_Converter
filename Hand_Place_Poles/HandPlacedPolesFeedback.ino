// https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency


//int m ; // initialize variable m
//int n ; // initialize variable n
void setup()
{
pinMode(9,OUTPUT) ; // set pwm pin 6 as output pin
TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz

 // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}
void loop()
{
analogWrite(9,127);

 // read the input on analog pin 0:
  double sensorValue1 = 4.7*analogRead(A0)/(1023);
  double sensorValue2 = 4.7*analogRead(A1)/(1023);
  
  // print out the value you read:
  Serial.print(sensorValue1);
  Serial.print("\t");
  Serial.print(sensorValue2);
  Serial.println();
  delay(1);        // delay in between reads for stability
}

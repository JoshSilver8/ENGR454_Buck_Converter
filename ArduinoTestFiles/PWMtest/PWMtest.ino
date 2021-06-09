// https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency


//int m ; // initialize variable m
//int n ; // initialize variable n
void setup()
{
pinMode(9,OUTPUT) ; // set pwm pin 6 as output pin
//pinMode(A1,INPUT) ; // set analog pin as input pin
TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
Serial.begin(9600) ; // begin serial communication
}
void loop()
{
//m= analogRead(A1) ;// read voltage value from pin A1 which feeds from resistor divider
//n= map(m,0,1023,0,255) ; // map this ip value betwenn 0 and 255
analogWrite(9,127) ; // write mapped value on pin 12
//Serial.print(" PWM Value ") ;

}

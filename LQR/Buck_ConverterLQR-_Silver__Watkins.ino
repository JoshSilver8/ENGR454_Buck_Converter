//////////////////////////////////////////////
// LQR Poles - Buck Converter Project
// Joshua Silver & Jordyn Watkins
// May 19, 2021
// ENGR 454
// Dr. Frohne
//////////////////////////////////////////////

// Define Variables and dependences
#include "Plotter.h"
int pwm = 9;
float Vout;
float Vin;
float il;
float Vscale = (2*Vin)/1023; // (2*Vin)/1023 Where Vin = 4.4V read off of the Arduino
float Rc = 1.09; //Current Sense Resistor
int serCount = 0; 

float Vin = 9;
float Vset = 5; //Input based off of USB Power
float Vos; // Offset Voltage from Desired Vout

int   DD = (255*Vset)/Vin;
float Dd = 0; //Duty Cycle
float dd = 0; //feedback

// Enter Feeback Here
// Poles [-5.8497E3,-5.1882E3]
// G = [g1 , g2] - found from Matlab 
float G[] = {1.5373,0.8384};

void setup() 
{
  // Setting pin 9 to PMW to operate at max frequency
  TCCR1B = TCCR1B & 0b11111000 | 0b00000001;
  pinMode(pwm, OUTPUT);
  Serial.begin(9600);
}

void loop() 
{
  // Read State Variables
  Vout = analogRead(A0)*Vscale; 
  il   = (analogRead(A1)*Vscale-Vout)/Rc;

  // Calculate Duty Cycle
  Vos = (Vout - Vset);
  dd = (-G[0]*il-G[1]*Vos)*255;
  Dd = DD + dd;

  // Ensure that Duty Cycle is in Range
  
  if (0 < Dd < 256)
    analogWrite(pwm, Dd);
  if (Dd < 0) //If PMW below min input
    analogWrite(pwm, 20);
  if (Dd > 255) //If PMW above max input
    analogWrite(pwm, 230);


  // Serial Print every ~2s
  if (serCount == 10000)
  {
    Serial.print("V_out: ");
    Serial.println(Vout);
    Serial.print("V Desired: "); //Voffset
    Serial.println(Vos);
    Serial.print("I_l: ");
    Serial.println(il);
    Serial.print("Duty Cycle: ");
    Serial.println(Dd);
    Serial.print("Large Signal: ");
    Serial.println(DD);
    Serial.print("Feedback: ");
    Serial.println(dd);
    Serial.print("\n\n\n\n\n\n\n\n"); //Poor Mans Clear Screen
    serCount = 0;
    }
  else
    serCount++;
}

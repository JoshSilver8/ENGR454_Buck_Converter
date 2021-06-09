//////////////////////////////////////////////
// Full Order Observer - Buck Converter Project
// Joshua Silver & Jordyn Watkins
// June 8, 2021
// ENGR 454
// Professor: Dr. Frohne
//////////////////////////////////////////////

// Define Variables and dependences
int pwm = 9;
float Vout;
float Vin=4.3;
float il;
float Vscale = (2*Vin)/1023; // (2*Vin)/1023 Where Vin = 4.4V read off of the Arduino
float Rc = 1.09; //Current Sense Resistor
int serCount = 0; 

float Vset = 5; //Input based off of USB Power
float Vos; // Offset Voltage from Desired Vout
float Voutprev;

int   DD = (255*Vset)/Vin;
float Dd = 0; //Duty Cycle
float dd = 0; //feedback

float timer;
int A11 = 0;
int A12 = -1000;
int A21 = 10000;
int A22 = -386;
int B_1 = 10000;
int B_2 = 0;
float ilhat = 0.5;
float vchat = 5;
float K1[] = {-0.0298,-0.11}; //the K in K*y[k]
float K2[] ={-999.98,-286.37}; //the K in (A-KC)*x_hat[k]

// Enter Feeback Here
// Poles [-5.8497E3,-5.1882E3]
// G = [g1 , g2] - found from Matlab 
float G[] = {-1.2581,-0.4872};

void setup() 
{
  // Setting pin 9 to PMW to operate at max frequency
  TCCR1B = TCCR1B & 0b11111000 | 0b00000001;
  pinMode(pwm, OUTPUT);
  Serial.begin(9600);
}

void loop() 
{
  timer = 0.00042;
  
  // Read State Variables
  Vout = analogRead(A0)*Vscale; 
  il   = (analogRead(A1)*Vscale-Vout)/Rc;

  ilhat = ((A11-B_1*G[0])*timer+1)*ilhat+(A12-K2[0]-B_1*G[1])*timer*vchat+K1[0]*timer*Vout;
  vchat = ((A21-B_2*G[0])*timer)*ilhat+((A22-K2[1]-B_2*G[1])*timer+1)*vchat+K1[1]*timer*Vout;

  // Calculate Duty Cycle
  Vos = (Vout - Vset);
  dd = (-G[0]*ilhat-G[1]*Vos)*255;
  Dd = DD + dd;

  // Ensure that Duty Cycle is in Range
  
  if (0 < Dd < 256)
    analogWrite(pwm, Dd);
  if (Dd < 0) //If PMW below min input
    analogWrite(pwm, 20);
  if (Dd > 255) //If PMW above max input
    analogWrite(pwm, 230);


//   Serial Print every ~2s
//  if (serCount == 10000)
//  {
//    Serial.print("V_out: ");
//    Serial.println(Vout);
//    Serial.print("V Desired: "); //Voffset
//    Serial.println(Vos);
//    Serial.print("I_l: ");
//    Serial.println(il);
//    Serial.print("I_l: ");
//    Serial.println(ilhat);
//    Serial.print("I_l error: ");
//    Serial.println(ilhat-il);
//    Serial.print("Duty Cycle: ");
//    Serial.println(Dd);
//    Serial.print("Large Signal: ");
//    Serial.println(DD);
//    Serial.print("Feedback: ");
//    Serial.println(dd);
//    Serial.print("\n\n\n\n\n\n\n\n"); //Poor Mans Clear Screen
//    serCount = 0;
//    }
//  else
//    serCount++;
}

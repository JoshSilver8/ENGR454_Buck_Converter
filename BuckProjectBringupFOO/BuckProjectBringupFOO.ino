#include "Plotter.h"

int pwm = 9;           // the PWM pin the LED is attached to
float Vout;
float il;
float duty = 0;
float feedback = 0;
float Vwanted;
int i = 0;
int feedbackcount = 0;
int negcount = 0;
int overcount = 0;
//int Vtp8 = 19;
float timer = 0.011;
float ilest = .5;
float vcest = .5;
Plotter p;

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  TCCR1B = TCCR1B & B11111000 | B00000001;
  pinMode(pwm, OUTPUT);
//  pinMode(Vtp8, INPUT);
  //p.Begin();
  //p.AddTimeGraph("Voltage", 500, "Vout", Voutplot);
  //p.AddTimeGraph("Inductor Current", 500, "iL", ilplot);
  //p.AddTimeGraph("Duty", 500, "duty", duty);
  //p.AddTimeGraph("Feedback", 500, "feedback", feedback);
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  //Serial.println(micros());
  
  Vout = analogRead(A0)*0.009142;
  il = (analogRead(A1)*0.009142-Vout)/1.18;

  ilest = (266*(timer)+1)*ilest+265.04*(timer)*Vout-999.04*Vout; 
  vcest = (10000*(timer)+1)*ilest-224.71*(timer)*Vout-186.1*Vout;
  
  Vwanted = (vcest-5);
  
  feedback = (-(0.0266*255)*ilest-(0.0989*255)*
d);
  
  
  
  duty = 123+(feedback);

  if(i == 10000){
    Serial.print("Vout:");
    Serial.println(vcest);
    Serial.print("Il:");
    Serial.println(ilest);
    Serial.print("feedback:");
    Serial.println(feedback);
    Serial.print("duty:");
    Serial.println(duty);
    Serial.print("FeedbackCount:");
    Serial.println(feedbackcount);
    Serial.print("Undercount:");
    Serial.println(negcount);     
    Serial.print("OverCount:");
    Serial.println(overcount);
       
    //Serial.println(duty);
    i = 0;
  } else {
    i++;
  }

  //p.Plot();
  //Serial.println(Vout);
  //Serial.println(feedback);
  
  if(duty >0 && duty < 256){
    analogWrite(pwm,(int) duty);
    feedbackcount++;
  }

  if(duty >256) {
    analogWrite(pwm, 255);
    overcount++;
  }

  if(duty < 0 ) {
    analogWrite(pwm, 25);
    negcount++;
  }
}

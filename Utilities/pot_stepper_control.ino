#include <Stepper.h>

const int dirPin = 7;  // arm 2 dir
const int stepPin = 8; // arm 2 step
const int dirPin2 = 9;  // wrist dir
const int stepPin2 = 10; // wrist step

const int in1 = 7; // Control pin 1 for end effector
const int in2 = 8; // Control pin 2 for end effector

int arm2_pot = A1;
int wrist_pot = A2;
int end_pot = A3;

void setup() {
  // put your setup code here, to run once:
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(stepPin2,OUTPUT); 
  pinMode(dirPin2,OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int wrist = analogRead(wrist_pot);
  int arm2 = analogRead(arm2_pot);

  int end_e = analogRead(end_pot);
  
  int delay_t = 2000;
  Serial.println(end_e);
  if (arm2 < 160){
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(delay_t);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(delay_t);
   }
   else if (arm2 > 380){
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(delay_t);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(delay_t);
   }
   else {
    }
   if (wrist < 400){
    digitalWrite(dirPin2, LOW);
    digitalWrite(stepPin2,HIGH);
    delayMicroseconds(delay_t);
    digitalWrite(stepPin2,LOW);
    delayMicroseconds(delay_t);
   }
   else if (wrist > 700){
    digitalWrite(dirPin2, HIGH);
    digitalWrite(stepPin2,HIGH);
    delayMicroseconds(delay_t);
    digitalWrite(stepPin2,LOW);
    delayMicroseconds(delay_t);
   }
   else {
    }
  if (end_e < 400) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    }
  else if (end_e > 700){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    }
   else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    }
}

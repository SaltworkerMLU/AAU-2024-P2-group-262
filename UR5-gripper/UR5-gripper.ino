#include <Servo.h>

int inA = 8;
int inB = 9;
int inC = 10;

int outA = 41; // Blå A // 41
int outB = 43; // Gul B // 43
int outC = 45; // Grøn C // 45

int openGripper = 20;
int closeGripper = 70;
int timeToGrip = 0; // in milliseconds

Servo Servo_A;
Servo Servo_B;
Servo Servo_C;

bool Change_A = 0;
bool Change_B = 0;
bool Change_C = 0;

void setup() {
  // put your setup code here, to run once:
 
  /*pinMode(inA, INPUT);
  pinMode(inB, INPUT);
  pinMode(inC, INPUT);
  pinMode(outA, OUTPUT);
  pinMode(outB, OUTPUT);
  pinMode(outC, OUTPUT);*/
Servo_A.attach(outA); 
Servo_B.attach(outB); 
Servo_C.attach(outC); 
  Serial.begin(9600);
  Servo_A.write(closeGripper);
  Servo_B.write(closeGripper);
  Servo_C.write(openGripper);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(inA) != Change_A){
    RunServoA();
    Change_A = !Change_A;
  }
  if(digitalRead(inB) != Change_B){
    RunServoB();
    Change_B = !Change_B;
  }
  if(digitalRead(inC) != Change_C){
    RunServoC();
    Change_C = !Change_C;
  }
  Serial.print(digitalRead(inA));
  Serial.print("\t");
  Serial.print(digitalRead(inB));
  Serial.print("\t");
  Serial.println(digitalRead(inC));
  delay(100);
}

void RunServoA(){
  if (digitalRead(inA) == LOW) {
    Servo_A.write(closeGripper);
    delay(timeToGrip);
  }

  else{
    Servo_A.write(openGripper);
    delay(timeToGrip);
  }
}

void RunServoB(){
  if (digitalRead(inB) == LOW) {
    Servo_B.write(closeGripper-5);
    delay(timeToGrip);
  }

  else{
    Servo_B.write(openGripper+5);
    delay(timeToGrip);
  }
}

void RunServoC(){
  if (digitalRead(inC) == HIGH) {
    Servo_C.write(closeGripper);
    delay(timeToGrip);
  }

  else{
    Servo_C.write(openGripper);
    delay(timeToGrip);
  }
}

void RunServos(){
  if (digitalRead(inA) == HIGH) {
  Servo_A.write(closeGripper);
  delay(timeToGrip);
}

else{
  Servo_A.write(openGripper);
  delay(timeToGrip);
}

// Servo B
if (digitalRead(inB) == HIGH) {
  Servo_B.write(closeGripper);
  delay(timeToGrip);
}

else{
  Servo_B.write(openGripper);
  delay(timeToGrip);
}

// Servo C
if (digitalRead(inC) == HIGH) {
  Servo_C.write(closeGripper);
  delay(timeToGrip);
}

else{
  Servo_C.write(openGripper);
  delay(timeToGrip);
}


}


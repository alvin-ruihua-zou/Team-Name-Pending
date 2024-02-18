
#include <Encoder.h>
//#include "Servo.h"
#include "MultiServo.h"
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

// pindef_____________________________________________________________________

const int m0en0 = 3;
const int m0en1 = 4;
const int m0pwm = 2;
const int m1en0 = 6;
const int m1en1 = 7;
const int m1pwm = 5;


// end pindef__________________________________________________________________
long currPosition  = 0;
long currPosition2 = 0;
//Encoder myEnc(2, 8);
Servo s1(m0en0, m0en1, m0pwm, 19, 23);
Servo s2(m1en0, m1en1, m1pwm, 18, 22);
MultiServo zAxis(s1, s2);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  pinMode(m0en0, OUTPUT);
  pinMode(m0en1, OUTPUT);
  pinMode(m0pwm, OUTPUT);
  pinMode(m1en0, OUTPUT);
  pinMode(m1en1, OUTPUT);
  pinMode(m1pwm, OUTPUT);

  //s1.set_current_limiting(400, true);
}

void fw(int pwm){
  digitalWrite(m0en0, HIGH);
  digitalWrite(m0en1, LOW);
  analogWrite(m0pwm, pwm);
}

void bw(int pwm){
  digitalWrite(m0en0, LOW);
  digitalWrite(m0en1, HIGH);
  analogWrite(m0pwm, pwm);
}

void idle(){
  digitalWrite(m0en0, LOW);
  digitalWrite(m0en1, LOW);
}

void mbreak(){
  digitalWrite(m0en0, HIGH);
  digitalWrite(m0en1, HIGH);
}

void cmd_servo_multi(){
  Serial.println("Enter revolution:");
  while (Serial.available() == 0){
    delay(10);
  }
  String cmd = Serial.readString();
  Serial.println(cmd);
  char dir = cmd[0];
  double rev =cmd.substring(1).toFloat();
  
  long currPosition1 = s1.myEnc.read();
  long currPosition2 = s2.myEnc.read();
  long tick1 = rev * 792 + currPosition1;
  long tick2 = rev * 792 + currPosition2;
  // if(dir == 'w'){
  //   s1.servo_to(tick1, 160, 0.2, 0.1);
  // }
  if(dir == 'w'){
    // s1.fw(60);
    // delay(1000);
    // s1.mbreak();
    // delay(1000);
    // zAxis.s1.fw(60);
    // delay(1000);
    zAxis.servo_to(tick1, tick2, 120, 0.2, 0.1);
  }
}

// void cmd_servo(){
//   Serial.println("Enter M1 revolution:");
//   while (Serial.available() == 0){
//     delay(10);
//   }
//   String cmd = Serial.readString();
//   Serial.println(cmd);
//   char dir = cmd[0];
//   double rev =cmd.substring(1).toFloat();
  
//   currPosition = s1.myEnc.read();
//   int tick = rev * 792 + currPosition;
//   if(dir == 'w'){
//     s1.servo_to(tick, 160, 0.2, 0.1);
//   }

//   currPosition2 = s2.myEnc.read();
//   int tick2 = rev * 792 + currPosition2;
//   if(dir == 'w'){
//     s2.servo_to(tick2, 160, 0.2, 0.1);
//   }
// }
void loop() {
  cmd_servo_multi();



}

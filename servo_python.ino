#include <Servo.h>
Servo myServo_lr;
Servo myServo_ud;
int servo_left_right_Pin = 9;
int servo_up_down_Pin = 10;
String servoPos_lr;
String servoPos_ud;

int poslr, posud;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
myServo_lr.attach(servo_left_right_Pin);
myServo_ud.attach(servo_up_down_Pin);
myServo_lr.write(90);
myServo_ud.write(40);
}

void loop() {
  // put your main code here, to run repeatedly:
while (Serial.available() == 0){
}
  String servoPos = Serial.readStringUntil('\r');
  if(servoPos[0]=='a'){
    servoPos.remove(0,1);
    poslr = servoPos.toInt();
    myServo_lr.write(poslr);
  }
  
  else if(servoPos[0]=='c'){
    servoPos.remove(0,1);
    posud = servoPos.toInt();
    myServo_ud.write(posud);
  }
  else{}
 
  Serial.println(poslr + ' ' + posud);
  delay(50);
/*  if (stop_time >30){
    myServo_lr.write(90);
    myServo_lr.write(40);
    stop_time=0;
    delay(500); 
  }*/
  
}

#ifndef motor 
#define motor    


#define MotorDriverLeft(pwm , dir) {    switch (dir)  { case 1: analogWrite(motorL1, pwm);digitalWrite( motorR1, LOW );  break;  case -1: analogWrite(motorR1, pwm); digitalWrite( motorL1, LOW ); break;}}
#define MotorDriverRight(pwm , dir){   switch (dir)  { case 1: analogWrite(motorL2, pwm); digitalWrite( motorR2, LOW ); break;  case -1: analogWrite(motorR2, pwm); digitalWrite( motorL2, LOW ); break;}}


#define BrakeMotorLeft() digitalWrite( motorL1, HIGH );    digitalWrite( motorR1, HIGH )
#define BrakeMotorRight() digitalWrite( motorL2, HIGH );    digitalWrite( motorR2, HIGH )

#define StopMotorLeft() digitalWrite( motorL1, LOW );    digitalWrite( motorR1, LOW )
#define StopMotorRight() digitalWrite( motorL2, LOW );    digitalWrite( motorR2, LOW )

#define motorL1 5 //11
#define motorR1 10 //5
#define motorL2 6
#define motorR2 9 //9


void init_motor_driver()
{
	pinMode(motorL1,OUTPUT);
	pinMode(motorR1,OUTPUT);
	pinMode(motorL2,OUTPUT);
	pinMode(motorR2,OUTPUT);
  BrakeMotorLeft();
  BrakeMotorRight();

}

#endif

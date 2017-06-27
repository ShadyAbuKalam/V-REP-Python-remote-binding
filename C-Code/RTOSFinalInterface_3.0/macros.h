#ifndef macro
#define macro

//conststants
#define SWITCHTASKSTACKSIZE        128  //Stack Size
#define MEGA          0     // Arduino Mega requires different Pinout
#define VERBOSE       0     // Enables verbose output via Serial Port 
#define RFRECVPERIOD  10    // RF Receiving period in system ticks
#define RFSENDPERIOD  15    // RF Sending period in system ticks
#define TASKSTACKSZIE 128   // Stack size for the tasks

//#define MotorDriverLeft( pwm , dir){    digitalWrite( motorL1, LOW );    digitalWrite( motorR1, LOW );   switch (dir)  { case 1: analogWrite(motorL1, pwm);  break; case 0: analogWrite(motorR1, pwm);   break;  }}
//#define MotorDriverRight( pwm , dir){    digitalWrite( motorL2, LOW );    digitalWrite( motorR2, LOW );   switch (dir)  { case 1: analogWrite(motorL2, pwm);  break; case 0: analogWrite(motorR2, pwm);   break;  }}

#/*define MotorDriverLeft( pwm , dir){    switch (dir)  { case 1: analogWrite(motorL1, 255 - pwm); digitalWrite(motorR1,HIGH);  break; case 0: analogWrite(motorL1, pwm); digitalWrite(motorR1,LOW);   break;  }}
#define MotorDriverRight( pwm , dir){    switch (dir)  { case 1: analogWrite(motorL2, 255 - pwm); digitalWrite(motorR2,HIGH);  break; case 0: analogWrite(motorL2, pwm); digitalWrite(motorR2,LOW);   break;  }}

#define BreakMotorLeft() digitalWrite( motorL1, HIGH );    digitalWrite( motorR1, HIGH )
#define BreakMotorRight() digitalWrite( motorL2, HIGH );    digitalWrite( motorR2, HIGH )

#define StopMotorLeft() digitalWrite( motorL1, LOW );    digitalWrite( motorR1, LOW )
#define StopMotorRight() digitalWrite( motorL2, LOW );    digitalWrite( motorR2, LOW )
*/

//1 tick = 17.5 ms


#define ROTATION_90_TIME 2 * TICKS_PER_SECOND



//pins used
#define ultraSonicEcho  A3 //Ultrasound signal pin
#define ultraSoundSignal  A0 // Ultrasound signal pin
#define s0  4
//#define s1  3
#define s2  10
#define s3  12
#define out  4
#define motorL1 11
#define motorR1 5
#define motorL2 6
#define motorR2 9

#define LineDetectorPin_1 2
#define LineDetectorPin_2 3 
#define RPMCounter_1 LineDetectorCounter_1
#define RPMCounter_2 LineDetectorCounter_2


//macros and enums
#define BLUE 0
#define RED 1
#define Green 2
#define WHITE 3

#endif

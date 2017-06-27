#include <Arduino_FreeRTOS.h>
#include <RF24.h>
#include <SPI.h>
#include "task.h"
#include "macros.h"
#include "ultrasonic.h"
#include "colorsensor.h"
#include "motordriver.h"
#include "linetracker.h"
#include "rfrtos.h"

// All distances are in meter
#define AT_GOAL 0.03
#define AT_OBSTACLE 0.25
#define UNSAFE 0.15

#define WHEEL_R 0.0325 // Need to be measured: the wheel radius
#define WHEEL_L 0.09925 // Need to be measured: the distance between the centers of the two wheels

#define RAD_TO_DEG (180 / (2 * PI))
#define DEG_TO_RAD ((2 * PI) / 180)
#define RAD_PER_SEC_TO_RPM (60 / (2 * PI))
#define RPM_TO_RAD_PER_SEC ((2 * PI) / 60)

#define TICKS_PER_REV 2
#define METERS_PER_TICK (2*PI*WHEEL_R / TICKS_PER_REV)

#define MAX_PWM 250 // Need to be measured: this value should be measured on the surface we will work on
#define MIN_PWM 200 // Need to be measured: this value should be measured on the surface we will work on
#define MAX_RPM 120 // Need to be measured: this value should be measured on the surface we will work on
#define MIN_RPM 80 // Need to be measured: this value should be measured on the surface we will work on
#define MAX_ANG_VEL (MAX_RPM * RPM_TO_RAD_PER_SEC)
#define MIN_ANG_VEL (MIN_RPM * RPM_TO_RAD_PER_SEC)

#define MAX_ANG_VEL_AT_ZERO_LIN_VEL 5.2 // Need to be measured: unicycle model maximum angular velocity at zero linear velocity (1 is just a dummy)
#define MAX_lIN_VEL_AT_ZERO_ANG_VEL  0.3 // Need to be measured: unicycle model maximum linear velocity at zero angular velocity

#define GO_TO_ANGLE_K_P 1 // P-regulator

#define GO_TO_GOAL_K_P 4
#define GO_TO_GOAL_K_I 0.01
#define GO_TO_GOAL_K_D 0.01

#define AVOID_OBSTACLES_K_P 4
#define AVOID_OBSTACLES_K_I 0.01
#define AVOID_OBSTACLES_K_D 0.01

#define AO_AND_GTG_K_P 4
#define AO_AND_GTG_K_I 0.01
#define AO_AND_GTG_K_D 0.01
#define BLENDING_ALPHA 0.25

#define US_LEFT_X 0.08 // Need to be measured from real robot
#define US_LEFT_Y 0.04 // Need to be measured from real robot
#define US_LEFT_THETA 0 // Need to be measured from real robot

#define US_MID_X 0.08 // Need to be measured from real robot
#define US_MID_Y 0 // Need to be measured from real robot
#define US_MID_THETA 0 // Need to be measured from real robot

#define US_RIGHT_X 0.08 // Need to be measured from real robot
#define US_RIGHT_Y -0.04 // Need to be measured from real robot
#define US_RIGHT_THETA 0 // Need to be measured from real robot

typedef struct Vector {
  float x;
  float y;
} Vector;

typedef struct Pose {
  float x;
  float y;
  float theta;
  signed int leftDirection; // 1 for forward motion and -1 for backward motion
  signed int rightDirection; // 1 for forward motion and -1 for backward motion
  float v; // linear velocity m/s
  float w; // angular velocity rad/s
} Pose;

Pose pose = {.x = 0, .y = 0, .theta = PI / 2, .leftDirection = 1, .rightDirection = 1, .v = 0.1, .w = 0};

// Global variables for applyDynamics
float vLeft, vRight, velMax, velMin;
int leftRPM, rightRPM, leftPWM, rightPWM;

// Global variables for updateOdometry
unsigned long totalTicksLeft = 0, totalTicksRight = 0, prevTicksLeft = 0, prevTicksRight = 0;
float deltaLeft, deltaRight, deltaCenter, phi, radiusCenter;

// Global variables for goToAngle
float goToAngleError;

// Global variables for goToGoal
float goToGoalError, goToGoalTotalError, goToGoalPrevError, pGoToGoalError, iGoToGoalError, dGoToGoalError;
float uGoToGoalX, uGoToGoalY; // vector u from robot to goal
float thetaGoToGoal;

// Global variables for updateVectorsUS
float leftDistance, midDistance, rightDistance;
Vector robotFrameLeftVectorUS, robotFrameMidVectorUS, robotFrameRightVectorUS;
Vector leftVectorUS, midVectorUS, rightVectorUS;

// Global variables for avoidObstacles
float avoidObstaclesError, avoidObstaclesTotalError, avoidObstaclesPrevError, pAvoidObstaclesError, iAvoidObstaclesError, dAvoidObstaclesError;
float uAvoidObstaclesX, uAvoidObstaclesY; // summation of obstace vectors attempting to steer robot in the direction having maximum space
float thetaAvoidObstacles;

// Global variables for blendedAOandGTG
float blendedAOandGTGError, blendedAOandGTGTotalError, blendedAOandGTGPrevError, pBlendedAOandGTGError, iBlendedAOandGTGError, dBlendedAOandGTGError;
float uBlendedAOandGTGX, uBlendedAOandGTGY; // blending of goal vector and obstace vectors attempting to steer robot in the direction having maximum space
float thetaBlendedAOandGTG;

// Global variable for goal point
float xGoal, yGoal;

// RF variables
char posex_temp[12];
char posey_temp[12];

// String posex_temp;
// String posey_temp;
char posetheta_temp[8];
extern unsigned char RF_EVENT;

// Sensors
unsigned long ultrasonic(char i);
unsigned long colorSensor(char color);

// Control functions
float distance(float x2, float y2, float x1, float y1) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void applyDynamics() {
  // Limit v,w from controller to +/- of their max
  pose.w = max(min(pose.w, MAX_ANG_VEL_AT_ZERO_LIN_VEL), -MAX_ANG_VEL_AT_ZERO_LIN_VEL);
  pose.v = max(min(pose.v, MAX_lIN_VEL_AT_ZERO_ANG_VEL), -MAX_lIN_VEL_AT_ZERO_ANG_VEL);
  // Unicycle model to differential model
  vLeft = (2 * pose.v - pose.w * WHEEL_L) / (2 * WHEEL_R); // Angular velocity rad/s of left wheel
  vRight = (2 * pose.v + pose.w * WHEEL_L) / (2 * WHEEL_R); // Angular velocity rad/s of left wheel
  // Find the max and min of vLeft/vRight
  velMax = max(vLeft, vRight);
  velMin = min(vLeft, vRight);
  // Shift vLeft and vRight if they exceed allowed max/min vel
  if (velMax > MAX_ANG_VEL) {
    vLeft -= (velMax - MAX_ANG_VEL);
    vRight -= (velMax - MAX_ANG_VEL);
  } else if (velMin < -MAX_ANG_VEL) {
    vLeft -= (velMin + MAX_ANG_VEL);
    vRight -= (velMin + MAX_ANG_VEL);
  }
  // Limit vLeft,vRight according to physical limitation
  vLeft = max(min(vLeft, MAX_ANG_VEL), -MAX_ANG_VEL);
  vRight = max(min(vRight, MAX_ANG_VEL), -MAX_ANG_VEL);
  // update pose directions and ensure there is enough power to move motors
  if (vLeft < 0 && vLeft > -MIN_ANG_VEL) {
    vLeft = -MIN_ANG_VEL;
  }
  else if (vLeft < MIN_ANG_VEL) {
    vLeft = 0;
  }
  if (vRight < 0 && vRight > -MIN_ANG_VEL) {
    vRight = -MIN_ANG_VEL;
  }
  else if (vRight < MIN_ANG_VEL) {
    vRight = 0;
  }
  leftRPM = abs(vLeft) * RAD_PER_SEC_TO_RPM;
  rightRPM = abs(vRight) * RAD_PER_SEC_TO_RPM;

  pose.leftDirection = (vLeft >= 0) ? 1:-1;
  pose.rightDirection = (vRight >= 0) ? 1:-1;
  if (vLeft != 0)
  leftPWM = map(leftRPM, MIN_RPM, MAX_RPM, MIN_PWM, MAX_PWM);
  else
  leftPWM = 0;

  if(vRight != 0)
  rightPWM = map(rightRPM, MIN_RPM, MAX_RPM, MIN_PWM, MAX_PWM);
  else
  rightPWM = 0;
  /*Serial.println("++++++++++++++++");
  Serial.println(pose.w);
  Serial.println(pose.v);
  Serial.println(vLeft);
  Serial.println(vRight);
  Serial.println(leftRPM);
  Serial.println(rightRPM);
  Serial.println(leftPWM);
  Serial.println(rightPWM);
  Serial.println("++++++++++++++++");*/
  MotorDriverLeft(leftPWM , pose.leftDirection);
  MotorDriverRight(rightPWM, pose.rightDirection);
}

void updateOdometry() {
  // This function should be called every 10 ms (or less if possible since the robots are fast)
  // this can be done either through timer interrupt or being a task in RTOS
  totalTicksLeft = RPMCounter_1;
  totalTicksRight = RPMCounter_2;
  if((totalTicksLeft - prevTicksLeft) == 0 && (totalTicksRight - prevTicksRight) == 0)
    return;
  deltaLeft = pose.leftDirection * (totalTicksLeft - prevTicksLeft) * METERS_PER_TICK;
  deltaRight = pose.rightDirection * (totalTicksRight - prevTicksRight) * METERS_PER_TICK;
  deltaCenter = (deltaRight + deltaLeft) / 2.0;
  phi = (deltaRight - deltaLeft) / WHEEL_L;
  if (1) { // phi == 0
    pose.x += deltaCenter * cos(pose.theta); // approx phi is small
    pose.y += deltaCenter * sin(pose.theta); // approx phi is small
  }
  else {
    //radiusCenter = deltaCenter / phi;
    //pose.x += radiusCenter * (-sin(pose.theta) + sin(phi) * cos(pose.theta) + sin(pose.theta) * cos(phi)); // no approx
    //pose.y += radiusCenter * (cos(pose.theta) - cos(phi) * cos(pose.theta) + sin(pose.theta) * sin(phi)); // no approx
  }
  pose.theta = atan2(sin(pose.theta + phi), cos(pose.theta + phi));
  prevTicksLeft = totalTicksLeft;
  prevTicksRight = totalTicksRight;
  updateVectorsUS();
}

void goToAngle(float thetaGoToAngle) {
  // This function should be called in a loop or interrupt or RTOS task until abs(pose.theta - thetaGoToGoal) is very small
  // Calling this function will alter angular velocity of robot while maintaining linear velocity constant
  goToAngleError = thetaGoToAngle - pose.theta;
  goToAngleError = atan2(sin(goToAngleError), cos(goToAngleError));
  pose.v = 0;
  pose.w = GO_TO_ANGLE_K_P * goToAngleError;
  applyDynamics();
}

void goToGoal() {
  // This function should be called in a loop or interrupt or RTOS task until abs(ditance_to_goal) is very small
  // Calling this function will alter angular velocity of robot while maintaining linear velocity constant
  uGoToGoalX = xGoal - pose.x;
  uGoToGoalY = yGoal - pose.y;
  thetaGoToGoal = atan2(uGoToGoalY, uGoToGoalX);
  goToGoalError = thetaGoToGoal - pose.theta;
  goToGoalError = atan2(sin(goToGoalError), cos(goToGoalError));
  pGoToGoalError = goToGoalError;
  iGoToGoalError = goToGoalTotalError + goToGoalError;
  dGoToGoalError = goToGoalError - goToGoalPrevError;
  goToGoalTotalError = iGoToGoalError;
  goToGoalPrevError = goToGoalError;
  pose.v = 0.2;
  pose.w = GO_TO_GOAL_K_P * pGoToGoalError + GO_TO_GOAL_K_I * iGoToGoalError + GO_TO_GOAL_K_D * dGoToGoalError;
  applyDynamics();
}

void updateVectorsUS() {
  // This function is called inside updateOdometry only
  leftDistance = 1; // replaced by a function or MACRO to get ultrasonic reading
  robotFrameLeftVectorUS.x = cos(US_LEFT_THETA) * leftDistance + US_LEFT_X;
  robotFrameLeftVectorUS.y = sin(US_LEFT_THETA) * leftDistance + US_LEFT_Y;
  leftVectorUS.x = cos(pose.theta) * robotFrameLeftVectorUS.x - sin(pose.theta) * robotFrameLeftVectorUS.y + pose.x;
  leftVectorUS.y = sin(pose.theta) * robotFrameLeftVectorUS.x + cos(pose.theta) * robotFrameLeftVectorUS.y + pose.y;

  midDistance = 1; // replaced by a function or MACRO to get ultrasonic reading
  robotFrameMidVectorUS.x = cos(US_MID_THETA) * midDistance + US_MID_X;
  robotFrameMidVectorUS.y = sin(US_MID_THETA) * midDistance + US_MID_Y;
  midVectorUS.x = cos(pose.theta) * robotFrameMidVectorUS.x - sin(pose.theta) * robotFrameMidVectorUS.y + pose.x;
  midVectorUS.y = sin(pose.theta) * robotFrameMidVectorUS.x + cos(pose.theta) * robotFrameMidVectorUS.y + pose.y;

  rightDistance = 1; // replaced by a function or MACRO to get ultrasonic reading
  robotFrameRightVectorUS.x = cos(US_RIGHT_THETA) * rightDistance + US_RIGHT_X;
  robotFrameRightVectorUS.y = sin(US_RIGHT_THETA) * rightDistance + US_RIGHT_Y;
  rightVectorUS.x = cos(pose.theta) * robotFrameRightVectorUS.x - sin(pose.theta) * robotFrameRightVectorUS.y + pose.x;
  rightVectorUS.y = sin(pose.theta) * robotFrameRightVectorUS.x + cos(pose.theta) * robotFrameRightVectorUS.y + pose.y;
}

void avoidObstacles() {
  // This should be used when unsafe until obstacle is cleared
  uAvoidObstaclesX = leftVectorUS.x + 0.5 * midVectorUS.x + rightVectorUS.x;
  uAvoidObstaclesY = leftVectorUS.y + 0.5 * midVectorUS.y + rightVectorUS.y;
  thetaAvoidObstacles = atan2(uAvoidObstaclesY, uAvoidObstaclesX);
  avoidObstaclesError = thetaAvoidObstacles - pose.theta;
  avoidObstaclesError = atan2(sin(avoidObstaclesError), cos(avoidObstaclesError));
  pAvoidObstaclesError = avoidObstaclesError;
  iAvoidObstaclesError = avoidObstaclesTotalError + avoidObstaclesError;
  dAvoidObstaclesError = avoidObstaclesError - avoidObstaclesPrevError;
  avoidObstaclesTotalError = iAvoidObstaclesError;
  avoidObstaclesPrevError = avoidObstaclesError;
  pose.v = 0.2;
  pose.w = AVOID_OBSTACLES_K_P * pAvoidObstaclesError + AVOID_OBSTACLES_K_I * iAvoidObstaclesError + AVOID_OBSTACLES_K_D * dAvoidObstaclesError;
  applyDynamics();
}

void blendedAOandGTG() {
  // This function should be called in a loop or interrupt or RTOS task until abs(ditance_to_goal) is very small
  // Calling this function will alter angular velocity of robot while maintaining linear velocity constant
  uBlendedAOandGTGX = BLENDING_ALPHA * (xGoal - pose.x) + (1 - BLENDING_ALPHA) * (leftVectorUS.x + 0.5 * midVectorUS.x + rightVectorUS.x);
  uBlendedAOandGTGY = BLENDING_ALPHA * (yGoal - pose.y) + (1 - BLENDING_ALPHA) * (leftVectorUS.y + 0.5 * midVectorUS.y + rightVectorUS.y);
  thetaBlendedAOandGTG = atan2(uBlendedAOandGTGY, uBlendedAOandGTGX);
  blendedAOandGTGError = thetaBlendedAOandGTG - pose.theta;
  blendedAOandGTGError = atan2(sin(blendedAOandGTGError), cos(blendedAOandGTGError));
  pBlendedAOandGTGError = blendedAOandGTGError;
  iBlendedAOandGTGError = blendedAOandGTGTotalError + blendedAOandGTGError;
  dBlendedAOandGTGError = blendedAOandGTGError - blendedAOandGTGPrevError;
  blendedAOandGTGTotalError = iBlendedAOandGTGError;
  blendedAOandGTGPrevError = blendedAOandGTGError;
  pose.v = 0.2;
  pose.w = AO_AND_GTG_K_P * pBlendedAOandGTGError + AO_AND_GTG_K_I * iBlendedAOandGTGError + AO_AND_GTG_K_D * dBlendedAOandGTGError;
  applyDynamics();
}

void brakeRobot() {
  // use brake macros
  BrakeMotorLeft();
  BrakeMotorRight();
}

void MainTask(void * param) {
  int stateEvent = 3;
  float desiredTheta = - PI / 2;
  for (;;) {


    pose.v = 0.4;
    pose.w = 0;
    applyDynamics();
    Serial.print("x = ");
    Serial.print(pose.x);
    Serial.print(", y =");
    Serial.println(pose.y);
    Serial.print(" , theta = ");
    Serial.println(pose.theta);
    Serial.println("==================");
         updateOdometry();
// TEST CODE END
    vTaskDelay(10);
  }
}
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  init_color_sensor();
  init_motor_driver();
  init_line_tracker();
  init_ultrasonic();
//  init_rf();

  //Serial.begin(9600); //initialises the serial for debugging
  TaskHandle_t  First_handle; //task handles and it is redundant and can be replaced with NULL as long as it is neede
  xTaskCreate(MainTask, "Task2", 256, NULL, 5, &First_handle); //creating main task
  //starting the scheduler
  vTaskStartScheduler();


}

void loop() {
  // put your main code here, to run repeatedly:

}

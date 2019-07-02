
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>

#include <stdio.h>
#include <math.h>

/*
 * macros
 */
#define TIME_STEP 64
#define PI 3.141592
#define OBSTACLE_DIST 50.0


enum {
  GO,
  TURN,
  FREEWAY,
  OBSTACLE,
  AUTONOMUS,
  MANUAL,
  LEFT,
  RIGHT
};


int mode;
double straightLineAngle;

int searchForObstacles(WbDeviceTag distance_sensor) {
  double distance_of_sensor = wb_distance_sensor_get_value(distance_sensor);
  printf("Distance%lf\n", distance_of_sensor );
  if (distance_of_sensor > OBSTACLE_DIST)
    return FREEWAY;
  else
    return OBSTACLE;
}


void fowardLinearly(WbDeviceTag *wheels, double velocity) {
  wb_motor_set_velocity(wheels[0], -6);
  wb_motor_set_velocity(wheels[1], 6);
  wb_motor_set_velocity(wheels[2], 0);
}

void backwardLinearly(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], -6);
  wb_motor_set_velocity(wheels[2], 0);
}

void rightLinearly(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2],-6);
}

void leftlinearly(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0],-6);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 6);
}

void wheelsTurnRight(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], -6);
  wb_motor_set_velocity(wheels[1], -6);
  wb_motor_set_velocity(wheels[2], -6);
}

void wheelsTurnLeft(WbDeviceTag *wheels){
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], 6);
  wb_motor_set_velocity(wheels[2], 6);
}

void stopWheels(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
}

double getAngleRobot(WbDeviceTag pos_sensor) {
  printf("Angle calculation\n");
  double angle, rotationAngleW1;

  rotationAngleW1 = wb_position_sensor_get_value(pos_sensor);
  angle = fabs(rotationAngleW1- straightLineAngle);
  printf("Angle: %lf\n", angle);

  return angle;
}
float clearAngleRobot() {
  printf("Clearing angle\n");
}
/*
 * main
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);

   //Motor devices
   WbDeviceTag wheel1= wb_robot_get_device("wheel1");
   WbDeviceTag wheel2= wb_robot_get_device("wheel2");
   WbDeviceTag wheel3= wb_robot_get_device("wheel3");

   WbDeviceTag wheels[3];
   wheels[0] = wheel1;
   wheels[1] = wheel2;
   wheels[2] = wheel3;

   wb_motor_set_position(wheel1, INFINITY);
   wb_motor_set_position(wheel2, INFINITY);
   wb_motor_set_position(wheel3, INFINITY);

   float velocity;
   int robot_state = GO;

   //distance sensor devices
   WbDeviceTag dist_right=wb_robot_get_device("distance_right");
   wb_distance_sensor_enable(dist_right, TIME_STEP);

   WbDeviceTag dist_left=wb_robot_get_device("distance_left");
   wb_distance_sensor_enable(dist_left, TIME_STEP);

   double distance_value;

  //encoder device
   WbDeviceTag encoder = wb_robot_get_device("encoder1");
   wb_position_sensor_enable(encoder, TIME_STEP);
   float angle;

   //keyboard variables
   int keyboard;


  /* main loop*/
  while (wb_robot_step(TIME_STEP) != -1) {

    keyboard = wb_keyboard_get_key();

    if (keyboard == 'A'){
      mode = RIGHT;
      straightLineAngle= wb_position_sensor_get_value(encoder);
    }
    else if (keyboard == 'S'){
      mode = LEFT;
      straightLineAngle = wb_position_sensor_get_value(encoder);
    }
    else if (keyboard == 'G'){
      mode = AUTONOMUS;
    }
    else if (keyboard == 'W')
      mode = MANUAL;

if (mode == AUTONOMUS){

    if (robot_state == GO) {
      distance_value = searchForObstacles(dist_left);
      if (distance_value== FREEWAY) {
        velocity = 8;
        fowardLinearly(wheels, velocity);
        angle = wb_position_sensor_get_value(encoder);
        printf("Angle: %lf\n", angle);
      } else if (distance_value== OBSTACLE) {
        robot_state = TURN;
        stopWheels(wheels);
        straightLineAngle = wb_position_sensor_get_value(encoder);
      }
    } else if (robot_state == TURN) {
      wheelsTurnRight(wheels);
      angle = getAngleRobot(encoder);
        if (angle >= PI) {
          robot_state = GO;
          stopWheels(wheels);
          clearAngleRobot();
      }
    }
  }
 else {
     if (keyboard == WB_KEYBOARD_UP){
       fowardLinearly(wheels, velocity);
       angle = wb_position_sensor_get_value(encoder);
       printf("Angle: %lf\n", angle);

     } else if (keyboard == WB_KEYBOARD_DOWN){
         backwardLinearly(wheels);
         angle = wb_position_sensor_get_value(encoder);
         printf("Angle: %lf\n", angle);

     } else if (keyboard == WB_KEYBOARD_RIGHT){
         rightLinearly(wheels);
         angle = wb_position_sensor_get_value(encoder);
         printf("Angle: %lf\n", angle);

     } else if (keyboard == WB_KEYBOARD_LEFT){
         leftlinearly(wheels);
         angle = wb_position_sensor_get_value(encoder);
         printf("Angle: %lf\n", angle);

     } else if (mode == LEFT){
         wheelsTurnLeft(wheels);
         angle =  wb_position_sensor_get_value(encoder);
         if (angle >= 0.4*PI) {
           robot_state = GO;
           stopWheels(wheels);
         }
     } else if (mode == RIGHT){
         wheelsTurnRight(wheels);
         angle = getAngleRobot(encoder);
         if (angle >= 0.4*PI) {
           robot_state = GO;
           stopWheels(wheels);
         }
     } else {
         stopWheels(wheels);
     }

   }
}

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

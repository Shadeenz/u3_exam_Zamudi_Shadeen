/*
 * File:          omni1_Zamudio_Shadeen.c
 * Date: 3/07/19
 * Description:
 * Author: Zamudio Shadeen
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
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
#define OBSTACLE_DIST 120.0

enum {
  GO,
  TURNRIGHT,
  TURNLEFT,
  FREEWAY,
  OBSTACLE,
  AUTONOMUS,
  MANUAL,
  LEFT,
  RIGHT
};

/*variables*/
int mode = AUTONOMUS;
double straightLineAngle;

/*Auxiliar functions*/
int searchForObstacles(WbDeviceTag distance_sensor) {
  double distance_of_sensor = wb_distance_sensor_get_value(distance_sensor);
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
  double angle, rotationAngleW1;

  rotationAngleW1 = wb_position_sensor_get_value(pos_sensor);
  angle = fabs(rotationAngleW1- straightLineAngle);

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

   double distance_left;
   double distance_right;

  //encoder device
   WbDeviceTag encoder = wb_robot_get_device("encoder1");
   wb_position_sensor_enable(encoder, TIME_STEP);
   float angle;

   //keyboard variables
   int keyboard;


  /* main loop*/
  while(wb_robot_step(TIME_STEP) != -1) {
    keyboard = wb_keyboard_get_key();

    if(keyboard == 'A'){
     mode = LEFT;
     straightLineAngle= wb_position_sensor_get_value(encoder);
    } else if(keyboard == 'S'){
       mode = RIGHT;
       straightLineAngle = wb_position_sensor_get_value(encoder);
    } else if(keyboard == 'G')
       mode = AUTONOMUS;
      else if(keyboard == 'W')
       mode = MANUAL;

    if(mode == AUTONOMUS){
     distance_left = searchForObstacles(dist_left);
     distance_right = searchForObstacles(dist_right);
     if(robot_state == GO){
      if(distance_left== FREEWAY && distance_right == FREEWAY) {
        velocity = 8;
        fowardLinearly(wheels, velocity);
      }else if(distance_left== OBSTACLE && distance_right == FREEWAY) {
        robot_state = TURNRIGHT;
        stopWheels(wheels);
      }else if(distance_right == OBSTACLE && distance_left == FREEWAY) {
        robot_state = TURNLEFT;
        stopWheels(wheels);
      }
     }else if(robot_state == TURNRIGHT){
       wheelsTurnLeft(wheels);
       if (distance_left== FREEWAY) {
         robot_state = GO;
      }
     }else if(robot_state == TURNLEFT){
        wheelsTurnRight(wheels);
        if(distance_left== FREEWAY) {
          robot_state = GO;
        }
      }
    }else{
     if(keyboard == WB_KEYBOARD_UP){
       fowardLinearly(wheels, velocity);
     }else if(keyboard == WB_KEYBOARD_DOWN){
       backwardLinearly(wheels);
       angle = wb_position_sensor_get_value(encoder);
     }else if(keyboard == WB_KEYBOARD_RIGHT){
       rightLinearly(wheels);
       angle = wb_position_sensor_get_value(encoder);
     }else if(keyboard == WB_KEYBOARD_LEFT){
       leftlinearly(wheels);
       angle = wb_position_sensor_get_value(encoder);
     }else if(mode == RIGHT){
         wheelsTurnLeft(wheels);
         angle = getAngleRobot(encoder);
         if(angle >= 0.4*PI) {
           robot_state = GO;
           stopWheels(wheels);
         }
     }else if(mode == LEFT){
       wheelsTurnRight(wheels);
       angle = getAngleRobot(encoder);
       if (angle >= 0.4*PI){
         robot_state = GO;
         stopWheels(wheels);
       }
     }else{
       stopWheels(wheels);
     }
    }
  }
wb_robot_cleanup();
return 0;
}

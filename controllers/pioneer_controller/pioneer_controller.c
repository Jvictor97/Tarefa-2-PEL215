/*
 * File:          pioneer_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
 
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64


void delay (int time_milisec) {
  double currentTime, initTime, Timeleft;
  double timeValue = (double)time_milisec/1000;
  initTime = wb_robot_get_time();
  Timeleft =0.00;
  while (Timeleft < timeValue)
  {
    currentTime = wb_robot_get_time();
    Timeleft = currentTime - initTime;
    wb_robot_step(TIME_STEP);
  }
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  // Criando tags para os motores do robô
  WbDeviceTag frontLeftMotor = wb_robot_get_device("front left wheel");   // motor dianteiro esquerdo
  WbDeviceTag frontRightMotor = wb_robot_get_device("front right wheel"); // motor dianteiro direito
  WbDeviceTag backLeftMotor = wb_robot_get_device("back left wheel");    // motor traseiro esquerdo
  WbDeviceTag backRightMotor = wb_robot_get_device("back right wheel");  // motor traseiro direito
  
  WbDeviceTag so7 = wb_robot_get_device("so7");
  WbDeviceTag so6 = wb_robot_get_device("so6");
  WbDeviceTag so0 = wb_robot_get_device("so0");
  WbDeviceTag so1 = wb_robot_get_device("so1");
  
  wb_distance_sensor_enable(so7, TIME_STEP);
  wb_distance_sensor_enable(so6, TIME_STEP);
  wb_distance_sensor_enable(so0, TIME_STEP);
  wb_distance_sensor_enable(so1, TIME_STEP); 
  
  // Configurando os motores
  wb_motor_set_position(frontLeftMotor, INFINITY);
  wb_motor_set_position(frontRightMotor, INFINITY);
  wb_motor_set_position(backLeftMotor, INFINITY);
  wb_motor_set_position(backRightMotor, INFINITY);    
  
  double so7Value, so6Value, so0Value, so1Value;
  
  double currentDistance, minimumDistance, error, 
         integral, errorDifference, oldError;

  while (wb_robot_step(TIME_STEP) != -1) {
 
    so0Value = wb_distance_sensor_get_value(so0); 
    so1Value = wb_distance_sensor_get_value(so1);   
    so6Value = wb_distance_sensor_get_value(so6);
    so7Value = wb_distance_sensor_get_value(so7);
    
    printf("s0: %f s1: %f s2: %f s3: %f s4: %f s5: %f s6: %f s7: %f\n", 
           so0Value, so1Value, so2Value, so3Value, so4Value, so5Value, so6Value,
           so7Value);
    fflush(stdout);
  
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

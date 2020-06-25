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
  Timeleft = 0.00;
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
  WbDeviceTag so5 = wb_robot_get_device("so5");
  WbDeviceTag so0 = wb_robot_get_device("so0");
  WbDeviceTag so1 = wb_robot_get_device("so1");
  WbDeviceTag so2 = wb_robot_get_device("so2");
  WbDeviceTag so3 = wb_robot_get_device("so3");
  
  wb_distance_sensor_enable(so7, TIME_STEP);
  wb_distance_sensor_enable(so6, TIME_STEP);
  wb_distance_sensor_enable(so5, TIME_STEP);
  wb_distance_sensor_enable(so0, TIME_STEP);
  wb_distance_sensor_enable(so1, TIME_STEP); 
  wb_distance_sensor_enable(so3, TIME_STEP);
  wb_distance_sensor_enable(so2, TIME_STEP);
  
  // Configurando os motores
  wb_motor_set_position(frontLeftMotor, INFINITY);
  wb_motor_set_position(frontRightMotor, INFINITY);
  wb_motor_set_position(backLeftMotor, INFINITY);
  wb_motor_set_position(backRightMotor, INFINITY);    
  
  double so7Value, so6Value, so5Value, so0Value, 
         so1Value, so2Value, so3Value;
  
  double currentRightDistance, currentLeftDistance;
  double minimumDistance = 200.0;
  
  /* Variáveis para os erros no controle PID */
  double error, integral = 0.0, errorDifference, oldError = 0.0;
  
  double kp = 0.25;
  double ki = 0.0002;
  double kd = 0.008; 
         
  double maxRightSensorValue, maxLeftSensorValue;
  double motorPower;
  double rightSpeed = 3.0;
  double leftSpeed = 3.0;

  while (wb_robot_step(TIME_STEP) != -1) {
 
    so0Value = wb_distance_sensor_get_value(so0); 
    so1Value = wb_distance_sensor_get_value(so1); 
    so2Value = wb_distance_sensor_get_value(so2); 
    so3Value = wb_distance_sensor_get_value(so3);  
    so5Value = wb_distance_sensor_get_value(so5); 
    so6Value = wb_distance_sensor_get_value(so6);
    so7Value = wb_distance_sensor_get_value(so7);
    
    
    // Condição para realizar um giro de 90 graus
    if (1024 - so3Value <= minimumDistance) {
      wb_motor_set_velocity(frontLeftMotor, -3.0);
      wb_motor_set_velocity(backLeftMotor, -3.0);
      wb_motor_set_velocity(frontRightMotor, 3.0);
      wb_motor_set_velocity(backRightMotor, 3.0);
      
      delay(1200);
      continue;
    }
     
    maxRightSensorValue = so5Value > so6Value ? 
                          (so5Value > so7Value ? so5Value : so7Value) : 
                          (so6Value > so7Value ? so6Value : so7Value);
                          
                          
    maxLeftSensorValue = so0Value > so1Value ?
                         (so0Value > so2Value ? so0Value : so2Value) :
                         (so1Value > so2Value ? so1Value : so2Value);                         
                          
    currentRightDistance = 1024 - maxRightSensorValue;
    currentLeftDistance = 1024 - maxLeftSensorValue;
    
    if (currentLeftDistance < 300) error = currentLeftDistance - currentRightDistance;
    else error = minimumDistance - currentRightDistance;
           
    integral += error;
    errorDifference = error - oldError;
    oldError = error;
    
    motorPower = (kp * error) + (ki * integral) + (kd * errorDifference);
    
    rightSpeed = 3.0 + motorPower;
    
    if (rightSpeed < 1.0) rightSpeed = 1.0;
    if (rightSpeed > 5.0) rightSpeed = 5.0;
       
    wb_motor_set_velocity(frontLeftMotor, leftSpeed);
    wb_motor_set_velocity(backLeftMotor, leftSpeed);
    wb_motor_set_velocity(frontRightMotor, rightSpeed);
    wb_motor_set_velocity(backRightMotor, rightSpeed);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

#ifndef AMR_BASE_CONFIG_H
#define AMR_BASE_CONFIG_H

#define AMR_BASE SKID_STEER      // 4WD robot

#define USE_MDDA10_DRIVER

//#define USE_MPU9250_IMU

#define DEBUG 1

//=================BIGGER ROBOT SPEC (MDDA10)=============================
#define K_P 0.6696  // P constant
#define K_I 7.8341   // I constant
#define K_D 0.012588   // D constant

// define your robot' specs here
#define MAX_RPM 55               // motor's maximum RPM
#define COUNTS_PER_REV 1155      // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.125      // wheel's diameter in meters
#define PWM_BITS 8               // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.45  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.215  // distance between front and back wheels. Ignore this if you're on 2WD/ACKERMANN
//================= END OF BIGGER ROBOT SPEC =============================


#define MOTOR1_ENCODER_A 13
#define MOTOR1_ENCODER_B 12 

#define MOTOR2_ENCODER_A 5
#define MOTOR2_ENCODER_B 23 

#define MOTOR3_ENCODER_A 18
#define MOTOR3_ENCODER_B 19 

#define MOTOR4_ENCODER_A 16
#define MOTOR4_ENCODER_B 17


  #define MOTOR_DRIVER MDD10A  

  #define MOTOR1_PWM 26 // Changed to a valid PWM pin
  #define MOTOR1_IN_A 32
  #define MOTOR1_CHANNEL 0

  #define MOTOR2_PWM 27 // Changed to a valid PWM pin
  #define MOTOR2_IN_A 33
  #define MOTOR2_CHANNEL 1

  #define MOTOR3_PWM 25 // Changed to a valid PWM pin
  #define MOTOR3_IN_A 15
  #define MOTOR3_CHANNEL 6

  #define MOTOR4_PWM 14 // Changed to a valid PWM pin
  #define MOTOR4_IN_A 4

  #define MOTOR4_CHANNEL 7

  #define PWM_MAX (pow(2, PWM_BITS) - 1)
  #define PWM_MIN (-PWM_MAX)


#define FREQ 5000
#define RES  8


#endif


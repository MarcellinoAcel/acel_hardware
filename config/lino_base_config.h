// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define LED_PIN 13 // used for debugging status

// uncomment the base you're building
//  #define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
//  #define LINO_BASE SKID_STEER            // 4WD robot
//  #define LINO_BASE MECANUM               // Mecanum drive robot
#define LINO_BASE OMNI

// uncomment the motor driver you're using
//  #define USE_GENERIC_2_IN_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
//  #define USE_GENERIC_1_IN_MOTOR_DRIVER      // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
#define USE_BTS7960_MOTOR_DRIVER // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER               // Motor ESC for brushless motors

// uncomment the IMU you're using
//  #define USE_GY85_IMU
//  #define USE_MPU6050_IMU
//  #define USE_MPU9150_IMU
//  #define USE_MPU9250_IMU
#define USE_BNO055_IMU

#define K_P 90            // 55
#define K_I 366.101694915 // 0.045454545
#define K_D 0             // 0

#define drib_kp 0.5 // 1  //0.9 //1.2
#define drib_ki 0   // 0 //0.0001 // 0.00006
#define drib_kd 0.2 // 0  //0 //0.000015

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD1/MECANUM)
         BACK
*/

/*
ROBOT ORIENTATION OMNI
FRONT = X
LEFT  = Y
          FRONT
     MOTOR1    MOTOR4
     MOTOR2    MOTOR3
          BACK
*/

// define your robot' specs here
#define MOTOR_MAX_RPS 8.4               // motor's max RPM
#define MAX_RPS_RATIO 1.3               // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 24      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24 // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 3840            // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 3840            // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 3840            // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 3840            // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.0985           // wheel's diameter in meters
#define ROBOT_DIAMETER 0.46             // distance between left and right wheels
#define ROBOT_RADIUS 0.23
#define PWM_BITS 8
#define PWM_FREQUENCY 20000

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV false
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

#define MOTOR3_ENCODER_A 14
#define MOTOR3_ENCODER_B 15

#define MOTOR4_ENCODER_A 28
#define MOTOR4_ENCODER_B 29

#define MOTOR1_ENCODER_A 17
#define MOTOR1_ENCODER_B 16

#define MOTOR2_ENCODER_A 26
#define MOTOR2_ENCODER_B 27

// dribble motor
#define dribble_enc_a 31
#define dribble_enc_b 30
#define dribble_cw 11
#define dribble_ccw 12

// catcher motor
#define catcher_cw 36
#define catcher_ccw 37
#define prox_start 7
#define prox_end 32

// servo
#define servo 8

// MOTOR PINS

// #define MOTOR1_PWM -1  // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR3_IN_A 18 // 2
#define MOTOR3_IN_B 19 // 2

// #define MOTOR2_PWM -1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR4_IN_A 22 // 4
#define MOTOR4_IN_B 10 // 4

// #define MOTOR3_PWM -1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR1_IN_A 5 // 1
#define MOTOR1_IN_B 6 // 1

// #define MOTOR4_PWM -1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR2_IN_A 3
#define MOTOR2_IN_B 4

const int cw[6] = {MOTOR1_IN_A, MOTOR2_IN_A, MOTOR3_IN_A, MOTOR4_IN_A, dribble_cw};
const int ccw[6] = {MOTOR1_IN_B, MOTOR2_IN_B, MOTOR3_IN_B, MOTOR4_IN_B, dribble_ccw};

/*
1 = 18, 19
2 = 5,  6
3 = 22, 23
4 = 3,  4

real by elect
1 = 18, 19
2 = 22,  23
3 = 6, 5
4 = 3,  4

3 = 18, 19
4 = 5,  6
2 = 22, 23
1 = 3,  4
*/
// const int pwm[4] ={-1,-1,-1,-1};
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

struct but
{
     int A;
     int B;
     int X;
     int Y;
     int RT;
     int LT;
     int LB;
     int RB;
} button;

#endif

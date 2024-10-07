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

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#define RPM_TO_RPS 1/60

class Kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE, SKID_STEER, MECANUM, OMNI};

        base base_platform_;

        struct rps
        {
            float motor1;
            float motor2;
            float motor3;
            float motor4;
        };
        
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };
        struct position{
            float linear_x;
            float linear_y;
            float angular_z;
        };

        struct pwm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };
        
        Kinematics(base robot_base, int motor_max_rps, float max_rps_ratio,
                   float motor_operating_voltage, float motor_power_max_voltage,
                   float wheel_diameter, float wheels_y_distance);
        
        velocities getVelocities(float rps1, float rps2, float rps3, float rps4);
        
        position getPosition(float enc_x, float enc_y);
        
        velocities getVelExternal(float enc1, float enc2);
        
        rps getRPS(float linear_x, float linear_y, float angular_z);
        
        float getMaxRPS();
        float toRad(float deg);

    private:
        rps calculateRPS(float linear_x, float linear_y, float angular_z);
        int getTotalWheels(base robot_base);

        // keliling roda encoder 4.635 cm
        // 1024 ppr
        float max_rps_;
        float wheels_y_distance_;
        float pwm_res_;
        float wheel_circumference_;
        int total_wheels_;
        float robot_circumference_ = 0.25;
        float enc_wheel_diameter = 4.635 / 100;
        float total_enc_pulse = 1024;
        float L = 20.325/100;
        float enc_wheel_circumference = enc_wheel_diameter * PI;
};

#endif
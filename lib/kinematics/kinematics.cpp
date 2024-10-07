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

#include "Arduino.h"
#include "kinematics.h"
#include "math.h"


float Kinematics::toRad(float deg)
{
    return deg * M_PI / 180;
}

Kinematics::Kinematics(base robot_base, int motor_max_rps, float max_rps_ratio,
                       float motor_operating_voltage, float motor_power_max_voltage,
                       float wheel_diameter, float wheels_y_distance) : base_platform_(robot_base),
                                                                        wheels_y_distance_(wheels_y_distance),
                                                                        wheel_circumference_(PI * wheel_diameter),
                                                                        total_wheels_(getTotalWheels(robot_base))
{
    motor_power_max_voltage = constrain(motor_power_max_voltage, 0, motor_operating_voltage);
    max_rps_ = ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rps) * max_rps_ratio;
}

Kinematics::rps Kinematics::calculateRPS(float linear_x, float linear_y, float angular_z)
{

    // float tangential_vel = angular_z * (robot_circumference_);
    float tangential_vel = angular_z;

    // convert m/s to m/min
    float linear_vel_x_mins = linear_x;
    float linear_vel_y_mins = linear_y;
    // convert rad/s to rad/min
    float tangential_vel_mins = tangential_vel;

    float x_rps = linear_vel_x_mins;
    float y_rps = linear_vel_y_mins;
    float tan_rps = tangential_vel_mins;

    Kinematics::rps rps;

    // calculate for the target motor rps and direction
    // front-left motor
    float rps_motor1 = sin(toRad(45)) * x_rps + cos(toRad(45)) * y_rps + robot_circumference_ * tan_rps;
    rps.motor1 = fmax(-max_rps_, fmin(rps_motor1, max_rps_));

    // front-right motor
    float rps_motor2 = sin(toRad(135)) * x_rps + cos(toRad(135)) * y_rps + robot_circumference_ * tan_rps;
    rps.motor2 = fmax(-max_rps_, fmin(rps_motor2, max_rps_));

    // rear-left motor
    float rps_motor3 = sin(toRad(225)) * x_rps + cos(toRad(225)) * y_rps + robot_circumference_ * tan_rps;
    rps.motor3 = fmax(-max_rps_, fmin(rps_motor3, max_rps_));

    // rear-right motor
    float rps_motor4 = sin(toRad(315)) * x_rps + cos(toRad(315)) * y_rps + robot_circumference_ * tan_rps;
    rps.motor4 = fmax(-max_rps_, fmin(rps_motor4, max_rps_));

    return rps;
}

Kinematics::rps Kinematics::getRPS(float linear_x, float linear_y, float angular_z)
{
    return calculateRPS(linear_x, linear_y, angular_z);
}

Kinematics::velocities Kinematics::getVelocities(float rps1, float rps2, float rps3, float rps4)
{
    Kinematics::velocities vel;
    float average_rps_x;
    float average_rps_y;
    float average_rps_a;

    //convert average revolutions per minute to revolutions per second
    average_rps_x = ((float)(sin(toRad(45)) * rps1 + sin(toRad(135)) * rps2 + sin(toRad(225)) * rps3 + sin(toRad(315)) * rps4) / total_wheels_); // rps
    vel.linear_x = average_rps_x * wheel_circumference_; // m/s

    //convert average revolutions per minute in y axis to revolutions per second
    average_rps_y = ((float)(cos(toRad(45)) * rps1 + cos(toRad(135)) * rps2 + cos(toRad(225)) * rps3 + cos(toRad(315)) * rps4) / total_wheels_); // rps
    vel.linear_y = average_rps_y * wheel_circumference_; // m/s

    //convert average revolutions per minute to revolutions per second
    average_rps_a = ((float)(-rps1 + rps2 - rps3 + rps4) / total_wheels_);
    vel.angular_z =  (average_rps_a * wheel_circumference_) / (robot_circumference_); //  rad/s

    return vel;
}

Kinematics::position Kinematics::getPosition(float enc_x, float enc_y){
    Kinematics::position pos;
    float estimate_x;
    float estimate_y;
    float estimate_h;

    estimate_x = (enc_x / total_enc_pulse) * enc_wheel_circumference;
    estimate_y = (enc_y / total_enc_pulse) * enc_wheel_circumference;

    // Using atan2 to handle quadrant issues and prevent division by zero
    estimate_h = atan2(estimate_y, estimate_x);

    pos.linear_x = estimate_x;
    pos.linear_y = estimate_y;
    pos.angular_z = estimate_h;

    return pos;
}
Kinematics::velocities Kinematics::getVelExternal(float vel_x, float vel_y){
    Kinematics::velocities velEx;
    float estimate_x;
    float estimate_y;
    float estimate_h;

    estimate_x = -vel_x*enc_wheel_circumference;
    estimate_y = vel_y*enc_wheel_circumference;
    estimate_h = atan2(vel_y, vel_x);
    
    velEx.linear_x = estimate_x;
    velEx.linear_y = estimate_y;
    velEx.angular_z = estimate_h;

    return velEx;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch (robot_base)
    {
    case DIFFERENTIAL_DRIVE:
        return 2;
    case SKID_STEER:
        return 4;
    case MECANUM:
        return 4;
    case OMNI:
        return 4;
    default:
        return 4;
    }
}

float Kinematics::getMaxRPS()
{
    return max_rps_;
}
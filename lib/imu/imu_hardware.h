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

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

// include IMU base interface
#include "imu_interface.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// include sensor API headers
//  #include "I2Cdev.h"
//  #include "ADXL345.h"
//  #include "ITG3200.h"
//  #include "HMC5883L.h"
//  #include "MPU6050.h"
//  #include "MPU9150.h"
//  #include "MPU9250.h"
class BNO055_IMU : public IMUInterface
{
    Adafruit_BNO055 bno;

public:
    BNO055_IMU() : bno(Adafruit_BNO055(55, 0x28, &Wire2)) {}

    bool startSensor() override
    {
        // Memulai sensor BNO055
        if (!bno.begin())
        {
            // Jika gagal inisialisasi, kembalikan false
            return false;
        }

        // Menunggu sensor untuk melakukan kalibrasi internal
        delay(1000);

        // Setel mode sensor (opsional, tergantung pada aplikasi Anda)
        bno.setExtCrystalUse(true);

        return true; // Inisialisasi berhasil
    }
    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        // Membaca data akselerasi dari sensor BNO055
        sensors_event_t event;
        bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);

        geometry_msgs__msg__Vector3 accel;
        accel.x = event.acceleration.x * g_to_accel_; // Konversi ke m/s²
        accel.y = event.acceleration.y * g_to_accel_;
        accel.z = event.acceleration.z * g_to_accel_;

        return accel;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        // Membaca data giroskop dari sensor BNO055
        sensors_event_t event;
        bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);

        geometry_msgs__msg__Vector3 gyro;
        gyro.x = event.gyro.x; // Data sudah dalam rad/s
        gyro.y = event.gyro.y;
        gyro.z = event.gyro.z;

        return gyro;
    }
};
#endif
#include <Arduino.h>
#include <imu_interface.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
class BNO055IMU : public IMU_INTERFACE
{
private:
    // constants specific to the sensor
    const float accel_scale_ = 1 / 256.0;
    const float gyro_scale_ = 1 / 14.375;

    // driver objects to be used
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

    // returned vector for sensor reading
    geometry_msgs__msg__Vector3 accel_;
    geometry_msgs__msg__Vector3 gyro_;

public:
    BNO055IMU()
    {
        // accel_cov_ = 0.001; //you can overwrite the convariance values here
        // gyro_cov_ = 0.001; //you can overwrite the convariance values here
    }

    bool startSensor() override
    {
        // here you can override startSensor() function and use the sensor's driver API
        // to initialize and test the sensor's connection during boot time
        if (!bno.begin())
        {
            return false;
        }
        return true;
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        // here you can override readAccelerometer function and use the sensor's driver API
        // to grab the data from accelerometer and return as a Vector3 object

        sensors_event_t angVelocityData;
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

        // accelerometer_.getAcceleration(&ax, &ay, &az);

        accel_.x = angVelocityData.acceleration.x * (double)accel_scale_ * g_to_accel_;
        accel_.y = angVelocityData.acceleration.y * (double)accel_scale_ * g_to_accel_;
        accel_.z = angVelocityData.acceleration.z * (double)accel_scale_ * g_to_accel_;

        return accel_;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        // here you can override readAccelerometer function and use the sensor's driver API
        // to grab the data from gyroscope and return as a Vector3 object
        sensors_event_t orient;

        bno.getEvent(&orient, Adafruit_BNO055::VECTOR_EULER);

        gyro_.x = orient.orientation.x * (double)gyro_scale_ * DEG_TO_RAD;
        gyro_.y = orient.orientation.y * (double)gyro_scale_ * DEG_TO_RAD;
        gyro_.z = orient.orientation.z * (double)gyro_scale_ * DEG_TO_RAD;

        return gyro_;
    }
};
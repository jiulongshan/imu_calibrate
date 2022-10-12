/*
 * File: imu_data_handle.hpp
 * Created: 2020-10-16 17:10:86
 * Author: Hongbing.luo (hongbing.luo@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#ifndef _SRC_IMU_DATA_HANDLE_HPP
#define _SRC_IMU_DATA_HANDLE_HPP

#include <atomic>
#include <vector>
#include <mutex>
#include "utils.hpp"

namespace imu_calibration {

const int IMU_DIMENSION_SIZE = 3;

enum class IMU_DATA_TYPE{
    IMU_OUTPUT_TIME = 0x50,
    IMU_OUTPUT_ACCELEROMETER = 0x51,
    IMU_OUTPUT_ANGULARVELOCITY = 0x52,
    IMU_OUTPUT_ANGLE = 0x53,
    IMU_OUTPUT_MAGNETOMETER = 0x54,
    IMU_OUTPUT_PORTSTATUS = 0x55,
    IMU_OUTPUT_CLIMATE = 0x56,
    IMU_OUTPUT_LATITUDE = 0x57,
    IMU_OUTPUT_GROUNDSPEED = 0x58,
    IMU_OUTPUT_QUATERNION = 0x59,
    IMU_OUTPUT_SATELITEPOS = 0x5a,
};


using namespace std;


class ImuDataCenter
{
public:
    static ImuDataCenter& GetInstance();

public:    
    ~ImuDataCenter();

    void SetImuMeterValues(char buf[]);
    std::vector<float> GetAccelerometer();
    std::vector<float> GetAngularVelocity();
    std::vector<float> GetAngleValue();
    std::vector<int> GetMagnetometer();

private:
    ImuDataCenter();
    std::atomic<float>  m_accelerometer[IMU_DIMENSION_SIZE];
    std::atomic<float>  m_angular_velocity[IMU_DIMENSION_SIZE];
    std::atomic<float>  m_angle[IMU_DIMENSION_SIZE];
    std::atomic<int>    m_magnetometer[IMU_DIMENSION_SIZE];

    // static std::mutex m_smtx;
    // static ImuDataCenter* m_singleton;
};

}
#endif
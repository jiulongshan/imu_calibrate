/*
 * File: imu_data_handl.cpp
 * Created: 2020-10-16 18:10:88
 * Author: Hongbing.luo (hongbing.luo@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#include "imu_data_handle.hpp"

namespace imu_calibration {

static std::mutex m_smtx;
static ImuDataCenter* m_singleton = nullptr;

ImuDataCenter& ImuDataCenter::GetInstance()
{
    if(m_singleton){
        return *m_singleton;
    }

    {
        lock_guard<std::mutex> lck(m_smtx);
        if (!m_singleton)
            m_singleton = new ImuDataCenter();
    }

    return *m_singleton; 
}

ImuDataCenter::ImuDataCenter()
{
    for(int i=0;i<IMU_DIMENSION_SIZE;i++){
        m_accelerometer[i] = 0.0f;
        m_angular_velocity[i] = 0.0f;
        m_angle[i] = 0.0f;
        m_magnetometer[i] = 0;
    }
}

ImuDataCenter::~ImuDataCenter()
{
}

std::vector<float> ImuDataCenter::GetAccelerometer()
{   
    std::vector<float> acces;
    for(int i=0;i<IMU_DIMENSION_SIZE;i++){
        acces.push_back(m_accelerometer[i].load());
    }
    
    return acces;
}

std::vector<float> ImuDataCenter::GetAngularVelocity()
{
    std::vector<float> angular_vels;
    for(int i=0;i<IMU_DIMENSION_SIZE;i++){
        angular_vels.push_back(m_angular_velocity[i].load());
    }
    
    return angular_vels;
}

std::vector<float> ImuDataCenter::GetAngleValue()
{
    std::vector<float> angles;
    for(int i=0;i<IMU_DIMENSION_SIZE;i++){
        angles.push_back(m_angle[i].load());
    }
    
    return angles;
}

std::vector<int> ImuDataCenter::GetMagnetometer()
{
    std::vector<int> magnes;
    for(int i=0;i<IMU_DIMENSION_SIZE;i++){
        magnes.push_back(m_magnetometer[i].load());
    }
    
    return magnes;
}

void ImuDataCenter::SetImuMeterValues(char buf[])
{
    short x = 0;    //X
    short y = 0;    //Y
    short z = 0;    //Z

    IMU_DATA_TYPE type = IMU_DATA_TYPE(buf[1]);
    switch (type)
    {
    case IMU_DATA_TYPE::IMU_OUTPUT_ACCELEROMETER:
        x = ((short)buf[3]<<8)|buf[2];
        m_accelerometer[0].store((float)x/32768*16*9.8);
        y = ((short)buf[5]<<8)|buf[4];
        m_accelerometer[1].store((float)y/32768*16*9.8);
        z = ((short)buf[7]<<8)|buf[6];
        m_accelerometer[2].store( (float)z/32768*16*9.8);
        // UBT_LOG(MSG_INFO, "%hd, %hd, %hd\n", x, y, z);
        UBT_LOG(MSG_INFO, "Acce: %02f, %02f, %02f\n", m_accelerometer[0].load(), 
            m_accelerometer[1].load(), m_accelerometer[2].load());
        break;

    case IMU_DATA_TYPE::IMU_OUTPUT_ANGULARVELOCITY:
        x = ((short)buf[3]<<8)|buf[2];
        m_angular_velocity[0].store((float)x/32768*2000);
        y = ((short)buf[5]<<8)|buf[4];
        m_angular_velocity[1].store( (float)y/32768*2000);
        z = ((short)buf[7]<<8)|buf[6];
        m_angular_velocity[2].store((float)z/32768*2000);
        // UBT_LOG(MSG_INFO, "%hd, %hd, %hd\n", x, y, z);
        UBT_LOG(MSG_INFO, "Angular velocity: %02f, %02f, %02f\n", m_angular_velocity[0].load(), 
            m_angular_velocity[1].load(), m_angular_velocity[2].load());
        break;

    case IMU_DATA_TYPE::IMU_OUTPUT_ANGLE:
        x = ((short)buf[3]<<8)|buf[2];
        m_angle[0].store((float)x/32768*180);
        y = ((short)buf[5]<<8)|buf[4];
        m_angle[1].store((float)y/32768*180);
        z = ((short)buf[7]<<8)|buf[6];
        m_angle[2].store((float)z/32768*180);
        // UBT_LOG(MSG_INFO, "%hd, %hd, %hd\n", x, y, z);
        UBT_LOG(MSG_INFO, "Angles: %02f, %02f, %02f\n", m_angle[0].load(), m_angle[1].load(), m_angle[2].load());
        break;

    case IMU_DATA_TYPE::IMU_OUTPUT_MAGNETOMETER:
        x = ((short)buf[3]<<8)|buf[2];
        m_magnetometer[0].store((int)x);
        y = ((short)buf[5]<<8)|buf[4];
        m_magnetometer[1].store((int)y);
        z = ((short)buf[7]<<8)|buf[6];
        m_magnetometer[2].store((int)z);
        // UBT_LOG(MSG_INFO, "%hd, %hd, %hd\n", x, y, z);
        UBT_LOG(MSG_INFO, "Magnetometer: %d, %d, %d\n", m_magnetometer[0].load(), m_magnetometer[1].load(), m_magnetometer[2].load());
        break;
    
    default:
        UBT_LOG(MSG_INFO, "default data type is %d\n", IMU_DATA_TYPE(buf[1]));
        break;
    }
}

}

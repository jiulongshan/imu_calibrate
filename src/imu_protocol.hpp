/*
 * File: imu_protocol.hpp
 * Created: 2020-10-15 10:10:84
 * Author: Hongbing.luo (hongbing.luo@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */


#ifndef _SRC_IMU_PROTOCOL_HPP
#define _SRC_IMU_PROTOCOL_HPP
#include <functional>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <list> 
#include <vector>
#include <chrono>
#include <condition_variable> 

#include "serialport.hpp"
#include "utils.hpp"
#include "linear_ringbuffer.hpp"


using namespace std;

namespace imu_calibration {

enum class CALIBRATE_STATUS : uint32_t{
    ST_READY = 0,
    ST_START = 1,
    ST_ERROR = 4,
};

enum class IMU_READ_RATE{
    IMU_0_1HZ = 0x00,
    IMU_0_2HZ = 0x01,
    IMU_0_HZ = 0x02,
    IMU_1_HZ = 0x03,
    IMU_2_HZ = 0x04,
    IMU_5_HZ = 0x05,
    IMU_10_HZ = 0x06,
    IMU_20_HZ = 0x07,
    IMU_50_HZ = 0x08,
    IMU_100_HZ = 0x09,
    IMU_NULL_HZ = 0x0A,
    IMU_200_HZ = 0x0B,
    IMU_SINGLE_HZ = 0x0C,
};

enum class IMU_BAUD_RATE{
    IMU_BAUDRATE_9600 = 0x02,
    IMU_BAUDRATE_115200 = 0x06,
};

class ImuProtocol
{
    static void ThreadReadIMU(ImuProtocol* imu_protocol_ptr);
    static void ThreadConsumeIMUData(ImuProtocol* imu_protocol_ptr);

public:
    typedef std::function<void(CALIBRATE_STATUS)> status_cb_t;

    ImuProtocol(uint32_t baudrate = B115200);
    ~ImuProtocol();

    bool Init();
    
    bool StartCaptureData();
    bool EnterUnlockCmd();
    bool EnterCalAccelerometer();
    bool CompleteCalAccelerometer();
    bool SaveConfigData();
    bool SetReadRate(IMU_READ_RATE rrate);
    bool SetBaudRate(IMU_BAUD_RATE brate);
    bool EnterCalMagnerometer();
    bool CompleteCalMagnerometer();

    bool AutoCalibrationAccelerometer();
    bool AutoCalibrationMagnerometer();

private:
    void OnThreadReadIMU();
    void OnThreadConsumeIMUData();

    shared_ptr<std::thread>  m_thread_produce, m_thread_consume;
    shared_ptr<Serialport>  m_serialport;
    // shared_ptr<ImuDataCenter> m_data_handle;
    std::mutex m_mtx;
    std::condition_variable m_cond;
    CALIBRATE_STATUS m_status;

    shared_ptr<linear_ringbuffer> m_io_buffer;
    uint32_t m_framerate;
    uint32_t m_baudrate;
    std::chrono::system_clock::time_point m_tp;
};



}
#endif
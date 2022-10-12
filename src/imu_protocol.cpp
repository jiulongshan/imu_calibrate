/*
 * File: imu_protocol.cpp
 * Created: 2020-10-15 10:10:96
 * Author: Hongbing.luo (hongbing.luo@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#include "imu_protocol.hpp"
#include "imu_data_handle.hpp"

namespace imu_calibration {

#define SINGLE_INTERIGATED_LEN 11
#define IO_BUFFER_SIZE 4096

ImuProtocol::ImuProtocol(uint32_t baudrate):m_framerate(100),
    m_status(CALIBRATE_STATUS::ST_READY)
{
    // Init();
    m_baudrate = baudrate;
}

ImuProtocol::~ImuProtocol()
{
    m_serialport->Release();
}

bool ImuProtocol::Init()
{
    m_serialport = make_shared<Serialport>(IOInterface::DeviceType::SerialUsbImu, m_baudrate);
    m_io_buffer = make_shared<linear_ringbuffer>(IO_BUFFER_SIZE);
    // m_data_handle = make_shared<ImuDataCenter>();

    if(!m_serialport->Init()){
        m_status = CALIBRATE_STATUS::ST_ERROR;
        return false;
    }
    m_status = CALIBRATE_STATUS::ST_START;
    return true;
}

bool ImuProtocol::StartCaptureData()
{   
    m_thread_produce = make_shared<std::thread>(ImuProtocol::ThreadReadIMU, this);
    m_thread_consume = make_shared<std::thread>(ImuProtocol::ThreadConsumeIMUData, this);
    m_thread_produce->detach();
    m_thread_consume->detach();
    return true;
}

bool ImuProtocol::EnterUnlockCmd()
{
    if(m_status != CALIBRATE_STATUS::ST_START)
        return false;

    UBT_LOG(MSG_RELEASE, "ImuProtocol::EnterUnlockCmd \n");
    char buf[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5,};
    m_serialport->Write(buf, sizeof(buf));
    return true;
}

bool ImuProtocol::EnterCalAccelerometer()
{
    if(m_status != CALIBRATE_STATUS::ST_START)
        return false;

    m_tp = std::chrono::system_clock::now();
    char buf[5] = {0xFF, 0xAA, 0x01, 0x01, 0x00,};
    m_serialport->Write(buf, sizeof(buf));
    return true;
}

bool ImuProtocol::CompleteCalAccelerometer()
{
    if(m_status != CALIBRATE_STATUS::ST_START)
        return false;

    auto tp_now =  std::chrono::system_clock::now();
    auto interval = std::chrono::duration_cast<std::chrono::seconds>(tp_now-m_tp).count();
    UBT_LOG(MSG_INFO, "interval is %d\n", interval);
    char buf[5] = {0xFF, 0xAA, 0x01, 0x00, 0x00,};
    if(interval > 3 && m_framerate >= 100){
         m_serialport->Write(buf, sizeof(buf));
         return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval*1000));
    m_serialport->Write(buf, sizeof(buf));
    return true;
}

bool ImuProtocol::SaveConfigData()
{
    if(m_status != CALIBRATE_STATUS::ST_START)
        return false;

    UBT_LOG(MSG_INFO, "ImuProtocol::SaveConfigData\n");
    char buf[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00,};
    m_serialport->Write(buf, sizeof(buf));
    return true;
}

bool ImuProtocol::SetReadRate(IMU_READ_RATE rrate)
{
    if(EnterUnlockCmd()){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        UBT_LOG(MSG_RELEASE, "IMU_READ_RATE is %d\n", rrate);
        char buf[5] = {0xFF, 0xAA, 0x03, char(rrate), 0x00,};
        m_serialport->Write(buf, sizeof(buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        SaveConfigData();
        return true;
    }

    return false;
}

bool ImuProtocol::SetBaudRate(IMU_BAUD_RATE brate)
{
    if(EnterUnlockCmd()){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        UBT_LOG(MSG_RELEASE, "IMU_BAUD_RATE is %d\n", brate);
        char buf[5] = {0xFF, 0xAA, 0x04, char(brate), 0x00,};
        m_serialport->Write(buf, sizeof(buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        SaveConfigData();
        return true;
    }
    return false;
}

void ImuProtocol::OnThreadReadIMU()
{
    char buf[512] = {'\0'}; // use ringbuffer
    long allread = 0;
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto byte_read = m_serialport->Read(buf, sizeof(buf));
        allread+=byte_read;
        UBT_LOG(MSG_INFO, "byte_read: %d, all: %ld\n", byte_read, allread);

        if(byte_read < 0){
            UBT_LOG(MSG_ERROR, "byte_read: %d\n", byte_read);
            continue;
        }
        std::unique_lock<std::mutex> lck(m_mtx);
        ::memcpy(m_io_buffer->write_head(), buf, byte_read);
        m_io_buffer->commit(byte_read);
         
        if(m_io_buffer->size() > SINGLE_INTERIGATED_LEN*2){
            m_cond.notify_all();
        }
    }
}

void ImuProtocol::ThreadConsumeIMUData(ImuProtocol* imuptr)
{
    auto imu_protocol_ptr = imuptr;
    imu_protocol_ptr->OnThreadConsumeIMUData();
}

void ImuProtocol::ThreadReadIMU(ImuProtocol* imuptr)
{
    auto imu_protocol_ptr = imuptr;
    imu_protocol_ptr->OnThreadReadIMU();
}

void ImuProtocol::OnThreadConsumeIMUData()
{
    char buf[32] = {'\0'}; // use ringbuffer
    // char tail_intact[SINGLE_INTERIGATED_LEN] = {0};
    bool count_on = false;
    int useless_count = 0;

    while(true){
        std::unique_lock<std::mutex> lck(m_mtx);
        auto len = m_io_buffer->size();
        if (len < SINGLE_INTERIGATED_LEN * 2 ) {
            UBT_LOG(MSG_INFO, "waiting ----------  %d\n", len);
            m_cond.wait(lck);
        }
        len = m_io_buffer->size();
        UBT_LOG(MSG_INFO, "go ----------  %d\n", len);

        // 处理异常情况: 首次接收的前一部分无效数据
        while(!count_on && len--){
            char c = *(m_io_buffer->read_head());
            if(c == 0x55){
                m_io_buffer->consume(useless_count);
                count_on = true;
                break;
            }else{
                useless_count++;
            }
        }

        if(!count_on){            
            UBT_LOG(MSG_INFO, "useless ----------  %d\n", useless_count);
            m_io_buffer->consume(useless_count);
            useless_count = 0;
            continue;
        }
        //end 处理异常情况
        
        ::memcpy(buf, m_io_buffer->read_head(), SINGLE_INTERIGATED_LEN);
        m_io_buffer->consume(SINGLE_INTERIGATED_LEN);

        // annotation, could diable this codes
        for(int i=0;i<SINGLE_INTERIGATED_LEN;i++){
            UBT_LOG(MSG_INFO, "%02x ", buf[i]);
        }
        UBT_LOG(MSG_INFO, "\n");

        ImuDataCenter::GetInstance().SetImuMeterValues(buf);
    }
}

}
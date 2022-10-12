/*
 * File: test.cpp
 * Created: 2020-10-15 16:10:05
 * Author: Hongbing.luo (hongbing.luo@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#include <iostream>
#include <unistd.h>
#include "imu_protocol.hpp"
#include "imu_data_handle.hpp"

using namespace std;
using namespace imu_calibration;

int main(int args, char* argv[])
{
    printf("run help: \n");
    printf("run `export IMU_LOGLEVLE=0`, for all print log in this terminal. \n");

    auto imu = make_shared<ImuProtocol>();
    if(!imu->Init()){
        return 0;
    }
    imu->StartCaptureData();

// test01
    // imu->EnterUnlockCmd();
    // imu->EnterCalAccelerometer();
    // sleep(3);
    // imu->CompleteCalAccelerometer();
    // imu->SaveConfigData();


// test02
    // imu->SetReadRate(IMU_READ_RATE::IMU_100_HZ);

// test03, Get data
    UBT_LOG(MSG_RELEASE, "begin loop.\n");
    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::vector<float> acces =  ImuDataCenter::GetInstance().GetAccelerometer();
        for(auto itor : acces){
            UBT_LOG(MSG_RELEASE, "%f ", itor);
        }
        UBT_LOG(MSG_RELEASE, "******************************************\n");
        
        std::vector<float> angles =  ImuDataCenter::GetInstance().GetAngleValue();
        for(auto itor : angles){
            UBT_LOG(MSG_RELEASE, "%f ", itor);
        }
        UBT_LOG(MSG_RELEASE, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    }
    return 0;
}
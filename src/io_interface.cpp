/*
 * File: io_interface.cpp
 * Created: 2020-08-10 21:08:01
 * Author: KunYang (kun.yang@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 * 
 */

#ifndef _SRC_COMMON_IO_INTERFACE
#define _SRC_COMMON_IO_INTERFACE

#include "io_interface.hpp"
#include <map>

namespace imu_calibration {

namespace {
    std::map<IOInterface::DeviceType, std::string> s_dev_map = {
        {IOInterface::DeviceType::SerialUsbImu, std::string("/dev/ttyUSB0")},     
    };
    
}

IOInterface::IOInterface(const DeviceType &dev)
    : m_dev_path(s_dev_map[dev])
{
}

}

#endif /* _SRC_COMMON_IO_INTERFACE */

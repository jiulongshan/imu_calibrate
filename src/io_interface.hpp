/*
 * File: io_interface.hpp
 * Created: 2020-08-10 16:08:00
 * Author: KunYang (kun.yang@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#ifndef _SRC_IO_INTERFACE_INTERFACE
#define _SRC_IO_INTERFACE_INTERFACE

#include <stdint.h>
#include <string>

namespace imu_calibration {

class IOInterface
{
public:
    enum class DeviceType{
        SerialUsbImu,
        DeviceTypeNone, // 增加设备类型时，要保证DeviceTypeNone为enum最后一项
    };

    IOInterface(const DeviceType &dev);
    virtual ~IOInterface() = default;

    /*
    *@函数名称： Init
    *@函数功能： 初始化IO设备
    *@返 回 值： true 成功
                false 失败
    */
    virtual bool Init() = 0;

    /**
     * @函数名称:  Release
     * @函数功能:  释放IO设备
     * @返 回 值： true 成功
                 false 失败
     */
    virtual bool Release() = 0;

    /*
    *@函数名称： Read
    *@函数功能： 从IO设备读取数据
    *@输入参数： buf_size(int)  buf数组大小 
    *@输出参数： buf(char[]) 保存数据的字符数组
    *@返 回 值： 实际读取数据长度
    */
    virtual int32_t Read(char buf[], uint32_t buf_size, uint32_t timeout) = 0;

    /*
    *@函数名称： Write
    *@函数功能： 向IO设备写数据
    *@输入参数： data(const char *)  要写入的数据
    *@输出参数： data_size(int)  data数据长度
    *@返 回 值： 实际写入的数据长度
    */
    virtual int32_t Write(const char *data, uint32_t data_size) = 0;

protected:
    std::string m_dev_path;

};

}

#endif /* _SRC_IO_INTERFACE_INTERFACE */

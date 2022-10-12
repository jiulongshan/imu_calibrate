/*
 * File: serialport.hpp
 * Created: 2020-08-10 16:08:52
 * Author: KunYang (kun.yang@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#ifndef _SRC_IO_INTERFACE_SERIALPORT
#define _SRC_IO_INTERFACE_SERIALPORT

#include "io_interface.hpp"
#include <termios.h>
#include <string>

namespace imu_calibration {

class Serialport : public IOInterface
{
public:
    /*
     * 奇偶校验
    */
    enum class Parity
    {
        ParityNone = 0,
        ParityOdd = 1,
        ParityEven = 2,
        ParityMark = 3,
        ParitySpace = 4
    };

    /*
     * 停止位
    */
    enum class Stopbits
    {
        StopbitsOne = 1,
        StopbitsTwo = 2,
    };

    /*
     * 数据位数
    */
    enum class ByteSize
    {
        Fivebits = 5,
        Sixbits = 6,
        Sevenbits = 7,
        Eightbits = 8
    };

    /*
     * 流控
    */
    enum class FlowControl
    {
        FlowControlNone = 0,
        FlowControlSoftware,
        FlowControlHardware
    };

    /**
     * @函数名称: Serialport
     * @函数功能: 构造函数
     * @输入参数: dev(DeviceType) 设备类型
     *           baudrate(speed_t) 波特率
     *           bytesize(ByteSize) 数据位数
     *           parity(Parity) 奇偶校验
     *           stopbits(Stopbits) 停止位
     *           flowcontrol(FlowControl) 流控
     * @返 回 值: 无
     */
    Serialport(const DeviceType &dev,
	    speed_t baudrate = B115200,
		ByteSize bytesize = ByteSize::Eightbits,
	    Parity parity = Parity::ParityNone,
	    Stopbits stopbits = Stopbits::StopbitsOne,
	    FlowControl flowcontrol = FlowControl::FlowControlNone
	);

    virtual ~Serialport() = default;

    /*
    *@函数名称： Init
    *@函数功能： 初始化, 打开串口设备， 设置串口属性
    *@返 回 值： true 成功
               false 失败
    */
    virtual bool Init() override;

    /**
     * @函数名称:  Release
     * @函数功能:  释放IO设备
     * @返 回 值： true 成功
                 false 失败
     */
    virtual bool Release() override;

    /*
    *@函数名称： Read
    *@函数功能： 从IO设备读取数据
    *@输入参数： buf_size(int)  buf数组大小 
    *@输出参数： buf(char[]) 保存数据的字符数组
    *@返 回 值： 实际读取数据长度, 出错时返回-1
    */
    virtual int32_t Read(char buf[], uint32_t buf_size, uint32_t timeout = 20) override;

    /*
    *@函数名称： Write
    *@函数功能： 向IO设备写数据
    *@输入参数： data(const char *)  要写入的数据
    *@输出参数： data_size(int)  data数据长度
    *@返 回 值： 实际写入的数据长度, 出错时返回-1
    */
    virtual int32_t Write(const char *data, uint32_t data_size) override;

private:

    /**
     * @函数名称:  Open
     * @函数功能:  打开串口设备
     * @返 回 值:  true  成功
     *            false 失败
     */
    bool Open();

    /**
     * @函数名称:  Setup
     * @函数功能:  设置串口
     * @返 回 值:  true  成功
     *            false 失败
     */
	bool Setup();

    /*
     * 设置串口属性
     */
	void SetupBytesize(struct termios &tconfig);
	void SetupParity(struct termios &tconfig);
	void SetupStopbits(struct termios &tconfig);
	void SetupFlowcontrol(struct termios &tconfig);

	// 打开的串口设备文件描述符
	int32_t m_fd;
	// 波特率
	speed_t m_baudrate;
	// 奇偶校验
	Parity m_parity;
	// 数据位
	ByteSize m_bytesize;
	// 停止位
	Stopbits m_stopbits;
	// 流控
	FlowControl m_flowcontrol;
};

}
#endif /* _SRC_IO_INTERFACE_SERIALPORT */

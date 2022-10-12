/*
 * File: serialport.cpp
 * Created: 2020-08-10 16:08:64
 * Author: KunYang (kun.yang@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#include "serialport.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include "utils.hpp"

namespace imu_calibration {

Serialport::Serialport(const DeviceType &dev,
        speed_t baudrate,
		ByteSize bytesize,
        Parity parity,
        Stopbits stopbits,
        FlowControl flowcontrol)
        : IOInterface(dev), m_fd(-1), m_baudrate(baudrate), m_parity(parity), 
        m_bytesize(bytesize), m_stopbits(stopbits), m_flowcontrol(flowcontrol)
{
	
}

bool Serialport::Init()
{
    if (!Open())
    {
        UBT_LOG(MSG_ERROR, "Failed to open serial port %s\n", m_dev_path.c_str());
        return false;
    }
    UBT_LOG(MSG_RELEASE, "Open serial port %s\n", m_dev_path.c_str());

    return Setup();
}

bool Serialport::Release()
{
    if (m_fd > 0)
    {
        int ret = close(m_fd);
        if (0 == ret)
        {
            return true;
        }
    }

    return false;
}

int32_t Serialport::Read(char buf[], uint32_t buf_size, uint32_t timeout)
{	
    if (m_fd < 0)
    {
        UBT_LOG(MSG_ERROR, "Fd not valid : %d\n", m_fd);
        return -1;
    }

    if(nullptr == buf)
    {
        UBT_LOG(MSG_ERROR, "Receive buffer is null\n");
        return -1;
    }

	int               nfds;
	fd_set            readfds;
    struct timeval    tv;
	tv.tv_sec = 0;
    tv.tv_usec = timeout*1000;
    FD_ZERO(&readfds);
    FD_SET(m_fd, &readfds);
	// nfds = select(m_fd+1, &readfds, NULL, NULL, &tv);
	if(nfds <= 0){
		return 0;
	}
	ssize_t bytes_read = ::read(m_fd, buf, buf_size);
	// for(int i=0;i<bytes_read;i++)
	// {
	// 	UBT_LOG(MSG_INFO, "%02x ", buf[i]);
	// }
	// UBT_LOG(MSG_INFO, "\n");

	return bytes_read;
}

int32_t Serialport::Write(const char *data, uint32_t data_size)
{
    if(nullptr == data || 0 == data_size)
	{
		return -1;
	}

	ssize_t bytes_write = ::write(m_fd, data, data_size);

	return bytes_write;
}

bool Serialport::Open()
{
	if (m_fd > 0)
	{
		// 设备已经打开
		return false;
	}

	// 非阻塞读
	m_fd = ::open(m_dev_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (m_fd == -1) {
		switch (errno) {
		case EINTR:
			static int32_t retry = 0;
			// 再尝试打开
			if(++retry < 3)
			{
				usleep(10 * 1000);
				return Open();
			}
			return false;

		case ENFILE:
		case EMFILE:
		default:
			return false;
		}
	}

	return true;
}

bool Serialport::Setup()
{
	if(-1 == m_fd)
	{
		UBT_LOG(MSG_ERROR, "Fd is -1\n");
		return false;
	}

	struct termios tty_config;
	memset(&tty_config, 0, sizeof(tty_config));
	// 获取串口当前配置
	if(tcgetattr(m_fd, &tty_config) != 0)
	{
		UBT_LOG(MSG_ERROR, "tcgetattr error : %s\n", strerror(errno));
		return false;
	}

	// 更改配置
	// 奇偶校验
	SetupParity(tty_config);
	// 停止位
	SetupStopbits(tty_config);
	// 字符大小
	SetupBytesize(tty_config);
	// 流控
	SetupFlowcontrol(tty_config);

	// 接收使能，本地模式
	tty_config.c_cflag |= CREAD | CLOCAL;
	// 原始输出
	tty_config.c_oflag &= ~OPOST;
	// 原始输入模式
	tty_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty_config.c_iflag &= ~ICRNL;
	// 波特率
	cfsetispeed(&tty_config, m_baudrate);
	cfsetospeed(&tty_config, m_baudrate);

	// 更新配置
	tcflush(m_fd, TCIFLUSH);

	if(tcsetattr(m_fd, TCSANOW, &tty_config) != 0)
	{
		UBT_LOG(MSG_ERROR, "tcsetattr error : %s\n", strerror(errno));
		return false;
	}

	return true;
}

void Serialport::SetupBytesize(struct termios &tconfig)
{
	tconfig.c_cflag &= (tcflag_t) ~CSIZE;
	switch (m_bytesize)
	{
	case ByteSize::Fivebits:
		tconfig.c_cflag |= CS5;
		break;
	case ByteSize::Sixbits:
		tconfig.c_cflag |= CS6;
		break;
	case ByteSize::Sevenbits:
		tconfig.c_cflag |= CS7;
		break;
	case ByteSize::Eightbits:
	default:
		tconfig.c_cflag |= CS8;
		break;
	}
}

void Serialport::SetupParity(struct termios &tconfig)
{
	tconfig.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
	if (m_parity == Parity::ParityNone) {
		tconfig.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
	} else if (m_parity == Parity::ParityEven) {
		tconfig.c_cflag &= (tcflag_t) ~(PARODD);
		tconfig.c_cflag |=  (PARENB);
	} else if (m_parity == Parity::ParityOdd) {
		tconfig.c_cflag |=  (PARENB | PARODD);
	}
}

void Serialport::SetupStopbits(struct termios &tconfig)
{
	if (m_stopbits == Stopbits::StopbitsOne)
		tconfig.c_cflag &= (tcflag_t) ~(CSTOPB);
	else if(m_stopbits == Stopbits::StopbitsTwo)
		tconfig.c_cflag |=  (CSTOPB);
}

void Serialport::SetupFlowcontrol(struct termios &tconfig)
{
	if(m_flowcontrol == FlowControl::FlowControlNone)
	{
		tconfig.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
		tconfig.c_cflag &= (uint32_t) ~(CRTSCTS);
	}
	else if(m_flowcontrol == FlowControl::FlowControlSoftware)
	{
		tconfig.c_iflag |=  (IXON | IXOFF);
		tconfig.c_cflag &= (uint32_t) ~(CRTSCTS);
	}
	else if(m_flowcontrol == FlowControl::FlowControlHardware)
	{
		tconfig.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
		tconfig.c_cflag |=  (CRTSCTS);
	}
}

}
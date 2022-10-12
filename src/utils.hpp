/*
 * File: utils.hpp
 * Created: 2020-10-15 10:10:90
 * Author: Hongbing.luo (hongbing.luo@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#ifndef _SRC_COMMON_UTILS_HPP
#define _SRC_COMMON_UTILS_HPP

#include <unistd.h>
#include <string>
#include <chrono>
#include <sstream>
#include <stdarg.h>
#include <sys/stat.h>

using namespace std;

namespace imu_calibration {

#define ESC_START     "\033["
#define DBG_FG_BLACK "\033[30m"
#define DBG_FG_RED "\033[31m"
#define DBG_FG_GREEN "\033[32m"
#define DBG_FG_YELLOW "\033[33m"
#define DBG_FG_BLUE "\033[34m"
#define DBG_FG_VIOLET "\033[35m"
#define DBG_FG_VIRDIAN "\033[36m"
#define DBG_FG_WHITE "\033[37m"
#define DBG_BG_BLACK "\033[40m"
#define DBG_BG_RED "\033[41m"
#define DBG_BG_GREEN "\033[42m"
#define DBG_BG_YELLOW "\033[43m"
#define DBG_BG_BLUE "\033[44m"
#define DBG_BG_VIOLET "\033[45m"
#define DBG_BG_VIRDIAN "\033[46m"
#define DBG_BG_WHITE "\033[47m"
#define DBG_END "\033[0m"

typedef enum _IMU_DEBUG_LEVEL_
{
    MSG_INFO,
    MSG_DEBUG,
    MSG_WARN,
    MSG_ERROR,
    MSG_RELEASE,
    MSG_NUMBER
}IMU_DEBUG_LEVEL;

void UBT_LOG(IMU_DEBUG_LEVEL DebugLevel, const char *format, ...);

}
#endif
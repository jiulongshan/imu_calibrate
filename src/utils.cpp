/*
 * File: utils.cpp
 * Created: 2020-10-15 10:10:72
 * Author: Hongbing.luo (hongbing.luo@ubtrobot.com)
 * Copyright 2020 - 2020 Ubtech Robotics Corp. All rights reserved.
 * -----
 * Description:
 */

#include "utils.hpp"

namespace imu_calibration {

void UBT_LOG(IMU_DEBUG_LEVEL DebugLevel, const char *format, ...) 
{
    unsigned int DebugLevelTmp = DebugLevel;
    char Buffer[1024] = {0}, LogBuffer[1024] = {0};
    char *env_debuglevel = getenv("IMU_LOGLEVEL");
    int log_level = MSG_ERROR;
    if (env_debuglevel){
        stringstream sstr(env_debuglevel);

        sstr >> log_level;
        //printf("log_level is %d\n", log_level);
    }

    if(DebugLevelTmp < log_level)
        return ;
    switch(DebugLevelTmp)
    {
        case MSG_INFO:
            printf("%s",DBG_FG_GREEN);
            break;
        case MSG_DEBUG:
            printf("%s",DBG_FG_GREEN);
            break;
        case MSG_WARN:
            printf("%s",DBG_FG_YELLOW);
            break;
        case MSG_ERROR:
            printf("%s",DBG_FG_RED);
            break;
        case MSG_NUMBER:
            printf("%s",DBG_FG_RED);
            break;
        default:
            printf("%s",DBG_FG_GREEN);
            break;
    }
    va_list arg_ptr;
    va_start(arg_ptr, format);
    vprintf(format, arg_ptr);
    vsprintf(Buffer, format, arg_ptr);
    va_end(arg_ptr);
    printf("%s",DBG_END);
}

}
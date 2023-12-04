#pragma once

#include <cstring>
#include <iostream>
#include "def.h"

//// filename: depends on platform
#if defined __GNUC__ || defined LINUX
#define COMMON_FILENAME_ (strrchr(__FILE__, '/') + 1)
#else
#define COMMON_FILENAME_ (strrchr(__FILE__, '\\') + 1)
#endif
// weird: when not found, strrchr returns 0x1 instead of NULL
#define COMMON_FILENAME_STR ((unsigned long long)COMMON_FILENAME_ < 0x2 ? "" : COMMON_FILENAME_)

//// time str: depends on OS
COMMON_FORCE_INLINE char* COMMON_GetTimeString() {
    static char str[24];
    static struct tm now_time;
    time_t time_seconds = time(0);
#ifdef _WIN32
    localtime_s(&now_time, &time_seconds);
#elif defined LINUX
    localtime_r(&time_seconds, &now_time);
#endif
    sprintf(str, "%04d-%02d-%02d %02d:%02d:%02d",
        now_time.tm_year + 1900,
        now_time.tm_mon + 1,
        now_time.tm_mday,
        now_time.tm_hour,
        now_time.tm_min,
        now_time.tm_sec);
    return str;
}

//// log
extern const char* COMMON_LOG_COLORS[5];
extern const char* COMMON_LOG_COLOR_RESET;
#define COMMON_LOG_(type) \
    std::cout << COMMON_LOG_COLORS[(int)(type)] << \
    "[" << COMMON_GetTimeString() << "]" \
    "[" << COMMON_FILENAME_STR << ":" << __LINE__ << "] " << \
    COMMON_LOG_COLOR_RESET
#define COMMON_LOG_DEBUG COMMON_LOG_(common::LogType::Debug)
#define COMMON_LOG_INFO COMMON_LOG_(common::LogType::Info)
#define COMMON_LOG_WARN COMMON_LOG_(common::LogType::Warn)
#define COMMON_LOG_ERROR COMMON_LOG_(common::LogType::Error)
#define COMMON_LOG_FATAL COMMON_LOG_(common::LogType::Fatal)

namespace common {
    enum LogType { Debug = 0, Info, Warn, Error, Fatal };
} // namespace common
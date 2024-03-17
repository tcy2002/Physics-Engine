#pragma once

#include <cstring>
#include <iostream>
#include "common/general.h"

//// filename: depends on platform
#if defined __GNUC__ || defined LINUX
#define PE_FILENAME_ (strrchr(__FILE__, '/') + 1)
#else
#define PE_FILENAME_ (strrchr(__FILE__, '\\') + 1)
#endif
// weird: when not found, strrchr returns 0x1 instead of NULL
#define PE_FILENAME_STR ((unsigned long long)PE_FILENAME_ < 0x2 ? "" : PE_FILENAME_)

//// time str: depends on OS
COMMON_FORCE_INLINE char* PE_GetTimeString() {
    static char str[24];
    static struct tm now_time;
    time_t time_seconds = time(0);
#ifdef _WIN32
    localtime_s(&now_time, &time_seconds);
#elif defined LINUX
    localtime_r(&time_seconds, &now_time);
#endif
    sprintf_s(str, "%04d-%02d-%02d %02d:%02d:%02d",
        now_time.tm_year + 1900,
        now_time.tm_mon + 1,
        now_time.tm_mday,
        now_time.tm_hour,
        now_time.tm_min,
        now_time.tm_sec);
    return str;
}

//// log
extern const char* PE_LOG_COLORS[5];
extern const char* PE_LOG_COLOR_RESET;
#define PE_LOG_(type) \
    std::cout << PE_LOG_COLORS[(int)(type)] << \
    "[" << PE_GetTimeString() << "]" \
    "[" << PE_FILENAME_STR << ":" << __LINE__ << "]" << \
    PE_LOG_COLOR_RESET
#define PE_LOG_DEBUG PE_LOG_(PE_LogType::Debug) << " DEBUG: "
#define PE_LOG_INFO PE_LOG_(PE_LogType::Info) << " INFO: "
#define PE_LOG_WARN PE_LOG_(PE_LogType::Warn) << " WARN: "
#define PE_LOG_ERROR PE_LOG_(PE_LogType::Error) << " ERROR: "
#define PE_LOG_FATAL PE_LOG_(PE_LogType::Fatal) << " FATAL: "

enum PE_LogType { Debug = 0, Info, Warn, Error, Fatal };
#include "common/logger.h"

using namespace common;

int main() {
    std::cout << COMMON_FILENAME_STR << std::endl;
    COMMON_LOG_DEBUG << "debug" << std::endl;
    COMMON_LOG_INFO << "info" << std::endl;
    COMMON_LOG_WARN << "warn" << std::endl;
    COMMON_LOG_ERROR << "error" << std::endl;
    COMMON_LOG_FATAL << "fatal" << std::endl;
    return 0;
}

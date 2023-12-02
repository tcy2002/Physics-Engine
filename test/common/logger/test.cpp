#include "common/logger.h"

using namespace pe_common;

int main() {
    std::cout << PE_FILENAME_STR << std::endl;
    PE_LOG_DEBUG << "debug" << std::endl;
    PE_LOG_INFO << "info" << std::endl;
    PE_LOG_WARN << "warn" << std::endl;
    PE_LOG_ERROR << "error" << std::endl;
    PE_LOG_FATAL << "fatal" << std::endl;
    return 0;
}

file(GLOB_RECURSE PE_COMMON_SOURCES
        logger.cpp thread_pool.cpp)

add_library(PECommon STATIC ${PE_COMMON_SOURCES})

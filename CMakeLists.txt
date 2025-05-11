cmake_minimum_required(VERSION 3.31)
project(EurobotCoreMath)

set(CMAKE_CXX_STANDARD 20)
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS include/*.h)
add_library(EurobotCoreMath INTERFACE)
target_include_directories(EurobotCoreMath
        INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

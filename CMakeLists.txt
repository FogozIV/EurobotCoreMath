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
include(GNUInstallDirs)
install(
        TARGETS EurobotCoreMath
        EXPORT EurobotCoreMath-targets
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

# ---- Export for Downstream Projects ----
install(
        EXPORT EurobotCoreMath-targets
        FILE EurobotCoreMath-config.cmake
        NAMESPACE CoreMath::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/EurobotCoreMath
)
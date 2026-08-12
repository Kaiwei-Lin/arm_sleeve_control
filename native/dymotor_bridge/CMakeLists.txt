cmake_minimum_required(VERSION 3.15)

if(NOT UNIX OR APPLE)
    message(FATAL_ERROR "The bundled DyMotor SDK is a Linux .so; build this bridge on Linux.")
endif()

find_package(Threads REQUIRED)

set(DYMOTOR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/dymotor_sdk")
set(DYMOTOR_SDK_LIBRARY "${DYMOTOR_SDK_DIR}/lib/libMotorDrive.so")
if(NOT EXISTS "${DYMOTOR_SDK_LIBRARY}")
    message(FATAL_ERROR "libMotorDrive.so was not found under ${DYMOTOR_SDK_DIR}/lib")
endif()

add_library(dymotor_bridge SHARED dymotor_bridge.c)
target_compile_features(dymotor_bridge PRIVATE c_std_99)
target_compile_options(dymotor_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(
    dymotor_bridge
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
    PRIVATE
        "${DYMOTOR_SDK_DIR}/include"
        "${DYMOTOR_SDK_DIR}/include/c"
        "${DYMOTOR_SDK_DIR}/include/robot_motor"
        "${DYMOTOR_SDK_DIR}/ThirdParty/include"
        "${DYMOTOR_SDK_DIR}/ThirdParty/include/eigen3"
)
target_link_libraries(dymotor_bridge PRIVATE "${DYMOTOR_SDK_LIBRARY}" Threads::Threads m)

set_target_properties(
    dymotor_bridge
    PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/native/dymotor_bridge"
        BUILD_RPATH "\$ORIGIN/../../../third_party/dymotor_sdk/lib"
        INSTALL_RPATH "\$ORIGIN/../../../third_party/dymotor_sdk/lib"
)

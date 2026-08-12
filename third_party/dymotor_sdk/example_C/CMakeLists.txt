cmake_minimum_required(VERSION 3.15)

project(Control)
find_package(Threads REQUIRED)
if(WIN32)
    add_compile_options(/source-charset:utf-8)
endif()

if(UNIX AND NOT APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -finput-charset=UTF-8")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -finput-charset=UTF-8")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#添加库文件搜索路径
link_directories(${PROJECT_SOURCE_DIR}/../lib)


# 添加头文件搜索路径
#手动指定yaml-cpp头文件的路径
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/include)
include_directories(${PROJECT_SOURCE_DIR}/../ThirdParty/include/eigen3)

set(Demo_HEAD_FILES
)


set(Demo_CPP_FILES
#   ./set_id.c
    # ./get_motor_list.c
#    ./current_control_get_pvct.c
    # ./velcity_control_get_pvct.c
    # ./current_control_get_pvct.c
    # ./pose_control_get_pvct.c
    # ./pd_control_get_pvct.c
    # ./set_control_world_calibration.c
    ./set_sensor_less.c
    # ./scan_mainEditionInfo.c
    #  ./mother_board_firmware_ota.c
    # ./motor_firmware_ota.c
    # ./set_motor_zero.c
    # ./set_joint_zero_limit_calibration_test.c
)


add_executable(${PROJECT_NAME} 
${Demo_HEAD_FILES} 
${Demo_CPP_FILES} 
)

# 链接SDK库和其他依赖
target_link_libraries(${PROJECT_NAME} 
    PRIVATE 
    MotorDrive
    Threads::Threads
)


# 设置头文件搜索路径
target_include_directories(${PROJECT_NAME} 
    PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/c
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/robot_motor
)




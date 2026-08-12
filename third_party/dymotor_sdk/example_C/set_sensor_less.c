/*
 * set_sensor_less.c
 * 功能：强拖功能，设置电机为转速模式，使能电机，使用无感（开环）模式开启强拖，设置转速为 2
 * @author  xufuliang
 * @date    2025-08-27
 */

#include <stdio.h>
#include "RobotControl.h"
#include "Common.h"
int main(int argc, char* argv[]) {
    //生成主板对象
    RobotCtx* ctx = robot_create(0xFD);
    if (!ctx) return -1;
    //连接主板
    int value = robot_config_net(ctx, "192.168.3.245", 15021, 14999, "192.168.3.11");
    if (value)
    {
        printf("连接主板成功\n");
    }
    else
    {
        printf("连接主板失败\n");
    }
    //生成电机对象
    RobotMotor* motor = robot_create_motor(ctx, 13, 1);



    while (1)
    {
        //设置电机开环控制模式
        value = robot_motor_set_control_mode(motor, MOTOR_CTRL_MODE_OPENLOOP);

        if (value)
        {
            printf("set_control_mode success \n");
            break;
        }
        else
        {
            printf("set_control_mode failed \n");
        }
    }

    while (1)
    {
        //设置电机使能
        value = robot_motor_set_control_world(motor, CTRL_SERVO_ON);

        if (value)
        {
            printf("set_control_world success\n");
            break;
        }
        else
        {
            printf("set_control_world failed\n");
        }
    }

#ifdef _WIN32
    Sleep(2000);
#else
    sleep(2);
#endif

    //设置无感模式强拖，电压为2.4
    if (robot_motor_set_SetVoltageValue(motor, 2.4))
    {
    
        printf("set_StrongDragging success\n");
    }
    else 
    {
        printf("set_StrongDragging failed\n");
    }

    //释放电机对象
    robot_destroy_motor(ctx, motor);
    //释放主板对象
    robot_destroy(ctx);
    return 0;


}
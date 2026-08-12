/*
 * set_motor_zero.c
 * 功能： 设置电机位置置零
 * @author  xufuliang
 * @date    2026-03-17
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

    //设置偏移角度
    value = robot_motor_off_set_angle(motor,0);

    if(value)
    {
        printf("设置偏移角度 成功\n");
    }
    else
    {

        printf("设置偏移角度 失败\n");
    }
    
    //设置电机位置置零
    int return_value = robot_motor_set_control_world(motor, CTRL_POSITION_SET_ZERO);
    if (return_value)
    {
        printf("执行器位置置零 成功\n");
    }
    else
    {

        printf("执行器位置置零 失败\n");
    }
   
    //释放电机对象
    robot_destroy_motor(ctx,motor);
    //释放主板对象
    robot_destroy(ctx);

    return 0;

}
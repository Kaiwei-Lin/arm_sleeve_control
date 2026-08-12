/*
 * set_id.c
 * 功能： 设置电机id
 * @author  xufuliang
 * @date    2026-02-27
 */

#include <stdio.h>
#include "RobotControl.h"
int main(int argc, char* argv[]) {
    RobotCtx* ctx = robot_create(0xFD);
    if (!ctx) return -1;

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
    RobotMotor* motor = robot_create_motor(ctx, 13, 2);
    int return_value = robot_motor_set_motor_id(motor, 6);
    if (return_value)
    {
        printf("设置电机ID成功\n");
    }
    else 
    {

        printf("设置电机ID失败\n");
    }

  
    
    robot_destroy_motor(ctx,motor);
        
    robot_destroy(ctx);

    return 0;

}
   






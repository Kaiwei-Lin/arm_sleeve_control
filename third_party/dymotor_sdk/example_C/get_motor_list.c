/*
 * get_motor_list.c
 * 功能： 获取电机列表
 * @author  xufuliang
 * @date    2026-02-26
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
    
    RobotMotorListHandle motorlist = motorlist_create();
    //获取电机信息列表
    int return_value = get_robot_motorlist(ctx,motorlist);
    if (return_value)
    {
        printf("获取电机列表 成功\n");
    }
    else
    {

        printf("获取电机列表 失败\n");
    }

    MotorArray motorArray;
    //注意初始化数量为0
    motorArray.count = 0;
    //创建电机对象列表
    robot_create_motorObjectList(ctx,motorlist,&motorArray);
    

    printf("motorArray.count:%d\n",motorArray.count);
    int i = 0 ;
    for(i=0;i<motorArray.count;i++)
    {
        unsigned short id ,lineId;
        robot_motor_get_motor_id(motorArray.robotmotors[i],&id,&lineId);
        printf("id: %d,canid: %d\n",id,lineId);

    }
    //释放电机对象数组
    motorObjectlist_destroy(ctx,&motorArray);
    //释放电机信息列表对象
    motorlist_destroy(motorlist);
    //释放主板对象   
    robot_destroy(ctx);

        


    return 0;

}
   






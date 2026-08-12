/*
 * mother_board_firmware_ota.c
 * 功能：升级主板固件ota
 * @author  xufuliang
 * @date    2025-08-27
 */

#include <stdio.h>
#include "RobotControl.h"
#include "Common.h"

void func(int num, int error) 
{
    printf("num:%d,error:%d\n", num, error);

}
int main() 
{
    //生成主板对象
    RobotCtx* ctx = robot_create(0xFD);
    if (!ctx) return -1;
    //连接主板
    if (robot_config_net(ctx, "192.168.3.245", 15021, 14999, "192.168.3.11"))
    {
        printf("连接主板成功\n");
    }
    else
    {
        printf("连接主板失败\n");
        //释放主板对象
        robot_destroy(ctx);
        return -1;
    }
    char version[128] = {'\0'};
    
    if (robot_motor_get_mother_board_firmware_version(ctx, version))
    {
        printf("获取固件版本成功:%s\n",version);
    }
    else 
    {
        printf("获取固件版本失败:%s\n",version);
        //释放主板对象
        robot_destroy(ctx);
        return -1; 
    }
    int value = robot_mother_board_one_click_ota_upgradeing(ctx, "./MainControlBoard_56.0.0.0.bin", func);
    if ( value== 1) 
    {
      
        printf("升级成功\n");
    }
    else if(value == -1)
    {
        printf("握手失败\n");
    }
    else if (value == -2)
    {
        printf("发送开始传输标志失败\n");
    }
    else if (value == -3)
    {
        printf("发送传输文件失败\n");
    }
    else if (value == -4)
    {
        printf("停止传输失败\n");
    }
    if (robot_motor_get_mother_board_firmware_version(ctx, version))
    {
        printf("获取固件版本成功:%s\n", version);
    }
    else
    {
        printf("获取固件版本失败:%s\n", version);
        //释放主板对象
        robot_destroy(ctx);
        return -1;
    }

    //释放主板对象
    robot_destroy(ctx);
    
	return 0;
}
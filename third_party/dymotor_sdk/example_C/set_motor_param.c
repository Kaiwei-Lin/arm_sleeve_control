/*
 * motor_param.c
 * 功能： 获取电机列表
 * @author  xufuliang
 * @date    2026-03-16
 */

#include <stdio.h>
#include "RobotControl.h"
#include "Common.h"

int main()
{
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
        return -1;
    }

    //生成电机对象
    RobotMotor* motor = robot_create_motor(ctx, 1, 2);
    /****************************************************************** */
    //执行器类型参数
    ActuatorType_e type;

    if(robot_motor_get_actuator_type_param(motor,&type))
    {
        printf("获取电机 执行器类型参数 成功\n");

    }
    else
    {
        printf("获取电机 执行器类型参数 失败");
    }

    if(robot_motor_set_actuator_type_param(motor,type))
    {
        printf("设置电机 执行器类型参数 成功\n");
    }
    else
    {
        printf("设置电机 执行器类型参数 失败\n");
    }
    /****************************************************************** */
    //执行器基础配置
    ActuatorBsaeParam_t base_param;

    if(robot_motor_get_actuator_bsae_param(motor,&base_param))
    {
        printf("获取电机 执行器基础配置 成功\n");

    }
    else
    {
        printf("获取电机 执行器基础配置 失败");
    }

    if(robot_motor_set_actuator_bsae_param(motor,base_param))
    {
        printf("设置电机 执行器基础配置 成功\n");
    }
    else
    {
        printf("设置电机 执行器基础配置 失败\n");
    }
    /****************************************************************** */
    //执行器传感器参数
    ActuatorSensorParam1_t sensor_param1;

    if(robot_motor_get_actuator_sensor_param1(motor,&sensor_param1))
    {
        printf("获取电机 执行器传感器参数 成功\n");

    }
    else
    {
        printf("获取电机 执行器传感器参数 失败");
    }

    if(robot_motor_set_actuator_sensor_param1(motor,sensor_param1))
    {
        printf("设置电机 执行器传感器参数 成功\n");
    }
    else
    {
        printf("设置电机 执行器传感器参数 失败\n");
    }

     //执行器传感器参数
    ActuatorSensorParam2_t sensor_param2;

    if(robot_motor_get_actuator_sensor_param2(motor,&sensor_param2))
    {
        printf("获取电机 执行器传感器参数 成功\n");

    }
    else
    {
        printf("获取电机 执行器传感器参数 失败");
    }

    if(robot_motor_set_actuator_sensor_param2(motor,sensor_param2))
    {
        printf("设置电机 执行器传感器参数 成功\n");
    }
    else
    {
        printf("设置电机 执行器传感器参数 失败\n");
    }
    /****************************************************************** */
    //执行器控制参数
    ActuatorControlParam_t control_params;

    if(robot_motor_get_actuator_control_params(motor,&control_params))
    {
        printf("获取电机 执行器控制参数 成功\n");

    }
    else
    {
        printf("获取电机 执行器控制参数 失败");
    }

    if(robot_motor_set_actuator_control_params(motor,control_params))
    {
        printf("设置电机 执行器控制参数 成功\n");
    }
    else
    {
        printf("设置电机 执行器控制参数 失败\n");
    }
    /****************************************************************** */
    //执行器功能配置
    ActuatorFuncParam_t func_param;

    if(robot_motor_get_actuator_func_param(motor,&func_param))
    {
        printf("获取电机 执行器功能配置 成功\n");

    }
    else
    {
        printf("获取电机 执行器功能配置 失败");
    }

    if(robot_motor_set_actuator_func_param(motor,func_param))
    {
        printf("设置电机 执行器功能配置 成功\n");
    }
    else
    {
        printf("设置电机 执行器功能配置 失败\n");
    }
    /****************************************************************** */
    //电机参数
    MotorParam_t motor_param;

    if(robot_motor_get_motor_param(motor,&motor_param))
    {
        printf("获取电机 电机参数 成功\n");

    }
    else
    {
        printf("获取电机 电机参数 失败");
    }

    if(robot_motor_set_motor_param(motor,motor_param))
    {
        printf("设置电机 电机参数 成功\n");
    }
    else
    {
        printf("设置电机 电机参数 失败\n");
    }
    /****************************************************************** */
    //硬件参数
    HardwareParam_t hardware_param;

    if(robot_motor_get_hardware_param(motor,&hardware_param))
    {
        printf("获取电机 硬件参数 成功\n");

    }
    else
    {
        printf("获取电机 硬件参数 失败");
    }

    if(robot_motor_set_hardware_param(motor,hardware_param))
    {
        printf("设置电机 硬件参数 成功\n");
    }
    else
    {
        printf("设置电机 硬件参数 失败\n");
    }
    /****************************************************************** */
    //强拖控制参数
    ForceParam_t force_param;

    if(robot_motor_get_force_param(motor,&force_param))
    {
        printf("获取电机 强拖控制参数 成功\n");

    }
    else
    {
        printf("获取电机 强拖控制参数 失败");
    }

    if(robot_motor_set_force_param(motor,force_param))
    {
        printf("设置电机 强拖控制参数 成功\n");
    }
    else
    {
        printf("设置电机 强拖控制参数 失败\n");
    }
    /****************************************************************** */
    //无感控制参数
    FulxObsParam_t fulxObs_param;

    if(robot_motor_get_fulxObs_param(motor,&fulxObs_param))
    {
        printf("获取电机 无感控制参数 成功\n");

    }
    else
    {
        printf("获取电机 无感控制参数 失败");
    }

    if(robot_motor_set_fulxObs_param(motor,fulxObs_param))
    {
        printf("设置电机 无感控制参数 成功\n");
    }
    else
    {
        printf("设置电机 无感控制参数 失败\n");
    }
    /****************************************************************** */
    //电流力矩转换三次多项式系数
    TorqueCalibParam_t torqueCalib_param;

    if(robot_motor_get_torqueCalib_param(motor,&torqueCalib_param))
    {
        printf("获取电机 电流力矩转换三次多项式系数 成功\n");

    }
    else
    {
        printf("获取电机 电流力矩转换三次多项式系数 失败");
    }

    if(robot_motor_set_torqueCalib_param(motor,torqueCalib_param))
    {
        printf("设置电机 电流力矩转换三次多项式系数 成功\n");
    }
    else
    {
        printf("设置电机 电流力矩转换三次多项式系数 失败\n");
    }
    /****************************************************************** */
    //摩擦参数
    FrictionParam_t friction_param;

    if(robot_motor_get_friction_param(motor,&friction_param))
    {
        printf("获取电机 摩擦参数 成功\n");

    }
    else
    {
        printf("获取电机 摩擦参数 失败");
    }

    if(robot_motor_set_friction_param(motor,friction_param))
    {
        printf("设置电机 摩擦参数 成功\n");
    }
    else
    {
        printf("设置电机 摩擦参数 失败\n");
    }
    /****************************************************************** */
    //摄氏度电流系数
    TempCurParam_t tempCur_param;

    if(robot_motor_get_tempCur_param(motor,&tempCur_param))
    {
        printf("获取电机 摄氏度电流系数 成功\n");

    }
    else
    {
        printf("获取电机 摄氏度电流系数 失败");
    }

    if(robot_motor_set_tempCur_param(motor,tempCur_param))
    {
        printf("设置电机 摄氏度电流系数 成功\n");
    }
    else
    {
        printf("设置电机 摄氏度电流系数 失败\n");
    }
    /****************************************************************** */
    //安全保护模块
    ErrDectParam_t errDect_param;

    if(robot_motor_get_errDect_param(motor,&errDect_param))
    {
        printf("获取电机 安全保护模块 成功\n");

    }
    else
    {
        printf("获取电机 安全保护模块 失败");
    }

    if(robot_motor_set_errDect_param(motor,errDect_param))
    {
        printf("设置电机 安全保护模块 成功\n");
    }
    else
    {
        printf("设置电机 安全保护模块 失败\n");
    }

    if(robot_motor_set_sys_save_to_flash(motor))
    {
        printf("电机保存参数 成功\n");
    }
    else
    {
        printf("电机保存参数 失败\n");
    }
    robot_destroy_motor(ctx,motor);
    robot_destroy(ctx);
    
   


    return 0;
}
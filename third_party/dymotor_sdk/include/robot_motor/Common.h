#ifndef COMMON_H
#define COMMON_H
#include <stdint.h>
typedef enum {

    CTRL_NONE = 0,

    CTRL_SERVO_OFF = 1,    // 失能
    CTRL_SERVO_ON = 2,     // 使能
    CTRL_CLEAR_FAULT = 3,  // 清除错误

    RESET_DEFAULT = 4,  // 恢复默认参数

    // 传感器相关
    CTRL_CALIBRATE_ENCODER = 5,  // 编码器校准
    CTRL_RETURN_ZERO = 6 ,      // 执行器自动回零
    CTRL_POSITION_SET_ZERO = 7, // 执行器位置置零

    // 控制相关
    CTRL_FRICTION_IDENTIFY = 8,  // 摩擦校准
    CTRL_COGGING_IDENTIFY = 9,   // 齿槽转矩校准

    // 功能开关 0x3xxxF
    CTRL_FRICTION_COMP_OFF = 10,       // 失能摩擦补偿
    CTRL_FRICTION_COMP_ON = 11,        // 使能摩擦补偿
    CTRL_COGGING_COMP_OFF = 12,        // 失能齿槽转矩补偿
    CTRL_COGGING_COMP_ON = 13,         // 使能齿槽转矩补偿
    CTRL_TLOAD_COMP_OFF = 14,          // 失能负载补偿
    CTRL_TLOAD_COMP_ON = 15,           // 使能负载补偿
    CTRL_SOFT_POS_LIMIT_OFF = 16,      // 失能软限位
    CTRL_SOFT_POS_LIMIT_ON = 17,       // 使能软限位
    CTRL_OVER_TEMP_PROTECT_OFF = 18,   // 失能过温保护
    CTRL_OVER_TEMP_PROTECT_ON = 19,    // 使能过温保护
    CTRL_HEART_BEAT_PROTECT_OFF = 20,  // 失能心跳保护
    CTRL_HEART_BEAT_PROTECT_ON = 21,   // 使能心跳保护

} ControlWord_e;

// 电机控制模式
typedef enum {

    MOTOR_CTRL_MODE_NONE = 0,         // 无控制模式
    MOTOR_CTRL_MODE_CURRENT = 0x0A,      // 电流控制模式
    MOTOR_CTRL_MODE_VELOCITY = 0x09,     // 速度控制模式
    MOTOR_CTRL_MODE_POSITION = 0x08,     // 位置控制模式
    MOTOR_CTRL_MODE_PD = 0x0B,        // 转矩位置控制模式
    MOTOR_CTRL_MODE_BRAKE = 5,        // 制动模式
    MOTOR_CTRL_MODE_OPENLOOP = 0x0C,  // 开环控制模式

} MotorCtrlMode_e;

// // 电机状态码
// typedef enum {

//     IDLE_WM = 0,                  // 空闲状态
//     Init_WM = 1,                  // 初始化状态
//     Normal_WM = 2,                // 正常控制状态
//     Fault_WM = 3,                 // 异常状态
//     ENCODER_CAIL_WM = 4,          // 特殊状态 电机编码器校准
//     LINER_HALL_CAIL_WM = 5,       // 特殊状态 电机编码器校准
//     OUTPUT_ENCODER_CAIL_WM = 6,   // 特殊状态 出轴编码器校准
//     ACTUATOR_RETURN_ZERO_WM = 7,  // 特殊状态 执行器回零模式
//     FRICTION_IDENTIFY_WM = 8,     // 特殊状态 摩擦辨识模式

// } UserWorkMode_e;

//通信控制指令
typedef enum {
  
    ConfigNetWork = 0x600,        //网络通信参数设置
    SetId = 0x2001,               //设置电机ID
    ControlMode = 0x6060,         //控制模式
    ControlWord = 0x6040,         //控制字
    // WorkMode,                     //工作模式
    FastMode = 0x1800,            //快速上报模式
    ConfigIp = 0x1801,            //修改ip
    // Reboot,                       //软件重启                     
    ControlSetPos = 0x607A,       //指令位置
    ControlSetVel = 0x60FF,       //指令速度
    ControlSetTor =0x6071,        //指令力矩
    // ControlSetCur,                //指令电流
    ControlSetPosPID = 0x2020,    //指令位置环PID
    ControlSetVelPID = 0x2021,    //指令速度环PID
    // ControlSetPID,
    ControlSetPD = 0x2022,        //指令PD
    // GetMotorPVCT,                 //反馈PVCT
    ControlSetLimmit = 0x2011,     //设置电机限制
    NMT = 0x0100,                 //NMT状态机开启
    SaveParamToFlash = 0x200A,     //保存参数到flash
    GetENABLE_DRIVER = 0x200B,     //获取使能状态
    // Inverter,                     //反馈逆变器状态
    Heartbeat = 0x1017,           //设置电机心跳
    // SetDirction,                  //指令方向
    // SetOutputShaftRatio,          //指令减速比
    // SetPosThreshold,              //指令位置限制
    // SetVCTMax,                    //指令最大VCT
    // SetOutputOffset,              //指令出轴传感器位置偏置
    // SetOutEnable,                 //指令输出轴编码器使能
    // SetTorToCur,                  //指令转矩到电流多项式系数

    // AcBaseParams,                 //执行器基础参数集合

    // SendCalibrationData,	    // 发送校准数据到SDK
    // SetFocThetaSorce,			// 设置角度源
    // SetM1Voltage,				// 设置强拖电压

    OTA_CMD_HandShanking = 1563u,      // OTA 握手
    OTA_CMD_UpDateStart,       // OTA 开始下载
    OTA_CMD_FileTransmission,  // OTA 文件传输
    OTA_CMD_UpDateEnd,         // OTA 下载结束
    OTA_CMD_SendVersion,       // OTA 发送版本号
    Get_MotorID = 0x620,        // 获取电机
    DrvInfo = 0X2002,          //驱动器信息结构
    // SetMotor_Mode,			   // 设置电机类型  
}Command_Types;

// 设备信息结构体
typedef struct{

    char MainEditionIp[16]; //主机Ip
    uint16_t MainEditionPort; //主机端口
    uint32_t MainID; //主机ID
} DeviceInfo;

#endif
#ifndef __USER__PARAMETER_LIST__
#define __USER__PARAMETER_LIST__

#include "FocConfig.h"

// 执行器类型
typedef enum {
    PhyTypeNULL = 0,
    PhyArc47_S72V1_KV150 = 0x001,
    PhyArc68_S72V1_KV81 = 0x002,
    PhyArc78_S72V1_KV50 = 0x003,
    PhyArc95_S72V1_KV46 = 0x004,
    PhyArc102_S72V1_KV54 = 0x005,
    PhyArc150_S72V1_KV48 = 0x006,

    PhyArc102_S72V2_KV54 = 0x015,

    PhyArc47_G72V1_KV150 = 0x101,
    PhyArc68_G72V1_KV81 = 0x102,
    PhyArc78_G72V1_KV50 = 0x103,
    PhyArc95_G72V1_KV46 = 0x104,
    PhyArc102_G72V1_KV54 = 0x105,
    PhyArc150_G72V1_KV48 = 0x106,

    /***********钕磁环氮化镓**********/
    PhyArc47_G72V2_KV150 = 0x201,
    PhyArc68_G72V2_KV100 = 0x202,
    PhyArc78_G72V2_KV98 = 0x203,
    PhyArc95_G72V2_KV75 = 0x204,
    PhyArc102_G72V2_KV131 = 0x205,
    PhyArc150_G72V2_KV48 = 0x206,
    PhyArc150_G72V2_KV72 = 0x207,
	/***********钕磁环MOS**********/
    PhyArc47_S72V2_KV150 = 0x401,
    PhyArc68_S72V2_KV100 = 0x402,
    PhyArc78_S72V2_KV98 = 0x403,
    PhyArc95_S72V2_KV75 = 0x404,
    PhyArc102_S72V2_KV131 = 0x405,
    PhyArc150_S72V2_KV48 = 0x406,
    PhyArc150_S72V2_KV72 = 0x407,

    PhyArc150_G120V1_KV48 = 0x306,

    AnChi800W_G72_KV45 = 0xF001,
} ActuatorType_e;

typedef enum {
    MOTOR_NULL = 0,   //
    DYV1_95KV46 = 1,  //
    DYV1_150KV48,
    DYV1_102KV54,
    DYV1_68KV81,
    DYV1_78KV50,
    DYV1_47KV150,

    DYV2_47KVxx,
    DYV2_68KV100,
    DYV2_78KV98,
    DYV2_95KV75,
    DYV2_102KV131,
    DYV2_150KV72,

    ANCHI800W,
    MOTOR_ALL
} MotorType_e;

typedef enum {
    Hardware_NULL = 0,
    /*生产参数*/
    R47_S72_V1,
    R68_S72_V1,
    R78_S72_V1,
    R95_S72_V1,
    R150_S72_V1,
    R102_S72_V1,
    R47_G72_V1,
    R68_G72_V1,
    R95_G72_V1,
    R102_G72_V1,
    R150_G72_V1,
    /*测试参数*/
	R68_S72_V2,
    R102_S72_V2,
    R150_S72_V2,
	
    R47_G72_V2,
    R68_G72_V2,
    R102_G72_V2,
    R150_G72_V2,
    
    R150_G120_V1_T,
    WHEEL_G72_V1_T,
    Hardware_ALL,
} HardwareType_e;

typedef enum {
    SENSOR_NULL = 0,        // 表示无传感器或未知传感器类型
    SENSOR_TMR3110,         // 23位绝对值编码器
    SENSOR_MT6825,          // 23位绝对值编码器
    SENSOR_SWHALL,          // 霍尔+增量AB
    SENSOR_DOUBLE_TMR3110,  // 23位绝对值编码器
    SENSOR_ALL,             // 表示所有传感器类型的集合或总数
} SensorTypeList_e;

typedef enum {
    AnelgSensorNone = 0,
    Absolute_e = 1,    // 绝对值编码器
    Orthogonal_e = 2,  // 绝对值编码器
    SwHall_e = 3,
} AngleSensorType_e;

typedef struct
{
    int32_t FullValue;
    float DelayCompensation;
    uint8_t HallNumber;
    uint8_t PhaseCorrectionEnalbe;
} LinerHallParam_t;

typedef struct
{
    int32_t FullValue;
    float DelayCompensation;
    uint8_t NonlinearCalibrationEnalbe;
} MagEncoderParam_t;

/*接口反馈参数*/
typedef struct
{
    AngleSensorType_e Type;
    union
    {
        LinerHallParam_t LinerHall;
        MagEncoderParam_t MagEncoder;
    } Param;
} SensorParam_t;

typedef enum {
    B3950_10K = 0,
} NTC_RT_Type_e;

/* 执行器基础配置 */
typedef struct
{
	uint32_t MOTOR_ID;
    int32_t Dirction;              // 电机方向，注意取值为1或者-1，必须赋值,用于把出轴顺时针旋转为正方向
    float OutputShaftRatio;        // 输出轴减速比
    float MaxPos_Rad;              // 最大位置
    float MinPos_Rad;              // 最小位置
    float MaxVel_RadS;             // 最高速度
    float MaxCur_A;                // 最大电流
    float MaxTorque_NM;            // 力矩限幅
    float MaxAcc_RadSS;            // 每秒加速度
    Vct3_f32 PVCT_InterfaceLpfWc;  // 用户反馈数据低通滤波截止频率设定
} ActuatorBsaeParam_t;

/* 执行器传感器参数 */
typedef struct
{
    NTC_RT_Type_e CoilNTCType;              // NTC 温度电阻表类型
    NTC_RT_Type_e MosNTCType;               // MOS NTC 参数
    float NTC_R_GND;                        // GND分压电阻 KOhm
    uint32_t Coil2NTCEnable;                 // 第二绕组温度传感器使能
    //uint32_t NTC_Reserved[36];
}ActuatorSensorParam1_t;
    
typedef struct
{   
    SensorTypeList_e FocSensorType;          // FOC角度传感器类型
    SensorTypeList_e OutputShaftSensorType;  // 输出轴编码器类型
    uint32_t OutputShaftSensorIsReversal;    // 输出轴编码器反转标志
    float OutputShaftSensorOffset;           // 出轴零位偏置
    uint32_t OutputShaftSensorIsDiffGear;    // 出轴传感器是否为差齿
    uint32_t SyncGearToothNum;
    uint32_t DiffGearToothNum;
} ActuatorSensorParam2_t;

/*执行器控制参数 -- */
typedef struct
{
    uint32_t PosLoopDiv;       // 位置环分频
    float PosKp;               // 位置控制比例增益
//	float PosKi;               // 位置控制积分增益
    float PosKd;               // 位置控制微分增益
                               //
    uint32_t VelLoopDiv;       // 速度环分频
    float VelKp;               // 速度控制比例增益
    float VelKi;               // 速度控制积分增益
//	float VelKd;               // 速度控制微分增益
    float VelFdbkLpfWc;        // 速度反馈低通滤波截止频率
                               //
    uint32_t CurLoopDiv;       // 电流环分频
    float CurLoopWc;           // 电流环带宽
    float CurLoopLowFreqGain;  // 电流环低频增益
                               //
    uint32_t PDLoopDiv;        // PD环分频
    float PDKp;                // PD控制比例增益
    float PDKd;                // PD控制微分增益
} ActuatorControlParam_t;

/* 执行器功能配置 */
typedef struct
{
    FOCCtrlMode_e CtrlMode;         // 控制模式
    ThetaSorce_e FocThetaSorce;     // FOC角度源选择
    uint32_t FrictionCompEnable;    // 摩擦补偿使能
    uint32_t TorqueLoadCompEnable;  // 负载转矩补偿
    uint32_t CoggingCopmEnable;     // 齿槽转矩补偿使能
    uint32_t OutShaftSensorEnable;  // 输出轴编码器使能
} ActuatorFuncParam_t;

// 安全保护模块
typedef struct
{
    float Iq_OverCur;                 // Iq保护电流
    float Iq_OverCur_S;               // Iq保护电流时间 秒
    float ChipOverTemp;               // 芯片过温保护阈值
    float MosOverTemp;                // Mos过温保护阈值
    float CoilOverTemp;               // 绕组过温保护阈值
    float OverTemp_S;                 // 过温保护时间 秒
    float MosTempWarn;                // Mos温度警告阈值
    float CoilTempWarn;               // 绕组温度警告阈值
    float PosMax_Rad;                 // 位置限制最大值
    float PosMin_Rad;                 // 位置限制最小值
    float MosOverTempRecLimit_mS;     // Mos过温恢复时间限制
    float CoilOverTempRecLimit_mS;    // 绕组过温恢复时间限制
    float LossPhaseCheckMinVel_RadS;  //
} ErrDectParam_t;


typedef struct
{
    ActuatorType_e ActuatorType;
    char ActuatorName[64];

    ActuatorBsaeParam_t ActuatorBsaeParams;  // 执行器基础配置
    uint32_t Reserved_0[63];

    ActuatorSensorParam1_t ActuatorSensorParams1;  // 执行器传感器配置
    ActuatorSensorParam2_t ActuatorSensorParams2;  // 执行器传感器配置
    uint32_t Reserved_1[15];

    ActuatorControlParam_t ActuatorControlParams;  // 执行器控制参数
    uint32_t Reserved_2[16];

    ActuatorFuncParam_t ActuatorFuncParams;  // 执行器功能配置
    uint32_t Reserved_3[16];

    MotorParam_t MotorParams;
    uint32_t Reserved_4[16];

    HardwareParam_t HardwareParams;
    uint32_t Reserved_5[13];

    ForceParam_t ForcelParams;
    uint32_t Reserved_6[16];

    FulxObsParam_t FulxObsParams;
    uint32_t Reserved_7[15];

    TorqueCalibParam_t TorqueCalibParams;
    uint32_t Reserved_8[16];

    FrictionParam_t FrictionParams;
    uint32_t Reserved_9[16];

    TempCurParam_t TempCurrParams;
    uint32_t Reserved_10[16];

    ErrDectParam_t ErrDectParams;
    uint32_t Reserved_11[18];
} ParamList_t;



#endif
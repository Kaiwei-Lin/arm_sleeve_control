/*
 * current_control_get_pvct.c
 * 功能：对电机设置电流模式，使能电机，设置电机电流值，循环获取实际快速帧pvctfast
 * @author  xufuliang
 * @date    2026-03-13
 */
#include <stdio.h>
#include "RobotControl.h"
#include "Common.h"
#include <time.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef _WIN32
#include <windows.h>

#define SLEEP_POS 2000000   //2000*1000 为2000微妙=2ms 500hz
/* 返回自系统启动以来的纳秒数（单调递增） */
static inline uint64_t  nanos_now(void)
{
    static LARGE_INTEGER freq = { 0 };
    if (freq.QuadPart == 0)           /* 只需第一次调用时初始化 */
        QueryPerformanceFrequency(&freq);

    LARGE_INTEGER cnt;
    QueryPerformanceCounter(&cnt);

    /* 计数器差值 ÷ 频率 × 1e9  => 纳秒 */
    return (uint64_t)(cnt.QuadPart * 1000000000ULL / freq.QuadPart);
}



/* 睡眠指定的纳秒数（busy-wait 最后几百微秒） */
void sleep_ns(unsigned ns) {
    const uint64_t t0 = nanos_now();
    const uint64_t t_end = t0 + ns;


    /* 1. 如果剩余时间 > 1 ms，用 Sleep 先“粗睡” */
    if (ns >= 1 * 1000000ULL) {
        DWORD ms = (DWORD)((t_end - nanos_now()) / 1000000ULL);
        if (ms) Sleep(ms - 1);      /* 留 1 ms 给下面的忙等 */
    }

    /* 2. 忙等直到准确时刻 */
    while (nanos_now() < t_end)
        YieldProcessor();           /* VS 也可 _mm_pause(); */
}


void format_ns(uint64_t ns, char* buf, size_t len)
{
    uint64_t h = ns / 3600000000000ULL;
    ns %= 3600000000000ULL;
    uint64_t m = ns / 60000000000ULL;
    ns %= 60000000000ULL;
    uint64_t s = ns / 1000000000ULL;
    ns %= 1000000000ULL;
    snprintf(buf, len,
        "%02llu:%02llu:%02llu.%09llu",
        h, m, s, ns);
}
#else
#include <sys/time.h>
#define SLEEP_POS 2000000   //2000*1000 Ϊ2000΢��=2ms 500hz
static inline uint64_t nanos_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

static inline void sleep_ns(uint64_t ns)
{
    struct timespec req = { ns / 1000000000ULL, ns % 1000000000ULL };
    nanosleep(&req, NULL);
}

static  uint64_t ts2ns(const struct timespec* ts)
{

    return (uint64_t)ts->tv_sec*1000000000ULL + ts->tv_nsec;
}

#endif 

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
        return -1;
    }


    
    //设置电机上传频率
    value = robot_set_fast_mode(ctx, 500);

    if (value)
    {

            printf("设置主板上传频率成功\n");
           
    }
    else
    {
            printf("设置主板上传频率失败\n");
            return -1;
    }




    //生成电机对象
    RobotMotor* motor = robot_create_motor(ctx, 7, 1);


    //关闭MIT使能状态
    robot_StateMachine(ctx,0x80);

    //设置MIT使能状态
    robot_StateMachine(ctx,1);
    while (1)
    {
        
        //关闭电机心跳
        value = robot_motor_setHeartbeat(motor, 0);

        if (value)
        {

            printf("关闭电机心跳成功\n");
            break;
        }
        else
        {
            printf("关闭电机心跳失败\n");
        }

       

    }

   

    while (1) 
    {
        //设置电机电流模式
        value = robot_motor_set_control_mode(motor, MOTOR_CTRL_MODE_CURRENT);

        if (value)
        {

            printf("设置电机电流模式成功\n");
            break;
        }
        else
        {
            printf("设置电机电流模式失败\n");
        }
    }

      //为更新电机参数设置pos
    robot_motor_set_pos(motor, 0, 0, 0);

    for (int i = 0; i < 45; i++)
    {
    #ifdef _WIN32
        char str[32];
        format_ns(nanos_now(), str, sizeof(str));
        puts(str);
        uint64_t t0 = nanos_now();
        robot_motor_set_big_pose(ctx);
        uint64_t t1 = nanos_now();
        //printf("CPU: %llu ns\n", (uint64_t)(t1 - t0));
        //printf("shijiancha:%llu ns\n", SLEEP_POS - ((uint64_t)(t1 - t0)));
        sleep_ns(SLEEP_POS - ((uint64_t)(t1 - t0)));

    #else
        struct timespec t1;
        struct timespec t2;
        clock_gettime(CLOCK_REALTIME, &t1);
        robot_motor_set_big_pose(ctx);
        clock_gettime(CLOCK_REALTIME, &t2);
        double total = (double)(ts2ns(&t2) - ts2ns(&t1)) / CLOCKS_PER_SEC;
        printf("CPU:%g\n", total);
        
        sleep_ns((SLEEP_POS-total));
    #endif 
    }
    

   

    while (1) 
    {
        //设置电机使能

        value = robot_motor_set_control_world(motor, CTRL_SERVO_ON);

        if (value)
        {

            printf("设置电机使能 成功\n");
            break;
        }
        else
        {
            printf("设置电机使能 失败\n");
        }
    }
   


    //睡眠1s
#ifdef _WIN32
    Sleep(1000);
#else
    sleep(1);
#endif






    while (1) 
    {
        uint64_t t0 = nanos_now();
       

        float Pos = 0.0, Vel = 0.0,  Tor_e = 0.0;
        uint32_t FastStateMechine = 0,FastMosTemperature = 0, FastWindingTemperature = 0,
        FastBusVoltage = 0,  FastErrorCode = 0;
        //获取电机pvct值
        robot_motor_get_PVCTFast(motor, &Pos, &Vel, &Tor_e,&FastStateMechine,
        &FastMosTemperature, &FastWindingTemperature,
        &FastBusVoltage, &FastErrorCode);
      
        //设置电流
        robot_motor_set_cur(motor,1);
           
       
        //发送数据
        robot_motor_set_big_pose(ctx);

        uint64_t t1 = nanos_now();
        if (((uint64_t)(t1 - t0)) > SLEEP_POS)
        {
            printf("((uint64_t)(t1 - t0)) >  SLEEP_POS\n");
            continue;
        }
        else
        {
            sleep_ns(SLEEP_POS - ((uint64_t)(t1 - t0)));
        }
    }
  
  



    //释放电机对象
    robot_destroy_motor(ctx,motor);
    //释放主板对象
    robot_destroy(ctx);

    return 0;

}
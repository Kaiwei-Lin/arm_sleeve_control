/*
 * set_control_world.c
 * 功能： 设置电机校准
 * @author  xufuliang
 * @date    2025-08-27
 */
#include <stdio.h>
#include "RobotControl.h"
#include "Common.h"
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#endif 

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
	}
 /*   int id = robot_motor_get_motor_id(ctx);
    printf("获取电机ID:%d\n", id);*/
    //生成电机对象
    RobotMotor* motor = robot_create_motor(ctx, 13, 1);
    //设置电机执行器校准
    int return_value = robot_motor_set_control_world(motor, CTRL_CALIBRATE_ENCODER);
    if (return_value)
    {
        printf("执行器校准 成功\n");
    }
    else
    {

        printf("执行器校准 失败\n");
        return -1;
    }

    //睡眠1s
#ifdef _WIN32
    Sleep(2000);
#else
    sleep(2);
#endif



    unsigned int encodervalue;
    unsigned int codevalue;
    unsigned int world;
    for(;;)
    {
        robot_motor_get_EncoderValue(motor,&encodervalue);
        printf("编码器值:%d\n",encodervalue);  
        sleep_ns(SLEEP_POS);

       
        
        robot_motor_get_EnableValue(motor,&world);
        printf("获取使能状态%d\n",world);
        
        if(world == 0)
        {
            printf("校准结束\n");
            break;
        }
    }
    
    
   
    //释放电机对象
    robot_destroy_motor(ctx,motor);
#ifdef _WIN32
    Sleep(30000);
#else
    sleep(30);
#endif

    motor = robot_create_motor(ctx, 13, 1);
    //设置电机执行器校准
    return_value = robot_motor_set_control_world(motor, CTRL_CALIBRATE_ENCODER);
    if (return_value)
    {
        printf("执行器校准 成功\n");
    }
    else
    {

        printf("执行器校准 失败\n");
    }


    //释放电机对象
    robot_destroy_motor(ctx,motor);
    //释放主板对象
    robot_destroy(ctx);

    return 0;

}
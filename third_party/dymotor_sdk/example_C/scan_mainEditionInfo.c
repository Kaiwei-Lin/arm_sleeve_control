#include <stdio.h>
#include "RobotControl.h"
#include "Common.h"
int main(int argc, char* argv[]) {

    DeviceInfo deviceinfos[3];
    int num = 0;
    get_robot_mainEditioninfos(deviceinfos,&num);
    int i = 0;
    for(;i<num;i++)
    {
        printf("ip:%s,port:%d,Id:%x\n",deviceinfos[i].MainEditionIp,deviceinfos[i].MainEditionPort,deviceinfos[i].MainID);
    }

    return 0;
}
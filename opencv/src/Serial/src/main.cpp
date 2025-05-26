#include <ros/ros.h>
#include <string>
#include "./Serial/Serial.h"
int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc,argv,"HTS20L");

    //创建 ros 节点句柄(非必须)
    ros::NodeHandle n;

    
    while(1)
    {
        
    }

    return 0;
}
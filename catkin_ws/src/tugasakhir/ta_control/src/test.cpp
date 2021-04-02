#include <iostream>
#include <ros/ros.h>

#include "ta_control/test.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control");

    printf("%s\n", HELLO_WORLD);
    printf("Press Enter to exit..");
    
    getchar();

    return 0;
}

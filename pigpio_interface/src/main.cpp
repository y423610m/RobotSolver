

#include <iostream>
#include <memory>
using namespace std;

#include "ros/ros.h"

#include "pigpio_manager.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "pigpio_node");
    ros::NodeHandle nh;

    unique_ptr<PigpioManager> pigpio_interface_(new PigpioManager());

    ros::Rate loop_rate(128);
    while(ros::ok()&&pigpio_interface_->checkLoop()){
        pigpio_interface_->update();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


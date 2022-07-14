#include <iostream>
#include <memory>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "robot_manager.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "solver");
    ros::NodeHandle nh;

    unique_ptr<RobotManager> robot_manager_(new RobotManager());

    ros::Rate loop_rate(100);
    while(ros::ok()){
        robot_manager_->update();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


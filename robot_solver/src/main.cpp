git clone -b main_ros https://github/y423610m/RobotSolver.git
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

#include <iostream>
#include <memory>
using namespace std;

#include "ros/ros.h"

#include "robot_manager.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "solver");
    ros::NodeHandle nh;

    unique_ptr<RobotManager> robot_manager_(new RobotManager());

    ros::Rate loop_rate(128);
    while(ros::ok()&&robot_manager_->checkLoop()){
        robot_manager_->update();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


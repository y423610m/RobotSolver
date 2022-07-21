// git clone -b main_ros https://github/y423610m/RobotSolver.git

#include <iostream>
#include <memory>
using namespace std;

#include "ros/ros.h"

#include "robot_manager.h"
#include "development_commands.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "solver_node");
    ros::NodeHandle nh("~");


    // XmlRpc::XmlRpcValue DHs;
    // nh.getParam("DHParams", DHs);
    // ROS_WARN("%i", (int)DHs.size());
    // for(int i=0;i<(int)DHs.size();i++){
    //     ROS_WARN("%lf", double(DHs[i]["a"]));
    //     ROS_WARN("%lf", double(DHs[i]["alp"]));
    // }

    int RobotType = -1;
    nh.getParam(ros::this_node::getNamespace()+ros::this_node::getName()+"RobotType", RobotType);
    //if(RobotType<=0) return 0;
    RobotType = 2;
    EL(RobotType)

    unique_ptr<RobotManager> robot_manager_(new RobotManager(RobotType));

    ros::Rate loop_rate(128);
    while(ros::ok()&&robot_manager_->checkLoop()){
        robot_manager_->update();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


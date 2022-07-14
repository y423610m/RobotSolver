#include "ros_interface.h"


ROSInterface::ROSInterface(){
    ros::NodeHandle nh;
    jointAnglesPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("/topicName", 1000);
}


ROSInterface::~ROSInterface(){

}

void ROSInterface::publishJointAngles(const vector<double>& jointAngles){
    //size
    if(jointAngles_.data.size()!=jointAngles.size()) jointAngles_.data.resize(jointAngles.size());

    //set
    for(int i=0;i<jointAngles.size();i++) jointAngles_.data[i] = jointAngles[i];

    //pub
    jointAnglesPublisher_.publish(jointAngles_);

}

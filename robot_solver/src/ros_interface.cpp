#include "ros_interface.h"


ROSInterface::ROSInterface(){
    ros::NodeHandle nh;
    jointAnglesPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("/topicName1", 1000);
    jointAnglesSubscriber_ = nh.subscribe("/topicName2", 3, &ROSInterface::_getJointAnglesCB, this);
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


// void ROSInterface::_getJointAnglesCB(const sensor_msgs::JointState& JointState){
void ROSInterface::_getJointAnglesCB(const sensor_msgs::JointState& jointState){
    if(jointPosition_.size()!=JointState.position.size()) jointPosition_.resize(JointState.position.size());
    for(int i=0;i<(int)JointState.position.size();i++) jointPosition_[i] = JointState.position[i]; 
}

vector<double> ROSInterface::getActualJointPosition(){
    while(jointPosition_.size()==0){/*一回取得するまで待機*/}
    if(jointAnglesArray.size()!=jointPosition_.size()) jointAngleArray.resize(jointPosition_);
    for(int i=0;i<(int)jointPosition_.size();i++) jointAnglesArray[i] = jointPosition_[i];
    return jointPosition_;
}

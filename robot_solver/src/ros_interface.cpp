#include "ros_interface.h"
#include "robot_type_definitions.h"

ROSInterface::ROSInterface(int RobotType)
:RobotType_(RobotType)
{
    if(RobotType==RobotType_CobottaWithoutTool) this->_initCobottaWithoutTool();
}


ROSInterface::~ROSInterface(){

}

void ROSInterface::_initCobottaWithoutTool(){
    ros::NodeHandle nh;
    targetJointAnglesPublisher_ = nh.advertise<std_msgs::Float64MultiArray>("targetJointAngles", 2);
    actualJointAnglesSubscriber_ = nh.subscribe("actualJointAngles", 2, &ROSInterface::_getActualJointAnglesCB, this);
    targetTipPoseSubscriber_ = nh.subscribe("targetTipPose", 2, &ROSInterface::_getTargetTipPoseCB, this);
    targetTipPose_.resize(7);
}
void ROSInterface::publishCobottaWithoutTool(const vector<double>& targetJointAngles){
    // if(jointAngles_.data.size()!=jointAngles.size()) jointAngles_.data.resize(jointAngles.size());
    targetJointAngles_.data = targetJointAngles;
    targetJointAnglesPublisher_.publish(targetJointAngles_);
}




void ROSInterface::_getActualJointAnglesCB(const sensor_msgs::JointState& actualJointState){
    actualJointAngles_ = actualJointState.position;
}
vector<double>& ROSInterface::getActualJointPosition(){
    //actualJointAngles_ = vector<double>(6, 0.5);
    return actualJointAngles_;
}




void ROSInterface::_getTargetTipPoseCB(const geometry_msgs::Pose& targetTipPose){
    targetTipPose_[0] = targetTipPose.position.x;
    targetTipPose_[1] = targetTipPose.position.y;
    targetTipPose_[2] = targetTipPose.position.z;
    targetTipPose_[3] = targetTipPose.orientation.w;
    targetTipPose_[4] = targetTipPose.orientation.x;
    targetTipPose_[5] = targetTipPose.orientation.y;
    targetTipPose_[6] = targetTipPose.orientation.z;
}
vector<double>& ROSInterface::getTargetTipPose(){
    return targetTipPose_;
}

















// int ROSInterface::getParamInt(string ParamName){
//     int retVal=1;
//     nh_.getParam(ParamName, retVal);
//     return retVal;
// }

// template<typename T>
// T ROSInterface::getParam(string ParamName){
//     T retVal;
//     nh_.getParam(ParamName, retVal);
//     return retVal;    
// }

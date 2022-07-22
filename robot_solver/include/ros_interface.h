#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"

#include<vector>
using namespace std;

class ROSInterface{
private:
    int RobotType_ = -1;
    ros::NodeHandle nh_;

    ros::Publisher targetJointAnglesPublisher_;
    std_msgs::Float64MultiArray targetJointAngles_;

    ros::Subscriber actualJointAnglesSubscriber_;
    vector<double> actualJointAngles_;
    void _getActualJointAnglesCB(const sensor_msgs::JointState& actualJointState);

    ros::Subscriber targetTipPoseSubscriber_;
    vector<double> targetTipPose_;
    void _getTargetTipPoseCB(const geometry_msgs::Pose& targetTipPose);

    // ros::Subscriber clutchStateSubscriber_;
    // int clutchState_;

    void _initCobottaWithTool();
    void _initCobottaWithoutTool();
    void _init6DOFArm();

public:
    ROSInterface(int RobotType);
    ~ROSInterface();

    void publishCobottaWithTool(const vector<double>& jointAngles, double toolAngle);
    void publishCobottaWithoutTool(const vector<double>& jointAngles);
    void publish6DOFArm(const vector<double>& jointAngles, double toolAngle);

    vector<double>& getActualJointPosition();
    vector<double>& getTargetTipPose();


    // int getParamInt(string ParamName);
    // template<typename T> T getParam(string ParamName);

};
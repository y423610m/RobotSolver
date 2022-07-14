// #include "/opt/ros/noetic/include/ros/ros.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include<vector>
using namespace std;

class ROSInterface{
private:
    ros::NodeHandle nh_;
    ros::Publisher jointAnglesPublisher_;
    std_msgs::Float64MultiArray jointAngles_;
    ros::Subscriber jointAnglesSubscriber_;
    vector<double> jointPosition_;

    void _getJointAnglesCB(const sensor_msgs::JointState& jointState);

public:
    ROSInterface();
    ~ROSInterface();
    void publishJointAngles(const vector<double>& jointAngles);
    vector<double> getActualJointPosition();

};
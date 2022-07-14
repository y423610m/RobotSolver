#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include<vector>
using namespace std;

class ROSInterface{
private:
    ros::NodeHandle nh_;
    ros::Publisher jointAnglesPublisher_;
    std_msgs::Float64MultiArray jointAngles_;
public:
    ROSInterface();
    ~ROSInterface();
    void publishJointAngles(const vector<double>& jointAngles);

};
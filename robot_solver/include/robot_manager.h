#include <iostream>
#include <vector>
#include <memory>
using namespace std;

#include "robot_solver.h"
#include "gpio_interface.h"
#include "ros_interface.h"


class RobotManager{
private:
    bool initialized_ = false;
    unique_ptr<RobotSolver> solver_;
    unique_ptr<GPIOInterface> gpio_;
    unique_ptr<ROSInterface> ros_interface_;

    int nJoint_ = -1;
    vector<double> minAngles_;
    vector<double> maxAngles_;

    vector<double> currentJointAngles_;
    vector<double> currentTipPose_;
    vector<double> targetJointAngles_;
    vector<double> targetTipPose_;
    vector<double> commandJointAngles_;

    int loopCnt_ = 0;
    int IKCnt_ = 0;
    int IKPeriod_ = 128;



public:
    RobotManager();
    ~RobotManager();
    void update();
    bool checkLoop();
};
#include <iostream>
#include <vector>
#include <memory>
using namespace std;

#include "robot_solver.h"
#include "ros_interface.h"


class RobotManager{
private:
    bool initialized_ = false;
    int RobotType_ = -1;
    unique_ptr<ROSInterface> ros_interface_;
    unique_ptr<RobotSolver> solver_;

    vector<double> currentJointAngles_;
    vector<double> currentTipPose_;
    vector<double> targetJointAngles_;
    vector<double> targetTipPose_;
    vector<double> commandJointAngles_;

    int loopCnt_ = 0;
    int IKCnt_ = 0;
    int IKPeriod_ = 128;

    bool _updateCobottaWithTool();
    bool _updateCobottaWithoutTool();
    bool _update6DOFArm();

public:
    RobotManager(int RobotType);
    ~RobotManager();
    void update();
    bool checkLoop();
};
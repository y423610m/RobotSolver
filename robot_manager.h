#include <iostream>
#include <vector>
#include <memory>
using namespace std;

#include "robot_solver.h"
#include "gpio_interface.h"


class RobotManager{
private:
    bool initialized_ = false;
    unique_ptr<RobotSolver> solver_;
    unique_ptr<GPIOInterface> gpio_;

    int nJoint_ = -1;
    vector<double> minAngles_;
    vector<double> maxAngles_;

    int cnt_ = 0;

    //test
    vector<double> tipPose;

public:
    RobotManager();
    ~RobotManager();
    void update();
    bool checkLoop();
};
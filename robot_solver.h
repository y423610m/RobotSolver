#include <iostream>
#include <vector>
using namespace std;

//#include <eigen3/Dense>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

struct DHParam{
    double a, alp, d, tht;
};

class RobotSolver{
private:
    bool initialized_ = false;
    int nJoint_ = -1;
    vector<double> minAngles_;
    vector<double> maxAngles_;
    vector<double> currentAngles_;
    vector<DHParam> DHs;

    // may no be required
    vector<Matrix4d> Ts;

    void _initSpecificParamsTemplate();
    void _initSpecificParams6dArm();
    void _initSpecificParamsCobotta();
    void _initCommon();

public:
    RobotSolver();
    ~RobotSolver();

    vector<double> FK(const vector<double>& jointAngles);
    vector<double> moveTo(const vector<double>& targetPose);
    vector<double> numericIK(const vector<double>& targetPose);
    int getNJoint(){return nJoint_;}
    vector<double> getMinAngles(){ return minAngles_; }
    vector<double> getMaxAngles(){ return maxAngles_; }


};
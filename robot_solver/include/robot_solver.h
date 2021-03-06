#pragma once

#include <iostream>
#include <vector>
using namespace std;

//#include <eigen3/Dense>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "dh_parameters.h"
 
/*
    length is always [m] !!!!! (not [mm])
    Except init specific fuunctions, any angle is radian!!!!!
*/


class RobotSolver{
private:
    bool initialized_ = false;
    //robot params
    int nJoint_ = -1;
    vector<double> minAngles_;
    vector<double> maxAngles_;
    vector<double> currentJointAngles_;
    vector<double> targetJointAngles_;
    vector<DHParam> DHs_;
    vector<double> basePose_;
    Matrix4d Tbase_;
    vector<double> tipPose_;
    Matrix4d Ttip_;
    vector<Matrix4d> Ti_;


    //init functions
    void _initSpecificParamsTemplate();
    void _initSpecificParams6dArm();
    void _initSpecificParamsCobottaWithoutTool();
    void _initSpecificParamsCobottaWithTool();
    void _initCommon();

    //for IK
    double h = 0.0001;
    double eps = 1e-7;
    Matrix<double, 6, Dynamic> J_;
    Matrix<double, Dynamic, 1> dq_;
    vector<Matrix4d> Tfront_; // base -> joint[i]
    vector<Matrix4d> Tback_;  // joint[i] -> base    void _calculateJ();
    void _calculateJ();


    //PIDControll
    //Gd/2 = sqrt(Gp)
    // double Gp_ = 0.00025;
    // double Gd_ = 0.03;
    double Gp_ = 0.00035;
    double Gd_ = 0.08;
    vector<double> jointVelocity_;
    vector<double> jointMaxVelocity_;
    vector<double> jointMaxAccel_;


public:
    RobotSolver(int RobotType = 1);
    ~RobotSolver();

    vector<double> FK(const vector<double>& jointAngles);
    vector<double> numericIK(const vector<double>& targetPose, int maxLoop=30);
    vector<double> getCommandJointAngles();


    int getNJoint(){return nJoint_;}
    vector<double> getMinAngles(){ return minAngles_; }
    vector<double> getMaxAngles(){ return maxAngles_; }
    vector<double> getCurrentAngles(){ return currentJointAngles_; }
    void setCurrentAngles(const vector<double> currentAngles);
    void setTargetAngles(const vector<double> targetAngles);

};
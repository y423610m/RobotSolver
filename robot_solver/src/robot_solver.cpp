#include "robot_solver.h"
#include "development_commands.h"
#include "cvt_convert_functions.h"
#include "robot_type_definitions.h"

#include <cassert>
#include <cmath>
#include <algorithm>




RobotSolver::RobotSolver(int RobotType){
    if(RobotType==RobotType_CobottaWithTool) this->_initSpecificParamsCobottaWithTool();
    if(RobotType==RobotType_CobottaWithoutTool) this->_initSpecificParamsCobottaWithoutTool();
    if(RobotType==RobotType_6DOFArm) this->_initSpecificParams6dArm();
    this->_initCommon();
    initialized_ = true;
    cerr<<"RobotSolver constructed"<<endl;
}

RobotSolver::~RobotSolver(){
    cerr<<"RobotSolver desstructed"<<endl;
}

// manually set robot parameters depending on your robot
// this is template therefore create copy of this function 
void RobotSolver::_initSpecificParamsTemplate(){
    nJoint_ = 6;
    //moters' Range [deg]H
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({0., 0., 0., 0.}); //base=>joint[0]
    DHs_.push_back({0., 0., 0., 0.}); //joint[0]->joint[1]
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.}); //joint[5]->armTip

}

void RobotSolver::_initSpecificParams6dArm(){
    nJoint_ = 6;
    //moters' Range [deg]
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);


    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({0., 0., 0., 0.}); //base=>joint[0]
    DHs_.push_back({0., 0., 0., 0.}); //joint[0]->joint[1]
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.}); //joint[5]->armTip

    cerr<<"init for 6d arm"<<endl;
}

void RobotSolver::_initSpecificParamsCobottaWithoutTool(){
    nJoint_ = 6;
    //moters' Range [deg]
    minAngles_ = vector<double>{-150., -60.,   18., -170., -95., -170.,};
    maxAngles_ = vector<double>{ 150., 100.,  140.,  170., 135.,  170.};


    //global origin to base. dx,dy,dz,dax,day,daz [m, deg]
    // basePose_ = {-0.25, -0.23, 0., 0., 0., 90.};
    basePose_ = {0., 0., 0., 0., 0., 90.};

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({   0.,   0.,    0.18,   0.}); //base=>joint[0]
    DHs_.push_back({   0., 270.,      0., 270.}); //joint[0]->joint[1]
    DHs_.push_back({0.165,   0.,   -0.02, 270.}); ////////////////////////last 0->90!!!
    DHs_.push_back({0.012, 270.,  0.1775,   0.}); //dummy3->dummy4
    DHs_.push_back({   0.,  90., -0.0645,   0.});
    DHs_.push_back({   0., 270., 0.0385+0,  0.});
    DHs_.push_back({0.012, 270.,   0.175,   0.}); //joint[6]->armTi

    tipPose_ = {0., 0., 0., 90., 0., 0.};

    jointMaxAccel_ = vector<double>(nJoint_, 5e-5);
    jointMaxVelocity_ = vector<double>(nJoint_, 1e-3);

    Gp_ = 0.00035;
    Gd_ = 0.08;

    cerr<<"init for cobotta without tip tool"<<endl;
}

void RobotSolver::_initSpecificParamsCobottaWithTool(){
    nJoint_ = 7;
    //moters' Range [deg]
    minAngles_ = vector<double>{-150., -60.,   18., -170., -95., -170., -360.};
    maxAngles_ = vector<double>{ 150., 100.,  140.,  170., 135.,  170.,  360.};

    //global origin to base. dx,dy,dz,dax,day,daz [m, deg]
    basePose_ = {-0.25, -0.23, 0., 0., 0., 90.};

    //set DH Parameters {a, alp, d, tht} [m, deg]
    DHs_.push_back({   0.,   0.,    0.18,   0.}); //base=>joint[0]
    DHs_.push_back({   0., 270.,      0., 270.}); //joint[0]->joint[1]
    DHs_.push_back({0.165,   0.,   -0.02, 270.}); ////////////////////////last 0->90!!!
    DHs_.push_back({0.012, 270.,  0.1775,   0.}); //dummy3->dummy4
    DHs_.push_back({   0.,  90., -0.0645,   0.});
    DHs_.push_back({   0., 270., 0.0385+0,  0.});
    DHs_.push_back({   0., 270.,    0.09,   0.}); //joint[6]->toolJoint
    DHs_.push_back({0.012,   0.,   0.085,   0.}); //toolJolint->armTip

    jointMaxAccel_ = vector<double>(nJoint_, 5e-5);
    jointMaxVelocity_ = vector<double>(nJoint_, 1e-3);

    Gp_ = 0.00035;
    Gd_ = 0.08;

    cerr<<"init for cobotta including tip tool"<<endl;
}

// applied for any robot
void RobotSolver::_initCommon(){

    //check parameters
    if(nJoint_<=0){ PS("robotsolver") PL("nJoint_ not set") EL(nJoint_)}
    if((int)minAngles_.size()!=nJoint_){
        PS("robotsolver") PS("minAngles not valid") PL("set nJoint*0.0")
        minAngles_ = vector<double>(nJoint_, 0.);
    }
    if((int)maxAngles_.size()!=nJoint_){
        PS("robotsolver") PS("maxAngles not valid") PL("set nJoint*180deg")
        maxAngles_ = vector<double>(nJoint_, 180.);
    }
    if((int)DHs_.size() != nJoint_+1){PS("robotsolver") PL("DHParams not valid") EL(DHs_.size())}
    if((int)basePose_.size()!=6){
        PS("robotsolver") PS("basePose_ not set") PL("set all 0.0")
        basePose_ = vector<double>(6, 0.0);
    }
    if((int)jointMaxAccel_.size()!=nJoint_){
        PS("robotsolver") PS("jointMaxAccel_ not set") PL("set nJint*1e-5")
        jointMaxAccel_.resize(nJoint_, 1e-5);
    }
    if((int)jointMaxVelocity_.size()!=nJoint_){
        PS("robotsolver") PS("jointMaxVelocity_ not set") PL("set nJint*1e-3")
        jointMaxVelocity_.resize(nJoint_, 1e-3);
    }

    // deg -> rad
    for(int i=0;i<nJoint_;i++) minAngles_[i] *= M_PI/180.;
    for(int i=0;i<nJoint_;i++) maxAngles_[i] *= M_PI/180.;
    for(int i=3;i<6;i++) basePose_[i] *= M_PI/180.;
    for(int i=0;i<(int)DHs_.size();i++) DHs_[i].alp *= M_PI/180.;
    for(int i=0;i<(int)DHs_.size();i++) DHs_[i].tht *= M_PI/180.;

    Eigen::Affine3d aff = Eigen::Translation<double,3>(basePose_[0], basePose_[1], basePose_[2])
                * AngleAxisd(basePose_[3], Vector3d::UnitX())
                * AngleAxisd(basePose_[4], Vector3d::UnitY())
                * AngleAxisd(basePose_[5], Vector3d::UnitZ());
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) Tbase_(i,j) = aff(i,j);

    aff = Eigen::Translation<double,3>(tipPose_[0], tipPose_[1], tipPose_[2])
                * AngleAxisd(tipPose_[3], Vector3d::UnitX())
                * AngleAxisd(tipPose_[4], Vector3d::UnitY())
                * AngleAxisd(tipPose_[5], Vector3d::UnitZ());
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) Ttip_(i,j) = aff(i,j);
 
    Ti_ = vector<Matrix4d>(nJoint_+1);
    Ti_[0] = Tbase_ * cvt::toMat44FromDH(DHs_[0]);
    for(int i=0;i<nJoint_;i++) Ti_[i+1] = cvt::toMat44FromDH(DHs_[i+1]);
    Ti_.back() *= Ttip_;

    currentJointAngles_.resize(nJoint_, 0.);
    targetJointAngles_ .resize(nJoint_, 0.);
    jointVelocity_ .resize(nJoint_, 0.);


    //for IK calc
    J_.resize(6, nJoint_);
    dq_.resize(nJoint_, 1);
    Tfront_.resize(nJoint_+1); // base -> joint[i]
    Tback_.resize(nJoint_+1);  // joint[i] -> base


    PS("robotsolver") PL("initCommon done")
}


// jointAngles.size() == nJoint [rad]
// return size == always 6(xyz,eulerxyz) [m,rad]
vector<double> RobotSolver::FK(const vector<double>& jointAngles){
    //Tfrom base to Tip
    Matrix4d Tb2t = Ti_[0];
    // for(int i=0;i<nJoint_;i++) Tb2t *= cvt::toMat44RotZ(jointAngles[i])*Ti_[i+1];
    //PL(0) PL(Tb2t)
    for(int i=0;i<nJoint_;i++){
        Tb2t *= cvt::toMat44RTFromDH(jointAngles[i], DHs_[i+1]);
        //PL(i+1) PL(Tb2t)
    }
    Tb2t *= Ttip_;
    vector<double> tipPose = cvt::toVecXYZEuler(Tb2t);
    return tipPose;
}

void RobotSolver::_calculateJ(){
    /*
        X_i+h = Tfront[i] * rotZ(angle+h) * Tback[i]
        X_i   = Tfront[i] * rotZ(angle)   * Tback[i]
    */
    Tfront_[0] = Ti_[0];
    Tback_.back() = Ti_.back();
    //T = T*RT
    // for(int i=0;i<nJoint_;i++) Tfront_[i+1] = Tfront_[i] * cvt::toMat44RotZ(currentJointAngles_[i]) * Ti_[i+1];
    for(int i=0;i<nJoint_;i++) Tfront_[i+1] = Tfront_[i] * cvt::toMat44RTFromDH(targetJointAngles_[i], DHs_[i+1]);
    Tfront_.back() *= Ttip_;
    // TR * T
    // for(int i=nJoint_-1;i>=0;i--) Tback_[i] = Ti_[i] * cvt::toMat44RotZ(currentJointAngles_[i]) * Tback_[i+1];
    for(int i=nJoint_-1;i>=0;i--) Tback_[i] = cvt::toMat44TRFromDH(DHs_[i], targetJointAngles_[i]) * Tback_[i+1];

    /*
        x_i+h = T[0][i] * rotZ(angle + h) * T[i][nJoint_]
        x_i   = T[0][i] * rotZ(angle) * T[i][nJoint_]
        --->    x_i+h - x_i / h  = J[:][i];
    */
    Matrix4d MXi  = Tfront_.back();
    Matrix<double, 6, 1> Xi = cvt::toMat61XYZEuler(MXi);
    for(int i=0;i<nJoint_;i++){
        Matrix4d MXih = Tfront_[i]*cvt::toMat44RotZ(targetJointAngles_[i]+h)*Tback_[i+1];
        Matrix<double, 6, 1> Xih = cvt::toMat61XYZEuler(MXih);
        for(int j=0;j<6;j++){
            if(Xih(j,0) > Xi(j,0) + M_PI) Xih(j,0) -= 2.0 * M_PI;
            if(Xih(j,0) < Xi(j,0) - M_PI) Xih(j,0) += 2.0 * M_PI;
            J_(j, i) = ( Xih(j,0) - Xi(j,0) )/h;
        }
    }
}

// targetAngles.size() == always 6(x,y,z,ax,ay,az) [m, deg]
// return size == nJoint [deg]
vector<double> RobotSolver::numericIK(const vector<double>& targetPose, int maxLoop){

    // targetJointAngles_ = currentJointAngles_;

    Matrix<double, 6,1> targetX = cvt::toMat61XYZEuler(targetPose);

    for(int loop=0; loop<maxLoop; loop++){

        //J_
        this->_calculateJ();

        //    dq = J-1 * dx
        Matrix<double, 6, 1> currentX = cvt::toMat61XYZEuler(Tfront_.back());
        Matrix<double, 6, 1> dX = targetX - currentX;
        for(int i=3;i<6;i++){
            double& val = dX(i,0);
            if(val>M_PI)  val-=2.0*M_PI;
            if(val<-M_PI) val+=2.0*M_PI;
        }

        double G = 0.5;

        //if(nJoint_==6) dq_ = J_.inverse() * dX * G;
        //else if(nJoint_>6) dq_ = J_.transpose()*(J_*J_.transpose()).inverse()* dX * G;

        dq_ = J_.transpose()*(J_*J_.transpose()).inverse()* dX * G;

        // q += dq
        double limit = 0.4 * (maxLoop-loop)/maxLoop + 0.2;
        double dMax = 0.;
        for(int i=0;i<nJoint_;i++){
            double dAngle = dq_[i];
            chmin(dAngle, limit);
            chmax(dAngle, -limit);
            targetJointAngles_[i] += dAngle;
            chmax(dMax, abs(dAngle));
        }

        //    check angle range 
        for(int i=0;i<nJoint_;i++){
            // targetJointAngles_[i] = max(targetJointAngles_[i], minAngles_[i]);
            // targetJointAngles_[i] = min(targetJointAngles_[i], maxAngles_[i]);
            if(targetJointAngles_[i]<-M_PI) targetJointAngles_[i] += 2.0*M_PI;
            if(targetJointAngles_[i]>M_PI) targetJointAngles_[i] -= 2.0*M_PI;

            chmin(targetJointAngles_[i], maxAngles_[i]);
            chmax(targetJointAngles_[i], minAngles_[i]);


        }
        //ES(loop) EL(targetJointAngles_)
        if(dMax<eps) break;
    }

    return targetJointAngles_;
}

void RobotSolver::setCurrentAngles(const vector<double> currentAngles){
    //if(currentAngles.size()!=currentJointAngles_.size()) return false;
    currentJointAngles_ = currentAngles;
}

void RobotSolver::setTargetAngles(const vector<double> targetAngles){
    //if(targetAngles.size()!=targetJointAngles_.size()) return false;
    targetJointAngles_ = targetAngles;
}


vector<double> RobotSolver::getCommandJointAngles(){
    for(int i=0;i<nJoint_;i++){
        double dVelocity = Gp_*(targetJointAngles_[i] - currentJointAngles_[i]) + Gd_*(0. - jointVelocity_[i]);
        chmin(dVelocity,  jointMaxAccel_[i]);
        chmax(dVelocity, -jointMaxAccel_[i]);
        
        jointVelocity_[i] += dVelocity;
        chmin(jointVelocity_[i], jointMaxVelocity_[i]);
        chmax(jointVelocity_[i], -jointMaxVelocity_[i]);

        currentJointAngles_[i] += jointVelocity_[i];
    }
    return currentJointAngles_;
}

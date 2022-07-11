#include "robot_solver.h"
#include "development_commands.h"
#include "cvt_convert_functions.h"

#include <cassert>
#include <cmath>
#include <algorithm>




RobotSolver::RobotSolver(){
    // this->_initSpecificParams6dArm();
    // this->_initSpecificParamsCobottaArmOnly();
    this->_initSpecificParamsCobottaArmAndTool();
    this->_initCommon();
    initialized_ = true;
    cout<<"RobotSolver constructed"<<endl;
}

RobotSolver::~RobotSolver(){
    cout<<"RobotSolver desstructed"<<endl;
}

// manually set robot parameters depending on your robot
// this is template therefore create copy of this function 
void RobotSolver::_initSpecificParamsTemplate(){
    nJoint_ = 6;
    //moters' Range [deg]
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);
    //init currentAngles as 0
    currentAngles_ = vector<double>(nJoint_, 0.);

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({0., 0., 0., 0.}); //base=>joint[0]
    DHs_.push_back({0., 0., 0., 0.}); //joint[0]->joint[1]
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.}); //joint[5]->armTip

    //DHs_.size() == nJoint + tipPose
    assert((int)DHs_.size() == nJoint_+1);
}

void RobotSolver::_initSpecificParams6dArm(){
    nJoint_ = 6;
    //moters' Range [deg]
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);
    //init currentAngles as 0
    currentAngles_ = vector<double>(nJoint_, 0.);

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({0., 0., 0., 0.}); //base=>joint[0]
    DHs_.push_back({0., 0., 0., 0.}); //joint[0]->joint[1]
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.});
    DHs_.push_back({0., 0., 0., 0.}); //joint[5]->armTip

    //DHs_.size() == nJoint + tipPose
    assert((int)DHs_.size() == nJoint_+1);
}



void RobotSolver::_initSpecificParamsCobottaArmOnly(){
    nJoint_ = 6;
    //moters' Range [deg]
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);
    //init currentAngles as 0
    currentAngles_ = vector<double>(nJoint_, 0.);

    //global origin to base. dx,dy,dz,dax,day,daz [m, deg]
    basePose_ = {-0.25, -0.23, 0., 0., 0., 90.};
    assert((int)basePose_.size()==6);

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({0., 0., 0.175, 0.}); //base=>joint[0]
    DHs_.push_back({0., 270., 0., 270.}); //joint[0]->joint[1]
    DHs_.push_back({0.17, 180., -0.02, 0.}); 
    DHs_.push_back({0.01, 90., 0.175, 0.}); //dummy3->dummy4
    DHs_.push_back({0., 270., 0.064, 0.});
    DHs_.push_back({0., 90., 0.0598, 0.});
    DHs_.push_back({0.12, 270., 0.175, 0.}); //joint[6]->armTip

    //DHs_.size() == nJoint + tipPose
    assert((int)DHs_.size() == nJoint_+1);
}


void RobotSolver::_initSpecificParamsCobottaArmAndTool(){
    nJoint_ = 7;
    //moters' Range [deg]
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);
    //init currentAngles as 0
    currentAngles_ = vector<double>(nJoint_, 0.);

    //global origin to base. dx,dy,dz,dax,day,daz [m, deg]
    basePose_ = {-0.25, -0.23, 0., 0., 0., 90.};
    assert((int)basePose_.size()==6);

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({0., 0., 0.175, 0.}); //base=>joint[0]
    DHs_.push_back({0., 270., 0., 270.}); //joint[0]->joint[1]
    DHs_.push_back({0.17, 180., -0.02, 0.}); 
    DHs_.push_back({0.01, 90., 0.175, 0.}); //dummy3->dummy4
    DHs_.push_back({0., 270., 0.064, 0.});
    DHs_.push_back({0., 90., 0.0598, 0.});
    DHs_.push_back({0., 270., 0.09, 0.}); //joint[6]->toolJoint
    DHs_.push_back({0.012, 0., 0.085, 0.}); //toolJolint->armTip

    //DHs_.size() == nJoint + tipPose
    assert((int)DHs_.size() == nJoint_+1);
}


// applied for any robot
void RobotSolver::_initCommon(){
    // deg -> rad
    for(int i=0;i<nJoint_;i++) minAngles_[i] *= M_PI/180.;
    for(int i=0;i<nJoint_;i++) maxAngles_[i] *= M_PI/180.;
    for(int i=0;i<nJoint_;i++) currentAngles_[i] *= M_PI/180.;
    for(int i=3;i<6;i++) basePose_[i] *= M_PI/180.;
    for(int i=0;i<(int)DHs_.size();i++) DHs_[i].alp *= M_PI/180.;
    for(int i=0;i<(int)DHs_.size();i++) DHs_[i].tht *= M_PI/180.;

    Affine3d aff = Eigen::Translation<double,3>(basePose_[0], basePose_[1], basePose_[2])
                * AngleAxisd(basePose_[3], Vector3d::UnitX())
                * AngleAxisd(basePose_[4], Vector3d::UnitY())
                * AngleAxisd(basePose_[5], Vector3d::UnitZ());
    // auto rot = aff.rotation();
    // auto trans = aff.translation();
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) Tbase_(i,j) = aff(i,j);
 
    Ti_ = vector<Matrix4d>(nJoint_+1);
    Ti_[0] = Tbase_ * cvt::toMat44FromDH(DHs_[0]);
    for(int i=0;i<nJoint_;i++){
        Ti_[i+1] = cvt::toMat44FromDH(DHs_[i+1]);
    }

    //for IK calc
    J_.resize(6, nJoint_);
    Tfront_.resize(nJoint_+1); // base -> joint[i]
    Tback_.resize(nJoint_+1);  // joint[i] -> base

}


// jointAngles.size() == nJoint [rad]
// return size == always 6(xyz,eulerxyz) [m,rad]
vector<double> RobotSolver::FK(const vector<double>& jointAngles){
    //Tfrom base to Tip
    Matrix4d Tb2t = Ti_[0];
    // for(int i=0;i<nJoint_;i++) Tb2t *= cvt::toMat44RotZ(jointAngles[i])*Ti_[i+1];
    for(int i=0;i<nJoint_;i++) Tb2t *= cvt::toMat44RTFromDH(jointAngles[i], DHs_[i+1]);
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
        // for(int i=0;i<nJoint_;i++) Tfront_[i+1] = Tfront_[i] * cvt::toMat44RotZ(currentAngles_[i]) * Ti_[i+1];
        for(int i=0;i<nJoint_;i++) Tfront_[i+1] = Tfront_[i] * cvt::toMat44RTFromDH(currentAngles_[i], DHs_[i+1]);
        // TR * T
        // for(int i=nJoint_-1;i>=0;i--) Tback_[i] = Ti_[i] * cvt::toMat44RotZ(currentAngles_[i]) * Tback_[i+1];
        for(int i=nJoint_-1;i>=0;i--) Tback_[i] = cvt::toMat44TRFromDH(DHs_[i], currentAngles_[i]) * Tback_[i+1];

        /*
            x_i+h = T[0][i] * rotZ(angle + h) * T[i][nJoint_]
            x_i   = T[0][i] * rotZ(angle) * T[i][nJoint_]
            --->    x_i+h - x_i / h  = J[:][i];
        */
        Matrix4d MXi  = Tfront_.back();
        Matrix<double, 6, 1> Xi = cvt::toMat61XYZEuler(MXi);
        for(int i=0;i<nJoint_;i++){
            Matrix4d MXih = Tfront_[i]*cvt::toMat44RotZ(currentAngles_[i]+h)*Tback_[i+1];
            Matrix<double, 6, 1> Xih = cvt::toMat61XYZEuler(MXih);
            for(int j=0;j<6;j++){
                if(Xih(j,0) > Xi(j,0) + M_PI) Xih(j,0) -= 2.0 * M_PI;
                if(Xih(j,0) < Xi(j,0) - M_PI) Xih(j,0) += 2.0 * M_PI;
                J_(j, i) = ( Xih(j,0) - Xi(j,0) )/h;
            }
        }
}


vector<double> RobotSolver::_redundantIK(const vector<double>& targetPose, int maxLoop){

    Matrix<double, 6,1> targetX = cvt::toMat61XYZEuler(targetPose);

    for(int loop=0; loop<maxLoop;loop++){

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

        // dq = J-1 * dx        
        Matrix<double, Dynamic, 1> dq = J_.transpose()*(J_*J_.transpose()).inverse()* dX;
        double limit = 0.3 * (maxLoop-loop)/maxLoop + 0.2;
        double dMax = 0.;
        for(int i=0;i<nJoint_;i++){
            double dAngle = max(min(dq[i], limit), -limit);
            currentAngles_[i] += dAngle;
            dMax = max(dMax, abs(dAngle));
        }

        //    check angle range 
        for(int i=0;i<nJoint_;i++){
            // currentAngles_[i] = max(currentAngles_[i], minAngles_[i]);
            // currentAngles_[i] = min(currentAngles_[i], maxAngles_[i]);
            if(currentAngles_[i]<-M_PI) currentAngles_[i] += 2.0*M_PI;
            if(currentAngles_[i]>M_PI) currentAngles_[i] -= 2.0*M_PI;
        }
        //ES(loop) EL(currentAngles_)
        //EL(dMax)
        if(dMax<eps) break;
    }    

    return vector<double>(nJoint_, 0.);
}




vector<double> RobotSolver::_uniqueIK(const vector<double>& targetPose, int maxLoop){

    Matrix<double, 6,1> targetX = cvt::toMat61XYZEuler(targetPose);

    for(int loop=0; loop<maxLoop;loop++){

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
        Matrix<double, 6, 1> dq = J_.inverse() * dX;

        // q += dq
        double limit = 0.4 * (maxLoop-loop)/maxLoop + 0.2;
        double dMax = 0.;
        for(int i=0;i<nJoint_;i++){
            double dAngle = max(min(dq[i], limit), -limit);
            currentAngles_[i] += dAngle;
            dMax = max(dMax, abs(dAngle));
        }

        //    check angle range 
        for(int i=0;i<nJoint_;i++){
            // currentAngles_[i] = max(currentAngles_[i], minAngles_[i]);
            // currentAngles_[i] = min(currentAngles_[i], maxAngles_[i]);
            if(currentAngles_[i]<-M_PI) currentAngles_[i] += 2.0*M_PI;
            if(currentAngles_[i]>M_PI) currentAngles_[i] -= 2.0*M_PI;
        }
        //ES(loop) EL(currentAngles_)
        if(dMax<eps) break;
    }

    return vector<double>(nJoint_, 0.);
}

vector<double> RobotSolver::_undeterminedIK(const vector<double>& targetPose){


    return vector<double>(nJoint_, 0.);
}

// targetAngles.size() == always 6(x,y,z,ax,ay,az) [m, deg]
// return size == nJoint [deg]
vector<double> RobotSolver::numericIK(const vector<double>& targetPose){
    vector<double> jointAngles;

    if(nJoint_ == 6) jointAngles = this->_uniqueIK(targetPose);

    if(nJoint_ > 6) jointAngles = this->_redundantIK(targetPose, 1);
    // else if(nJoint_ == 6){
    // else jointAngles = this->_undeterminedIK(targetPose);


    return jointAngles;
}

bool RobotSolver::setCurrentAngles(const vector<double> angles){
    if(angles.size()!=currentAngles_.size()) return false;
    currentAngles_ = angles;
    return true;
}

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

    // J_.resize(6, nJoint_);


}


// jointAngles.size() == nJoint [rad]
// return size == always 6(xyz,eulerxyz) [m,rad]
vector<double> RobotSolver::FK(const vector<double>& jointAngles){
    //Tfrom base to Tip
    Matrix4d Tb2t = Ti_[0];
    for(int i=0;i<nJoint_;i++){
        Tb2t *= cvt::toMat44RotZ(jointAngles[i])*Ti_[i+1];
        //EL(Tb2t)
    }
    vector<double> tipPose = cvt::toVecXYZEuler(Tb2t);
    //EL(tipPose)
    return tipPose;
}

vector<double> RobotSolver::_redundantIK(const vector<double>& targetPose){

    /*
        euler -> quaternion
        X           :: x, y, z, qx, qy, qz
    */
    Matrix<double, 6,1> targetX = cvt::toMat61XYZEuler(targetPose);

    /*
        x = Jq
        J[0] = { dx/dq0,  }
        dx = f(q+h)-f(q) / h;
    */
    double h = 0.0001;
    Matrix<double, 6, Dynamic> J;
    J.resize(6, nJoint_);

    int nloop = 1000;
    for(int loop=0; loop<nloop;loop++){

        /*
            X_i+h = Tfront[i] * rotZ(angle+h) * Tback[i]
            X_i   = Tfront[i] * rotZ(angle)   * Tback[i]
        */
        vector<Matrix4d> Tfront(nJoint_+1); // base -> joint[i]
        vector<Matrix4d> Tback(nJoint_+1);  // joint[i] -> base
        //for(int i=0;i<nJoint_+1;i++) PL(Ti_[i])
        Tfront[0] = Ti_[0];
        Tback.back() = Ti_.back();
        for(int i=0;i<nJoint_;i++) Tfront[i+1] = Tfront[i] * cvt::toMat44RotZ(currentAngles_[i]) * Ti_[i+1];
        for(int i=nJoint_-1;i>=0;i--) Tback[i] = Ti_[i] * cvt::toMat44RotZ(currentAngles_[i]) * Tback[i+1];

        /*
            x_i+h = T[0][i] * rotZ(angle + h) * T[i][nJoint_]
            x_i   = T[0][i] * rotZ(angle) * T[i][nJoint_]
            --->    x_i+h - x_i / h  = J[:][i];
        */
        for(int i=0;i<nJoint_;i++){
            Matrix4d MXih = Tfront[i]*cvt::toMat44RotZ(currentAngles_[i]+2*h)*Tback[i+1];
            Matrix4d MXi  = Tfront[i]*cvt::toMat44RotZ(currentAngles_[i]+h)  *Tback[i+1];
            Matrix<double, 6, 1> Xih = cvt::toMat61XYZEuler(MXih);
            Matrix<double, 6, 1> Xi = cvt::toMat61XYZEuler(MXi);
            for(int j=0;j<6;j++) {ES(i) ES(j) ES(Xi(j,0)) EL(Xih(j,0))}
            // EL(i)
            // PL(Tfront[i]*Tback[i+1])
            
            for(int j=0;j<6;j++){
                if(Xih(j,0) > Xi(j,0) + M_PI) Xih(j,0) -= 2.0 * M_PI;
                if(Xih(j,0) < Xi(j,0) - M_PI) Xih(j,0) += 2.0 * M_PI;
                J(j, i) = ( Xih(j,0) - Xi(j,0) )/h;
            }
            // for(int j=0;j<6;j++) {ES(i) ES(j) ES(currentAngles_[i]) ES(Xi(j,0)) EL(Xih(j,0)) 
            //     if(abs(Xih(j,0)-Xi(j,0))>0.5){
            //         PL(cvt::toMat44RotZ(currentAngles_[i]+h))
            //         PL(cvt::toMat44RotZ(currentAngles_[i]))
            //         PL("Mx")
            //         PL(MXih)
            //         PL(MXi)
            //     }
            // }
            //PL(J)
        }
        // EL(J)
        //EL(J.inverse())

        /*
            dq = J-1 * dx
        */
        Matrix<double, 6, 1> currentX = cvt::toMat61XYZEuler(Tfront.back());
        // EL(currentX)
        Matrix<double, 6, 1> dX = targetX - currentX;
        //EL(dX)
        for(int i=3;i<6;i++){
            double& val = dX(i,0);
            if(val>M_PI)  val-=2.0*M_PI;
            if(val<-M_PI) val+=2.0*M_PI;
        }
        //EL(dX)
        
        Matrix<double, Dynamic, 1> dq = J.transpose()*(J*J.transpose()).inverse()* dX;
        // EL(dq)

        double limit = 0.3 * (nloop-loop)/nloop + 0.2;
        double dMax = 0.;
        for(int i=0;i<nJoint_;i++){
            double dAngle = max(min(dq[i], limit), -limit);// * 0.1;//*180./M_PI;
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
        ES(loop) EL(currentAngles_)
        double eps = 1e-7;
        if(dMax<eps) break;
    }    

    return vector<double>(nJoint_, 0.);
}




vector<double> RobotSolver::_uniqueIK(const vector<double>& targetPose){

    /*
        euler -> quaternion
        X           :: x, y, z, qx, qy, qz
    */
    Matrix<double, 6,1> targetX = cvt::toMat61XYZEuler(targetPose);
    // EL(targetPose)
    // EL(targetX)

    // vector<double> tmp = vector<double>(6,0.);
    // Matrix<double, 6,1> actualX = cvt::toMat61XYZQuat(this->FK(tmp));
    // EL(actualX)

    /*
        x = Jq
        J[0] = { dx/dq0,  }
        dx = f(q+h)-f(q) / h;
    */
    double h = 0.0001;
    // PL("h?") 
    // cin>>h;

    int nloop = 100;
    for(int loop=0; loop<nloop;loop++){

        Matrix<double, 6, Dynamic> J;
        J.resize(6, nJoint_);
        /*
            X_i+h = Tfront[i] * rotZ(angle+h) * Tback[i]
            X_i   = Tfront[i] * rotZ(angle)   * Tback[i]
        */
        vector<Matrix4d> Tfront(nJoint_+1); // base -> joint[i]
        vector<Matrix4d> Tback(nJoint_+1);  // joint[i] -> base
        //for(int i=0;i<nJoint_+1;i++) PL(Ti_[i])
        Tfront[0] = Ti_[0];
        Tback.back() = Ti_.back();
        for(int i=0;i<nJoint_;i++) Tfront[i+1] = Tfront[i] * cvt::toMat44RotZ(currentAngles_[i]) * Ti_[i+1];
        for(int i=nJoint_-1;i>=0;i--) Tback[i] = Ti_[i] * cvt::toMat44RotZ(currentAngles_[i]) * Tback[i+1];

        /*
            x_i+h = T[0][i] * rotZ(angle + h) * T[i][nJoint_]
            x_i   = T[0][i] * rotZ(angle) * T[i][nJoint_]
            --->    x_i+h - x_i / h  = J[:][i];
        */
        for(int i=0;i<nJoint_;i++){
            Matrix4d MXih = Tfront[i]*cvt::toMat44RotZ(currentAngles_[i]+h)*Tback[i+1];
            Matrix4d MXi  = Tfront[i]*cvt::toMat44RotZ(currentAngles_[i])  *Tback[i+1];
            Matrix<double, 6, 1> Xih = cvt::toMat61XYZEuler(MXih);
            Matrix<double, 6, 1> Xi = cvt::toMat61XYZEuler(MXi);
            
            for(int j=0;j<6;j++){
                if(Xih(j,0) > Xi(j,0) + M_PI) Xih(j,0) -= 2.0 * M_PI;
                if(Xih(j,0) < Xi(j,0) - M_PI) Xih(j,0) += 2.0 * M_PI;
                J(j, i) = ( Xih(j,0) - Xi(j,0) )/h;
            }
            //for(int j=0;j<6;j++) {ES(i) ES(j) ES(Xi(j,0)) EL(Xih(j,0))}
            //PL(J)
        }
        // EL(J)
        // EL(J.inverse())

        /*
            dq = J-1 * dx
        */
        Matrix<double, 6, 1> currentX = cvt::toMat61XYZEuler(Tfront.back());
        // EL(currentX)
        Matrix<double, 6, 1> dX = targetX - currentX;
        //EL(dX)
        for(int i=3;i<6;i++){
            double& val = dX(i,0);
            if(val>M_PI)  val-=2.0*M_PI;
            if(val<-M_PI) val+=2.0*M_PI;
        }
        //EL(dX)
        
        Matrix<double, 6, 1> dq = J.inverse() * dX;


        double limit = 0.4 * (nloop-loop)/nloop + 0.2;
        double dMax = 0.;
        for(int i=0;i<nJoint_;i++){
            double dAngle = max(min(dq[i], limit), -limit);// * 0.1;//*180./M_PI;
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
        ES(loop) EL(currentAngles_)
        double eps = 1e-7;
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

    if(nJoint_ > 6) jointAngles = this->_redundantIK(targetPose);
    // else if(nJoint_ == 6){
    // else jointAngles = this->_undeterminedIK(targetPose);


    return jointAngles;
}

bool RobotSolver::setCurrentAngles(const vector<double> angles){
    if(angles.size()!=currentAngles_.size()) return false;
    currentAngles_ = angles;
    return true;
}

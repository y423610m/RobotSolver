#include "robot_solver.h"
#include "development_commands.h"

#include <cassert>
#include <cmath>


RobotSolver::RobotSolver(){
    // this->_initSpecificParams6dArm();
    this->_initSpecificParamsCobotta();
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

void RobotSolver::_initSpecificParamsCobotta(){
    nJoint_ = 6;
    //moters' Range [deg]
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);
    //init currentAngles as 0
    currentAngles_ = vector<double>(nJoint_, 0.);

    //global origin to base. dx,dy,dz,dax,day,daz
    basePose_ = {-0.25, -0.23, 0., 0., 0., 90.};
    assert((int)basePose_.size()==6);

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs_.push_back({0., 0., 0.175, 0.}); //base=>joint[0]
    DHs_.push_back({0., 270., 0., 270.}); //joint[0]->joint[1]
    DHs_.push_back({0.17, 180., -0.02, 0.}); 
    DHs_.push_back({0.01, 90., 0.175, 0.}); //dummy3->dummy4
    DHs_.push_back({0., 270., 0.064, 0.});
    DHs_.push_back({0., 90., 0.0598, 90.});
    DHs_.push_back({0.175, 0., 0., 270.}); //joint[6]->armTip

    //DHs_.size() == nJoint + tipPose
    assert((int)DHs_.size() == nJoint_+1);
}

//ref http:  kuwamai hatenablog 211232
//arg: dJ->jointAngle
Matrix4d makeDHMatrix(const DHParam& dh, const double& jointAngle = 0.){
    double a = dh.a;
    double alp = dh.alp;
    double d = dh.d;
    double tht = dh.tht;
    alp = alp * M_PI / 180.;
    tht = tht * M_PI / 180.;
    double ca = cos(alp), sa = sin(alp);
    double ct = cos(tht), st = sin(tht);
    double ja = jointAngle * M_PI / 180.;
    double cj = cos(ja), sj = sin(ja);

    Matrix4d retT;
    if(jointAngle ==0.)
        retT<<
            ct,    -st,    0.,     a,
            ca*st, ca*ct, -sa, -sa*d,
            sa*st, sa*ct,  ca,  ca*d,
            0.,     0.,     0.,     1.;

    else
        retT<<
            cj*ct-sj*ca*st, -cj*st-sj*ca*ct, sj*sa,  a*cj+d*sj*sa,
            sj*ct+cj*ca*st, -sj*st+cj*ca*ct, -cj*sa, a*sj-d*cj*sa,
            sa*st,          sa*ct,           ca,     d*ca,
            0.,             0.,              0.,      1.;
    
    return retT;
}

// applied for any robot
void RobotSolver::_initCommon(){

    //TO DO : convert DH angle into rad
    Ti_ = vector<Matrix4d>(nJoint_+1);
    for(int i=0;i<nJoint_;i++){
        Ti_[i] = makeDHMatrix(DHs_[i+1]);
    }

    //transformation from origin to base
    for(int i=3;i<6;i++) basePose_[i] *= M_PI/180.;
    Affine3d aff = Eigen::Translation<double,3>(basePose_[0], basePose_[1], basePose_[2])
                * AngleAxisd(basePose_[3], Vector3d::UnitX())
                * AngleAxisd(basePose_[4], Vector3d::UnitY())
                * AngleAxisd(basePose_[5], Vector3d::UnitZ());
    // auto rot = aff.rotation();
    // auto trans = aff.translation();
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) Tbase_(i,j) = aff(i,j);

}

Matrix4d makeRotZ(const double& angle){
    Affine3d aff = Translation<double,3>(0.,0.,0.) * AngleAxisd(angle, Vector3d::UnitZ());
    Matrix4d retT;
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) retT(i,j) = aff(i,j);
    return retT;
}

// jointAngles.size() == nJoint [deg]
// return size == always 6(xyz,ax,ay,az) [m,deg]
vector<double> RobotSolver::FK(const vector<double>& jointAngles){
    //Tfrom base to Tip
    Matrix4d Tb2t = Tbase_ * makeDHMatrix(DHs_[0]);
    PS("dummy") PL(1)
    PL(Tb2t)
    for(int i=0;i<nJoint_;i++){
        Matrix4d newT = makeDHMatrix(DHs_[i+1], jointAngles[i]);
        //Tb2t *= newT;
        Tb2t *= makeRotZ(jointAngles[i])*Ti_[i];
        if(i+2!=nJoint_+1){PS("dummy") PL(i+2)} else{PL("tip")}
        PL(Tb2t)
    }

    vector<double> tipPose = vector<double>(6, 0.);
    for(int i=0;i<3;i++) tipPose[i] = Tb2t(i,3);
    // TO DO : get Rotation Params
    return tipPose;
}

// will not be used
vector<double> RobotSolver::moveTo(const vector<double>& targetPose){
    return vector<double>(6,0.);
}

// targetAngles.size() == always 6(x,y,z,ax,ay,az) [m, deg]
// return size == nJoint [deg]
vector<double> RobotSolver::numericIK(const vector<double>& targetPose){
    vector<double> jointAngles(nJoint_, 0.);

    return jointAngles;
}


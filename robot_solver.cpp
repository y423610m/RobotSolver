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
    DHs.push_back({0., 0., 0., 0.}); //base=>joint[0]
    DHs.push_back({0., 0., 0., 0.}); //joint[0]->joint[1]
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.}); //joint[5]->armTip

    //DHs.size() == nJoint + tipPose
    assert((int)DHs.size() == nJoint_+1);
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
    DHs.push_back({0., 0., 0., 0.}); //base=>joint[0]
    DHs.push_back({0., 0., 0., 0.}); //joint[0]->joint[1]
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.});
    DHs.push_back({0., 0., 0., 0.}); //joint[5]->armTip

    //DHs.size() == nJoint + tipPose
    assert((int)DHs.size() == nJoint_+1);
}

void RobotSolver::_initSpecificParamsCobotta(){
    nJoint_ = 6;
    //moters' Range [deg]
    minAngles_ = vector<double>(nJoint_, 0.);
    maxAngles_ = vector<double>(nJoint_, 180.);
    //init currentAngles as 0
    currentAngles_ = vector<double>(nJoint_, 0.);

    //set DH Parameters {a, alp, d, tht} [m, deg]
    //*******  set tht as every Joint are 0 ***********
    DHs.push_back({0., 0., 0.175, 0.}); //base=>joint[0]
    DHs.push_back({0., 270., 0., 270.}); //joint[0]->joint[1]
    DHs.push_back({0.17, 180., 0., 0.}); //dummy3->dummy4
    DHs.push_back({0., 90., 0.1, 0.}); 
    DHs.push_back({0., 270., 0.064, 0.});
    DHs.push_back({0., 90., 0.0598, 0.});
    DHs.push_back({0., 0., 0.175, 270.}); //joint[6]->armTip

    //DHs.size() == nJoint + tipPose
    assert((int)DHs.size() == nJoint_+1);
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
    //jointAngle = 0
    retT<<
        ct,    -st,    0.,     a,
        ca*st, ca*ct, -sa, -sa*d,
        sa*st, sa*ct,  ca,  ca*d,
        0.,     0.,     0.,     1.;

    //jointAngle !=0.
    //retT  = rotT(jointAngle) * (i-1)T(i)

    // retT<<
    //     cj*ct-sj*ca*st, -cj*st-sj*ca*ct, sj*sa,  a*cj+d*sj*sa,
    //     sj*ct+cj*ca*st, -sj*st+cj*ca*ct, -cj*sa, a*sj-d*cj*sa,
    //     sa*st,          sa*ct,           ca,     d*ca,
    //     0.,             0.,              0.,      1.;
    
    return retT;
}

// applied for any robot
void RobotSolver::_initCommon(){
    // Ts = vector<Matrix4d>(nJoint_+1);
    // for(int i=0;i<nJoint_+1;i++){
    //     Ts[i] = makeDHMatrix(DHs[i]);
    // }
}

// jointAngles.size() == nJoint [deg]
// return size == always 6(xyz,ax,ay,az) [m,deg]
vector<double> RobotSolver::FK(const vector<double>& jointAngles){
    //Tfrom base to Tip
    Matrix4d Tb2t = makeDHMatrix(DHs[0]);
    // PL(0)
    // PL(Tb2t)
    for(int i=0;i<nJoint_;i++){
        Matrix4d newT = makeDHMatrix(DHs[i+1], jointAngles[i]);
        Tb2t *= newT;
        // PL(i+1)
        // PL(Tb2t)
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


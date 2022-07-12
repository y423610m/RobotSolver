#include "robot_manager.h"
#include "development_commands.h"

#include <thread>
#include <chrono>


RobotManager::RobotManager()
:solver_(new RobotSolver())
,gpio_(new GPIOInterface())
{
    //get Parameters
    nJoint_ = solver_->getNJoint();
    minAngles_ = solver_->getMinAngles();
    maxAngles_ = solver_->getMaxAngles();


    //init every joints as 0
    // gpio_->setMoterAngles(vector<double>(nJoint_, 0.), minAngles_, maxAngles_);
    // this_thread::sleep_for(chrono::milliseconds(1000));



    initialized_ = true;
    cout<<"RobotManager constructed"<<endl;
}

RobotManager::~RobotManager(){
    if(!initialized_) return;
    cout<<"RobotManager destructed"<<endl;
}

void RobotManager::update(){
    if(!initialized_) return;


    //gpio_->update();

    /*
        適当な関節角度与える
        その時の姿勢取得
        IK後の関節角度と姿勢を確認
    */

    solver_->setCurrentAngles(vector<double>(6,0.1));

    double a = 0.5;
    //; PL("A?") cin>>a;
    vector<double> jointAngles = vector<double>(6, a);
    vector<double> tipPose = solver_->FK(jointAngles);

    //solver_->setCurrentAngles(tipPose);


    solver_->numericIK(tipPose);
    vector<double> resultJointAngles = solver_->getCurrentAngles();
    vector<double> resultTipPose = solver_->FK(resultJointAngles);

    // PL("----------result---------")
    // EL(jointAngles)
    // EL(tipPose)
    // EL(resultJointAngles)
    // EL(resultTipPose)

    cnt_++;

    return;


    //gpio_->setMoterAngles(targetAngles, minAngles_, maxAngles_);

}

bool RobotManager::checkLoop(){
    if(cnt_>1000) return false;
    return true;
}
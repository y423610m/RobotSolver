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
    gpio_->setMoterAngles(vector<double>(nJoint_, 0.), minAngles_, maxAngles_);
    this_thread::sleep_for(chrono::milliseconds(1000));


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

    vector<double> targetAngles = solver_->numericIK(vector<double>(6, 0.));

    vector<double> tipPose = solver_->FK(targetAngles);
    // auto tipPose_ = tipPose;
    // tipPose_[0] -= 0.25;
    // tipPose_[1] -= 0.23;
    // PS(cnt_++) EL(tipPose_)

    //gpio_->setMoterAngles(targetAngles, minAngles_, maxAngles_);

}

bool RobotManager::checkLoop(){
    return true;
}
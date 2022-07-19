#include "robot_manager.h"
#include "development_commands.h"

#include <thread>
#include <chrono>
#include <cmath>


RobotManager::RobotManager()
:solver_(new RobotSolver())
,ros_interface_(new ROSInterface())
{
    //get Parameters
    // nJoint_ = solver_->getNJoint();
    // minAngles_ = solver_->getMinAngles();
    // maxAngles_ = solver_->getMaxAngles();
    // currentJointAngles_.resize(nJoint_);
    // currentTipPose_.resize(6);
    // targetJointAngles_.resize(nJoint_);
    // targetTipPose_.resize(6);
    // commandJointAngles_.resize(nJoint_);

    //init every joints as 0
    // gpio_->setMoterAngles(vector<double>(nJoint_, 0.), minAngles_, maxAngles_);
    // this_thread::sleep_for(chrono::milliseconds(1000));

    //  実機の関節角度でsolver初期化
    solver_->setCurrentAngles(ros_interface_->getActualJointPosition());

    initialized_ = true;
    cout<<"RobotManager constructed"<<endl;
}

RobotManager::~RobotManager(){
    if(!initialized_) return;
    cout<<"RobotManager destructed"<<endl;
}

void RobotManager::update(){
    if(!initialized_) return;
    if(ros_interface_->getActualJointPosition().size()==0) return;

 
    /*
        IKPeriod_毎に現在角度取得＆目標角度をIK計算
        Cobottaへの入力は、(dq/T) * (t-Tsin(t/T))
        t = IKcnt, T = period
        速度が(1-cos)になってcollision防げる？
    */

    //solver_->setCurrentAngles(vector<double>(nJoint_,0.1));


    if(IKCnt_==0){
        currentJointAngles_ = ros_interface_->getActualJointPosition();
        solver_->setCurrentAngles(currentJointAngles_);
        currentTipPose_ = solver_->FK(currentJointAngles_);

        targetJointAngles_ = currentJointAngles_;
        if( (loopCnt_/IKPeriod_)&1 ) targetJointAngles_[0] = 0.5;
        else targetJointAngles_[0] = 0.0;
        //if(targetJointAngles_[0]<minAngles_[0] || maxAngles_[0]<targetJointAngles_[0]) targetJointAngles_[0] = 
        solver_->setTargetAngles(targetJointAngles_);

        //targetJointAngles_ = solver_->numericIK(targetTipPose_);
    }

    // for(int i=0;i<nJoint_;i++) 
    //     commandJointAngles_[i] = (1.0*(targetJointAngles_[i]-currentJointAngles_[i])/IKPeriod_)
    //                                 *(1.0*IKCnt_-1.0*IKPeriod_*sin(1.0*(1+IKCnt_)/IKPeriod_));



    commandJointAngles_ = solver_->getCommandJointAngles();

    ros_interface_->publishJointAngles(commandJointAngles_);

    // PS(targetJointAngles_[0]) PL(commandJointAngles_[0])

    // PL("----------result---------")
    // EL(currentJointAngles_)
    // EL(currentTipPose_)
    // EL(targetJointAngles_)
    // EL(targetTipPose_)

    loopCnt_++;
    IKCnt_++;

    if(IKCnt_&IKPeriod_) IKCnt_=0;

    return;


    //gpio_->setMoterAngles(targetAngles, minAngles_, maxAngles_);

}

bool RobotManager::checkLoop(){
    if(loopCnt_>100) return false;
    return true;
}
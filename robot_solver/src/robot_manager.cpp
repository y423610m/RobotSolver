#include "robot_manager.h"
#include "development_commands.h"
#include "robot_type_definitions.h"

#include <thread>
#include <chrono>
#include <cmath>

#include <string>


RobotManager::RobotManager(int RobotType)
:ros_interface_(new ROSInterface(RobotType))
,solver_(new RobotSolver(RobotType))
,RobotType_(RobotType)
{
    //  実機の関節角度でsolver初期化
    //solver_->setCurrentAngles(ros_interface_->getActualJointPosition());

    initialized_ = true;
    cout<<"RobotManager constructed"<<endl;
}

RobotManager::~RobotManager(){
    if(!initialized_) return;
    cout<<"RobotManager destructed"<<endl;
}

void RobotManager::update(){
    if(!initialized_) return;

    bool ok = false;

    if(RobotType_==RobotType_CobottaWithTool) ok = this->_updateCobottaWithTool();
    if(RobotType_==RobotType_CobottaWithoutTool) ok = this->_updateCobottaWithoutTool();
    if(RobotType_==RobotType_6DOFArm) ok = this->_update6DOFArm();


    if(!ok) return;

    loopCnt_++;
    IKCnt_++;
    if(IKCnt_&IKPeriod_) IKCnt_=0;
}

bool RobotManager::_updateCobottaWithTool(){
    return false;
}

bool RobotManager::_updateCobottaWithoutTool(){
    if(ros_interface_->getActualJointPosition().size()==0) return false;

    if(loopCnt_==0) solver_->setCurrentAngles(ros_interface_->getActualJointPosition());

 
    if(IKCnt_==0){

        currentJointAngles_ = ros_interface_->getActualJointPosition();
        // solver_->setCurrentAngles(currentJointAngles_);
        // currentTipPose_ = solver_->FK(currentJointAngles_);

        targetJointAngles_ = currentJointAngles_;
        if( loopCnt_&IKPeriod_ ) targetJointAngles_[0] = 0.5;
        else targetJointAngles_[0] = 0.0;
        targetTipPose_ = solver_->FK(targetJointAngles_);

        PS("tar Ang") PL(targetJointAngles_)
        PS("tar Pose") PL(targetTipPose_)

        targetJointAngles_ = solver_->numericIK(targetTipPose_);
        PS("com Ang") PL(targetJointAngles_)
        PS("com Pose") PL(solver_->FK(targetJointAngles_))

        //solver_->setTargetAngles(targetJointAngles_);
    }

    if(1){
        //yzxQuat
        targetTipPose_ = ros_interface_->getTargetTipPose();
        /*
            ->
        */
       PL(targetTipPose_)
        



    }

    commandJointAngles_ = solver_->getCommandJointAngles();
    ros_interface_->publishCobottaWithoutTool(commandJointAngles_);
    // PS(targetJointAngles_[0]) PL(commandJointAngles_[0])

    /*
        IKPeriod_毎に現在角度取得＆目標角度をIK計算
        Cobottaへの入力は、(dq/T) * (t-Tsin(t/T))
        t = IKcnt, T = period
        速度が(1-cos)になってcollision防げる？
    */
    // for(int i=0;i<nJoint_;i++) 
    //     commandJointAngles_[i] = (1.0*(targetJointAngles_[i]-currentJointAngles_[i])/IKPeriod_)
    //                                 *(1.0*IKCnt_-1.0*IKPeriod_*sin(1.0*(1+IKCnt_)/IKPeriod_));
    return true;
}

bool RobotManager::_update6DOFArm(){
    return false;
}

bool RobotManager::checkLoop(){
    //if(loopCnt_>100) return false;
    return true;
}
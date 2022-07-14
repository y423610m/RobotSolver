#include "robot_manager.h"
#include "development_commands.h"

#include <thread>
#include <chrono>


RobotManager::RobotManager()
:solver_(new RobotSolver())
,gpio_(new GPIOInterface())
,ros_interface_(new ROSInterface())
{
    //get Parameters
    nJoint_ = solver_->getNJoint();
    minAngles_ = solver_->getMinAngles();
    maxAngles_ = solver_->getMaxAngles();


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
 
    //gpio_->update();

    /*
        適当な関節角度与える
        その時の姿勢取得
        IK後の関節角度と姿勢を確認
    */

    //solver_->setCurrentAngles(vector<double>(nJoint_,0.1));


    vector<double> jointAngles = ros_interface_->getActualJointPosition();
    int th = 10000;
    if(cnt_%th>th/2) jointAngles[0] += 0.1;
    else jointAngles[0] -= 0.1;
    //jointAngles[0] += 0.05*sin(0.1*cnt_);
    vector<double> tipPose = solver_->FK(jointAngles);

    //solver_->setCurrentAngles(tipPose);


    solver_->numericIK(tipPose);
    vector<double> resultJointAngles = solver_->getCurrentAngles();
    vector<double> resultTipPose = solver_->FK(resultJointAngles);


    ros_interface_->publishJointAngles(resultJointAngles);

    PL("----------result---------")
    EL(jointAngles)
    EL(tipPose)
    EL(resultJointAngles)
    EL(resultTipPose)

    cnt_++;

    return;


    //gpio_->setMoterAngles(targetAngles, minAngles_, maxAngles_);

}

bool RobotManager::checkLoop(){
    if(cnt_>1000) return false;
    return true;
}
/*
g++ -Wall -pthread main.cpp  robot_manager.cpp robot_solver.cpp gpio_interface.cpp cvt_convert_functions.cpp -lpigpio -lrt
sudo killall pigpiod
sudo ./a.out
*/

#include "robot_manager.h"

#include <memory>
#include <unistd.h>
#include <iostream>
using namespace std;

#include "development_commands.h"


int main(){

    unique_ptr<RobotManager> robot_manager(new RobotManager());

    while(robot_manager->checkLoop()){
        robot_manager->update();
        //break;
    }

    cout<<"program successfully ended"<<endl;
    return 0;
}
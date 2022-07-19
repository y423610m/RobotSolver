#include "pigpio_manager.h"


PigpioManager::PigpioManager()
: gpio_interface_(new GPIOInterface())
, ros_interface_(new ROSInterface())
{

}

PigpioManager::~PigpioManager(){

}

void PigpioManager::update(){

}

bool PigpioManager::checkLoop(){
    return true;
}
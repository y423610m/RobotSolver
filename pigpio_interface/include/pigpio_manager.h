#include <iostream>
#include <vector>
#include <memory>
using namespace std;

#include "gpio_interface.h"
#include "ros_interface.h"

class PigpioManager{
private:
    unique_ptr<GPIOInterface> gpio_interface_;
    unique_ptr<ROSInterface> ros_interface_;


public:
    PigpioManager();
    ~PigpioManager();
    void update();
    bool checkLoop();
};
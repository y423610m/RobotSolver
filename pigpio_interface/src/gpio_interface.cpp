#include "gpio_interface.h"
#include "development_commands.h"

#include <iostream>
#include <thread>
#include <chrono>
using namespace std;
using std::this_thread::sleep_for;

#if __has_include(<pigpio.h>)
    #include <pigpio.h>
    #define HAS_PIGPIO 1
#endif

// #if __has_include(<pigpiod_if.h>)
//     #include <pigpiod_if.h>
//     #define HAS_PIGPIO 1
// #endif

// #if __has_include(<pigpiod_if2.h>)
//     #include <pigpiod_if2.h>
//     #define HAS_PIGPIO 1
// #endif

#ifdef HAS_PIGPIO
    GPIOInterface::GPIOInterface()
    :pinModes_(vector<int>(100))
    {
        if( gpioInitialise() < 0 ){
            cout << "pigpio initialise failed\n";
            return;
        }
        pinAssign_ = vector<int>{21, 20, 16, 12, 26, 19, 13, 6};
        initialized_ = true;
        cout << "GPIOInterface constructed" <<endl;
    }


    GPIOInterface::~GPIOInterface(){
        if(!initialized_) return;
        gpioTerminate();
        cout << "GPIOInterface destructed" <<endl;
    }


    void GPIOInterface::update(){
        this->_LED(4,true);

        this->_servoMoter(21, 2500);
        //this->_servoMoter(21, 2500);

        //for(int i=500;i<2500;i++) gpioServo(21, i);
        sleep_for(chrono::milliseconds(1000));

    }


    void GPIOInterface::_LED(int pin, bool on = true){
        if(pinModes_[pin]!=PI_OUTPUT){
            gpioSetMode(pin, PI_OUTPUT);
            pinModes_[pin] = PI_OUTPUT;
        }
        if(on) gpioWrite(pin, 1);
        else gpioWrite(pin, 0);
    }

    void GPIOInterface::_servoMoter(int pin, int target){
        gpioServo(pin, target);
        
        sleep_for(chrono::milliseconds(1000));

    }

    void GPIOInterface::setMoterAngles(const vector<double>& targetAngles, const vector<double>& minAngles, const vector<double>& maxAngles){
        for(int i=0;i<(int)targetAngles.size();i++){
            //int target = minTarget_ + (targetAngles[i]/180.0)*(maxTarget_ - minTarget_);
            //gpioServo(pinAssign_[i], target);
            //PS("target") PS(i) PL(target)
            //if(i==0) break;
        }
        sleep_for(chrono::milliseconds(1000));
    }
#else
    //HAS_PIGPIO

    GPIOInterface::GPIOInterface(){}
    GPIOInterface::~GPIOInterface(){}
    void GPIOInterface::update(){}
    void GPIOInterface::setMoterAngles(const vector<double>& targetAngles, const vector<double>& minAngles, const vector<double>& maxAngles){}
    void _LED(int pin, bool on){}
    void _servoMoter(int pin, int target){};

#endif //HAS_PIGPIO
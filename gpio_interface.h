#include <pigpio.h>

#include <vector>
using namespace std;

class GPIOInterface{
private:
    bool initialized_ = false;
    vector<int> pinModes_;
    vector<int> pinAssign_;

    int minTarget_ = 500;
    int maxTarget_ = 2500;

    void _LED(int pin, bool on);
    void _servoMoter(int pin, int target);


public:
    GPIOInterface();
    ~GPIOInterface();
    void update();
    void setMoterAngles(const vector<double>& targetAngles, const vector<double>& minAngles, const vector<double>& maxAngles);

};
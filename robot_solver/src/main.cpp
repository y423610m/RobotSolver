#include <iostream>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "solver");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
    std_msgs::String msg;
    msg.data = "nenecchi";

    cout<<"hello world"<<endl;
    while(ros::ok()){

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


#ifndef CONTROLLER_CLASS_H
#define CONTROLLER_CLASS_H


#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/console.h"
#include <turtlesim/Pose.h>
#include "math.h"
#include <sstream>

class controller_class
{
private:
    
    geometry_msgs::Twist twist;
    turtlesim::Pose curr_pose;
    turtlesim::Pose ref;
    
    double err_ang = 0.0;
    double err_lin = 0.0;
    double err_ang_old = 0.0;
    double err_lin_old= 0.0;
    double kp1= 0.01;
    double ki1= 0.001;
    double kp2= 0.5;
    double ki2= 0;
    
    ros::NodeHandle n;
    ros::Subscriber current_pose_sub;
    ros::Subscriber reference_pose_sub;
    ros::Publisher controller_pub;
        
private:
    
    void ReadCurPos(const turtlesim::Pose::ConstPtr& msg);
    void ReadRef(const turtlesim::Pose::ConstPtr& msg);
    double AngularErr(turtlesim::Pose current, turtlesim::Pose reference);
    double LinearErr(turtlesim::Pose current, turtlesim::Pose reference);
    
public:
    
    controller_class();
    ~controller_class();
    void init();
    void run();
};

#endif //CONTROLLER_CLASS_H
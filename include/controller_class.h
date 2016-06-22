#ifndef CONTROLLER_CLASS_H
#define CONTROLLER_CLASS_H


#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/console.h"
#include <turtlesim/Pose.h>
#include "math.h"
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <controller/TutorialsConfig.h>


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
    std::vector<turtlesim::Pose> vertex;
    int vertex_count;
    int vertex_num;
    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;
   
    ros::NodeHandle n;
    ros::Subscriber current_pose_sub;
    ros::Publisher controller_pub;
    
public:
    
    double kp1= 0.5;
    double ki1= 0;
    double kp2= 2.5;
    double ki2= 0;
            
private:
    
    void ReadCurPos(const turtlesim::Pose::ConstPtr& msg);
    double AngularErr(turtlesim::Pose current, turtlesim::Pose reference);
    double LinearErr(turtlesim::Pose current, turtlesim::Pose reference);
    void SetRef(turtlesim::Pose center, double radius, int vertex_number);
    
public:
    
    controller_class();
    ~controller_class();
    void init();
    void run();
    void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level);

};

#endif //CONTROLLER_CLASS_H
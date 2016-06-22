#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "ros/console.h"
#include <turtlesim/Pose.h>
#include "math.h"
#include <sstream>
#include "controller_class.h"
#include <dynamic_reconfigure/server.h>
#include <controller/TutorialsConfig.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    
    controller_class contr;

    contr.init();
    contr.run();

    return 0;
}
#include "controller_class.h"

controller_class::controller_class()
{
    twist.angular.x = 1; //fail!!!
    twist.angular.y = 0;
    twist.angular.z = 0;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    
    curr_pose.x= 0;
    curr_pose.y = 0;
    curr_pose.theta = 0;
    
    ref.x = 0;
    ref.y = 0;
    ref.theta = 0;
}

controller_class::~controller_class(){}

void controller_class::ReadCurPos(const turtlesim::Pose::ConstPtr& msg)
{
    curr_pose = *msg;
}

void controller_class::ReadRef(const turtlesim::Pose::ConstPtr& msg)
{
    ref = *msg;
}

double controller_class::AngularErr(turtlesim::Pose current, turtlesim::Pose reference)
{
    double err_x = reference.x - current.x;
    double err_y = reference.y - current.y;
    double ref_theta = atan2f(err_y, err_x);
    return  ref_theta - current.theta;
}

double controller_class::LinearErr(turtlesim::Pose current, turtlesim::Pose reference)
{
    return  sqrt(pow((reference.y-current.y),2)+pow((reference.x-current.x),2));
}

void controller_class::init()
{
    current_pose_sub = n.subscribe("pose", 1, &controller_class::ReadCurPos, this);
    reference_pose_sub = n.subscribe("ref", 1, &controller_class::ReadRef, this);        
    controller_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);  
}

void controller_class::run()
{
    ros::Rate loop_rate(100);
    
    int count = 0;
    

    while (ros::ok())
    {
	if(count < 50)
	    ref=curr_pose;

	err_lin = LinearErr(curr_pose, ref);
	err_ang = AngularErr(curr_pose, ref);
	
	if(fabs(err_ang) > (M_PI*6)/7)
	    twist.angular.z = 0.1;

	    
	err_lin_old += err_lin;
        err_ang_old += err_ang;
        
        if(err_lin > 0.2)
        {
	    if (err_ang > 0.05 || err_ang < -0.05)
	    {
            twist.linear.x = 0;
            twist.angular.z = kp2*sin(err_ang) + ki2*err_ang_old;
	    }
	    else
	    {
	    twist.linear.x = kp1*err_lin + ki1*err_lin_old;
            twist.angular.z = 0;
	    }
	}
        else
        {
	    twist.linear.x = 0;
            twist.angular.z = 0;
        }
	
	
        
	
// 	if(err_lin > 0.005)
// 	{
// 	    if(abs(err_ang) > 0.01)
// 	    {
// 		twist.angular.z = ka*err_ang;
// 		twist.linear.x = 0;
// 	    }
// 	    else
// 	    {
// 		twist.angular.z = 0;
// 		twist.linear.x = kl*err_lin;
// 	    }
// 	}
// 	else
// 	{
// 	    twist.angular.z = 0.0; 
// 	    twist.linear.x = 0.0;
// 	}

	ROS_INFO_STREAM("angular error: " << err_ang*180/M_PI);
	ROS_INFO_STREAM("linear error: " << err_lin);
	
	controller_pub.publish(twist);

	ros::spinOnce();
	loop_rate.sleep();
	++count;
	}
}

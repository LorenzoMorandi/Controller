#include "controller_class.h"

controller_class::controller_class()
{
    
}

controller_class::~controller_class()
{
    
}

void controller_class::ReadCurPos(const turtlesim::Pose::ConstPtr& msg)
{
    curr_pose = *msg;
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

void controller_class::SetRef(turtlesim::Pose center, double radius, int vertex_number)
{
//     vertex.resize(vertex_number);
    vertex_count=0;
    
    turtlesim::Pose tmp;
    
    for(int i=0; i<vertex_number; i++)
    {
	tmp.x=center.x+radius*cos(M_PI/vertex_number+i*2*M_PI/vertex_number);
	tmp.y=center.y+radius*sin(M_PI/vertex_number+i*2*M_PI/vertex_number);
	vertex.push_back(tmp);
    }
    ref=vertex.at(vertex_count);
}

void controller_class::init()
{
    current_pose_sub = n.subscribe("pose", 1, &controller_class::ReadCurPos, this);
    controller_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    ref=curr_pose;
    
    turtlesim::Pose center;
    center.x=5;
    center.y=5;
   
    SetRef(center, 2, 8);
}

void controller_class::run()
{
    ros::Rate loop_rate(100);
    
    int count = 0;
    

    while (ros::ok())
    {

	err_lin = LinearErr(curr_pose, ref);
	err_ang = AngularErr(curr_pose, ref);
	      
	err_lin_old += err_lin;
        err_ang_old += err_ang;
        
        if(err_lin > 0.01)
        {
	    if (sin(err_ang) > 0.005 || sin(err_ang) < -0.005)
	    {
		twist.linear.x = 0;
		twist.angular.z = kp2*sin(err_ang) + ki2*sin(err_ang_old);
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
	    err_lin_old=0;
	    err_ang_old=0;
	    ROS_INFO_STREAM(vertex_count);
	    ref=vertex.at(vertex_count);
	    vertex_count++;

	    vertex_count=vertex_count%vertex.size();
        }
	
	if(count%200 == 0)
	{
// 	    ROS_INFO_STREAM("angular error: " << err_ang*180/M_PI << "\tlinear error: " << err_lin);
// 	    ROS_INFO_STREAM("Curr: (" << curr_pose.x << " " << curr_pose.y << ")");
// 	    ROS_WARN_STREAM("Ref: (" << ref.x << " " << ref.y << ")");
	    count = 0;
	}
	
	controller_pub.publish(twist);

	ros::spinOnce();
	loop_rate.sleep();
	++count;
    }
}

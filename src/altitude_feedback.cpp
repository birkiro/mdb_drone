/*
Birkir Oskarsson & Ilja Fursov
University of Southern Denmark
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_neg;
geometry_msgs::Twist twist_msg_pshover;
geometry_msgs::Twist twist_msg_up;
std_msgs::Empty emp_msg;

double vx 		= 0.0;
double vy 		= 0.0;
double vz 		= 0.0;
double az		= 0.0;
double altd_old = 0.0;
double altd	= 0.0;
double T		= 0.0;
double dt		= 0.0;
float takeoff_time	= 5.0;
float fly_time		= 12.0;
float land_time		= 3.0;
float kill_time 		= 4.0;	
			
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	// Save Drone states to use in feedback controller
	
	vx = msg_in.vx * 0.001;
	vy = msg_in.vy * 0.001;	
	vz = msg_in.vz * 0.001;
	az = msg_in.az * 10;
	altd = msg_in.altd * 0.001;
	
	// For debugging
//	ROS_INFO("Sensor readings: vx: %f - vy: %f - vz: %f - altd: %f", vx, vy, vz, altd);	
}

geometry_msgs::Twist altitude_controller(double altd_des, double vx_des, double vy_des, double K)
{
	geometry_msgs::Twist twist_msg_gen;

	double altd_deriv, vx_pub, vy_pub, z_estimate, z_dot_input;
	
	z_estimate = altd_old + vz * dt + (az - 9.81) * 0.5 * dt*dt;
	
	vx_pub 		= 0.4 * (vx_des - vx);
	vy_pub		= 0.4 * (vy_des - vy);
	if (vx_pub > 1) vx_pub = 1;
	if (vx_pub < -1) vx_pub = -1;
	if (vy_pub > 1) vy_pub = 1;
	if (vy_pub < -1) vy_pub = -1;

	z_dot_input = K * (altd_des - z_estimate);
	if (z_dot_input > 1) z_dot_input = 1;
	if (z_dot_input < -1) z_dot_input = -1;

	twist_msg_gen.linear.x = vx_pub;
	twist_msg_gen.linear.y  = vy_pub;
	twist_msg_gen.linear.z  = z_dot_input;
	twist_msg_gen.angular.x = 1.0; 
	twist_msg_gen.angular.y = 1.0;
	twist_msg_gen.angular.z = 0.0;

	ROS_INFO("vz - %f",twist_msg_gen.linear.z );	

	return twist_msg_gen;
}

void setupHover()
{
	twist_msg_hover.linear.x=0.0; 
	twist_msg_hover.linear.y=0.0;
	twist_msg_hover.linear.z=0.0;
	twist_msg_hover.angular.x=0.0; 
	twist_msg_hover.angular.y=0.0;
	twist_msg_hover.angular.z=0.0; 
}
void setupTakeoff()
{
	twist_msg_up.linear.x=0.0; 
	twist_msg_up.linear.y=0.0;
	twist_msg_up.linear.z=0.5;
	twist_msg_up.angular.x=0.0; 
	twist_msg_up.angular.y=0.0;
	twist_msg_up.angular.z=0.0;
}
void setupFlyCommands()
{
	twist_msg.linear.x=0.0; 
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0; 
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;
}

int main(int argc, char** argv)
{
	printf("Manual Test Node Starting");
	ros::init(argc, argv,"ARDrone_manual_test");
	ros::NodeHandle node;
	ros::Rate loop_rate(50);

	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_empty_reset;
	ros::Subscriber nav_sub;
	double start_time;

 	setupHover();
	setupTakeoff();
	setupFlyCommands();

	nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);	
  	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); 
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1);
	
	start_time =(double)ros::Time::now().toSec();	
	ROS_INFO("Altitude Controller - Birkir & Ilja");

	double altd_des = 3; // [m]
	double K = 0.3; 	 // Up for discussion / Testing / Comparing with Izzet and Forrest Sim
	double t_old;
	T = (double)ros::Time::now().toSec();

	while (ros::ok()) 
	{
		nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
		ROS_INFO("\nVx - %f    Vy - %f   Vz - %f",vx, vy);
		while ((double)ros::Time::now().toSec() < start_time + takeoff_time)
		{ //takeoff
			pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat
			
			ROS_INFO("Taking off");
			ros::spinOnce();
			loop_rate.sleep();
		}//while takeoff

		while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time)
		{
			pub_twist.publish(twist_msg_hover); //drone is flat
			ROS_INFO("Landing");

			if ((double)ros::Time::now().toSec() > takeoff_time + start_time + fly_time + land_time + kill_time)
			{
				pub_empty_land.publish(emp_msg); //lands the drone
				ROS_INFO("Closing Node");
				exit(0); 	
			}
			ros::spinOnce();
			loop_rate.sleep();			
		}
		
		nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
		T = (double)ros::Time::now().toSec();
		while ( (double)ros::Time::now().toSec() > start_time+takeoff_time && (double)ros::Time::now().toSec() < start_time + takeoff_time + fly_time/2)
		{	
			altd_old = altd;
			nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
//			ROS_INFO("\nVx - %f    Vy - %f   Vz - %f",vx, vy);


			t_old =	T;
			T = (double)ros::Time::now().toSec();
			dt = T - t_old;

			twist_msg = altitude_controller(altd_des, 0, 0, K);
			pub_twist.publish(twist_msg);
			
			ros::spinOnce();	
		}
		while ( (double)ros::Time::now().toSec() > start_time+takeoff_time + fly_time/2 && (double)ros::Time::now().toSec() < start_time + takeoff_time + fly_time)
		{	
			altd_old = altd;
			nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
//			ROS_INFO("\nVx - %f    Vy - %f   Vz - %f",vx, vy);

			t_old =	T;
			T = (double)ros::Time::now().toSec();
			dt = T - t_old;

			twist_msg = altitude_controller(0.3, 0, 0, 0.3);
			pub_twist.publish(twist_msg);
			
			ros::spinOnce();	
		}
		ros::spinOnce();
//		loop_rate.sleep();
	}
}

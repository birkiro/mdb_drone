/*
Birkir Oskarsson & Ilja Fursov
University of Southern Denmark
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
//#include "std_msgs/Int32.h"
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include "std_msgs/Float32.h"

#include <sstream>

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
double altd	= 0.0;

double z_hat_old	= 0.0;
double T		= 0.0;
double T_old		= 0.0;


float takeoff_time	= 10.0;
float fly_time		= 15.0;
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

double altitude_observer(double L, double Ts){
	double z_hat = 0.0;
	
	z_hat = z_hat_old + Ts*(vz + L*(altd - z_hat_old));
	z_hat_old = z_hat;
	return z_hat;
}

geometry_msgs::Twist altitude_controller(double z_des, double Kp, double z_hat){
	geometry_msgs::Twist twist_msg_gen;

	double vx_pub, vy_pub, z_input;

	vx_pub 		= 0.4 * (0 - vx);			//leave for now(changes required)
	vy_pub		= 0.4 * (0 - vy);
	if (vx_pub > 1) vx_pub  =  1;
	if (vx_pub < -1) vx_pub = -1;
	if (vy_pub > 1) vy_pub  =  1;
	if (vy_pub < -1) vy_pub = -1;

	z_input = Kp*(z_des - altd);				//P controller using measurement(CHANGE altd -> z_hat in future)
	if (z_input > 1) z_input  =  1;
	if (z_input < -1) z_input = -1;

	twist_msg_gen.linear.x = vx_pub;
	twist_msg_gen.linear.y  = vy_pub;
	twist_msg_gen.linear.z  = z_input;
	twist_msg_gen.angular.x = 1.0; 
	twist_msg_gen.angular.y = 1.0;
	twist_msg_gen.angular.z = 0.0;
	

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
double get_sampl_time(double T_now, double T_old){
	double Ts = 0.0;
	Ts = T_now - T_old;
	T_old = T_now;
	return Ts;
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
	ros::Publisher pub_z_hat;
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
	
	pub_z_hat = node.advertise<std_msgs::Float32>("z_hat", 1);  // #include "std_msgs/Float32.h"
//------------------------------------------------------------------------------------------------------------------------------//	
	start_time =(double)ros::Time::now().toSec();
		
	ROS_INFO("Altitude Controller - Birkir & Ilja");

	double altd_des = 1; // [m]
	double Kp = 0.9; 	 // Up for discussion / Testing / Comparing with Izzet and Forrest Sim
	double  L  = 5;
	double Ts = 0.0;
	double z_hat = 0.0;
	T_old = start_time;

	while (ros::ok()) 
	{
		std_msgs::Float32 msg_z_hat;   // Create a message for the publisher. See documentation for std_msgs for possible data types
//TAKING_OFF & LANDING -------------------------------------------------------------------
		while ((double)ros::Time::now().toSec() < start_time + takeoff_time)
		{ //takeoff
			T_old =	T;
			nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
			T = (double)ros::Time::now().toSec();
			Ts = T - T_old;
			z_hat = altitude_observer(L, Ts);
			
			pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat
			msg_z_hat.data = z_hat*1000; 		// Fill message with data
			pub_z_hat.publish(msg_z_hat); 	// publish data. Try it with "rostopic echo /z_hat" when drone is connected

			
			ROS_INFO("Taking off");
			ros::spinOnce();
			loop_rate.sleep();
		}//while takeoff

		while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time)
		{
			T_old =	T;
			nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
			T = (double)ros::Time::now().toSec();
			Ts = T - T_old;
			
			z_hat = altitude_observer(L, Ts);
			
			pub_twist.publish(twist_msg_hover); //drone is flat
			msg_z_hat.data = z_hat*1000; 		// Fill message with data
			pub_z_hat.publish(msg_z_hat); 	// publish data. Try it with "rostopic echo /z_hat" when drone is connected

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
//--------------------------------------------------------------------------------------		

		while ( (double)ros::Time::now().toSec() > start_time+takeoff_time && (double)ros::Time::now().toSec() < start_time + takeoff_time + fly_time)
		{	
			T_old =	T;
			nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
//			ROS_INFO("\nVx - %f    Vy - %f   Vz - %f",vx, vy);

			T = (double)ros::Time::now().toSec();
			Ts = T - T_old;

			z_hat = altitude_observer(L, Ts);
			
			twist_msg = altitude_controller(altd_des, Kp, z_hat);
			ROS_INFO("%f : %f     : %f", z_hat, altd, Ts);
			pub_twist.publish(twist_msg);
			msg_z_hat.data = z_hat*1000; 		// Fill message with data
			pub_z_hat.publish(msg_z_hat); 	// publish data. Try it with "rostopic echo /z_hat" when drone is connected
			ros::spinOnce();	
		}
		
		ros::spinOnce();
//		loop_rate.sleep();
	}
}

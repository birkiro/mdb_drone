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

double vx 			= 0.0;
double vy 			= 0.0;
double vz 			= 0.0;
double altd		= 0.0;
float takeoff_time	= 8.0;
float fly_time		= 3.0;
float land_time		= 3.0;
float kill_time 	= 4.0;	
double altd_error	= 0.0;			
			
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	// Save Drone states to use in feedback controller	
	vx = msg_in.vx;
	vy = msg_in.vy;	
	vz = msg_in.vz;
	altd = msg_in.altd;	
	
	// For debugging
	//ROS_INFO("Sensor readings: vx: %f - vy: %f - vz: %f - altd: %f", vx, vy, vz, altd);	
}

geometry_msgs::Twist altitude_controller(double altd_des, double K)
{
	geometry_msgs::Twist twist_msg_gen;

	altd_error = altd_des - (K * altd); // Altitude then needs to be changed to velocity
	
	
	twist_msg_gen.linear.x = 
	ROS_INFO("(altd_des - altd) &f", (altd_des - altd));
	twist_msg_gen.linear.y  = 0.0; 
	twist_msg_gen.linear.z  = 0.0;
	//twist_msg_gen.angular.x = 0.0; 
	//twist_msg_gen.angular.y = 0.0;
	//twist_msg_gen.angular.z = 0.0;
	return twist_msg_gen;
}

void setupHover()
{
	twist_msg_hover.linear.x	= 0.0; 
	twist_msg_hover.linear.y	= 0.0;
	twist_msg_hover.linear.z	= 0.0;
	twist_msg_hover.angular.x	= 1.0; // Set to 1 to disable auto-hover
	twist_msg_hover.angular.y	= 1.0; // Set to 1 to disable auto-hover
	twist_msg_hover.angular.z	= 0.0; 
}
void setupTakeoff()
{
	twist_msg_up.linear.x	= 0.0; 
	twist_msg_up.linear.y	= 0.0;
	twist_msg_up.linear.z	= 0.5;
	twist_msg_up.angular.x	= 0.0; 
	twist_msg_up.angular.y	= 0.0;
	twist_msg_up.angular.z	= 0.0;
}
void setupFlyCommands()
{
	twist_msg.linear.x	= 0.0; 
	twist_msg.linear.y	= 0.0;
	twist_msg.linear.z	= 0.0;
	twist_msg.angular.x	= 0.0; 
	twist_msg.angular.y	= 0.0;
	twist_msg.angular.z	= 0.0;
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

	double altd_des = 1500; // [mm]
	double K = 0.75; 	 // Up for discussion / Testing / Comparing with Izzet and Forrest Sim

	while (ros::ok()) 
	{
		//while ((double)ros::Time::now().toSec() < start_time + takeoff_time)
		//{ //takeoff
			//pub_empty_takeoff.publish(emp_msg); //launches the drone
			//pub_twist.publish(twist_msg_hover); //drone is flat
			//ROS_INFO("Taking off");
			//ros::spinOnce();
			//loop_rate.sleep();
		//}//while takeoff

		//while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time)
		//{
			//pub_twist.publish(twist_msg_hover); //drone is flat
			//ROS_INFO("Landing");

			//if ((double)ros::Time::now().toSec() > takeoff_time + start_time + fly_time + land_time + kill_time)
			//{
				//pub_empty_land.publish(emp_msg); //lands the drone
				//ROS_INFO("Closing Node");
				//exit(0); 	
			//}
			//ros::spinOnce();
			//loop_rate.sleep();			
		//}

		while ( (double)ros::Time::now().toSec() > start_time+takeoff_time && (double)ros::Time::now().toSec() < start_time + takeoff_time + fly_time)
		{	
			// Fill in here Ilja, and probably work on the while loop timings
			// We need to get current altitude from the callback function (I guess)
			// The controller could be something like this:
			//
			// Read altd;

			// ardrone_autonomy/Navdata publishes on /ardrone/navdata
			nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
			
			//ROS_INFO("Sensor readings: vx: %f - vy: %f - vz: %f - altd: %f", vx, vy, vz, altd);	
			// Run altitude_controller with altd_des and K:
			twist_msg = altitude_controller(altd_des, K);
			// Make a variable or something to hold the old altd value
			// altd_old = altd;

			// Maybe we can figure out something better than these stupid while loops
			// I don't like them. There are 100 better ways to code this. Put on your thinking hat :)
	
			ros::spinOnce();
			//loop_rate.sleep();
		}
		ros::spinOnce();
		//loop_rate.sleep();
	}
}

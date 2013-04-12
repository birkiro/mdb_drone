/*
Birkir Oskarsson
University of Southern Denmark

Takeoff routine for the AR. Drone
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
std_msgs::Empty emp_msg;	
int main(int argc, char** argv)
{
	ROS_INFO("Flying ARdrone");
	ros::init(argc, argv,"ARDrone_test");
    	ros::NodeHandle node;
    	ros::Rate loop_rate(50);
	ros::Publisher pub_empty;
	ros::Publisher pub_empty_land;
	pub_empty = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
 	while (ros::ok()) 
	{
		double time_start=(double)ros::Time::now().toSec();
		while ((double)ros::Time::now().toSec() < time_start + 5.0) /* Send command for five seconds*/
		{ 
			pub_empty.publish(emp_msg); /* launches the drone */
			ros::spinOnce();
			loop_rate.sleep();
		}//time loop

		ROS_INFO("We Have Liftoff");
		exit(0);
	}//ros::ok loop
	
	pub_empty_land.publish(emp_msg); /* lands the drone */
	ros::spinOnce();
	loop_rate.sleep();
}//main

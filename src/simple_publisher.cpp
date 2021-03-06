/*
Birkir Oskarsson
University of Southern Denmark
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <sstream>
#include <ardrone_autonomy/Navdata.h>


double vx = 0.0;

// nav_callback: This function is called every time a message is published
//		 and the subscriber object picks it up. We can use the callback
// 		 function to store the data and make sense of it and use it
//		 in our state feedback controller.
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	// Take in state of ardrone	
	vx = msg_in.vx;
	
	// Write it to the terminal
	
}
			
int main(int argc, char** argv)
{
	printf("Simple Subscriber Starting");
	ros::init(argc, argv,"Subscriber_Tester"); // can be seen using rxgraph or rqt_graph
	ros::NodeHandle node;
	ros::Subscriber nav_sub;	

	while (ros::ok()) 
	{
		// ardrone_autonomy/Navdata publishes on /ardrone/navdata
		nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);	
		ROS_INFO("getting sensor reading: %f [mm/s]", vx);	
		ros::spinOnce(); 	// receive published messages constantly
	}
	return 0; 	
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}



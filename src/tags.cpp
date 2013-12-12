/*
Birkir Oskarsson
University of Southern Denmark
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

uint32_t tags_count;
uint32_t tags_xc[10];
uint32_t tags_yc[10];
uint32_t tags_distance[10];
uint32_t tags_orientation[10];
// nav_callback: This function is called every time a message is published
//		 and the subscriber object picks it up. We can use the callback
// 		 function to store the data and make sense of it and use it
//		 in our state feedback controller.
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	// Take in state of ardrone	
	//vx = msg_in.vx;
	tags_count=msg_in.tags_count;
	// Write it to the terminal
	for (uint32_t i=0; i < tags_count; i++)
    {
		tags_xc[i] = msg_in.tags_xc[i];
		tags_yc[i] = msg_in.tags_yc[i];
		tags_distance[i] = msg_in.tags_distance[i];
		tags_orientation[i] = msg_in.tags_orientation[i];
    }
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
		for (uint32_t j=0; j<tags_count; j++){
			printf("tags_xc: %d => tags_yc: %d => tags_distance: %d => tags_orientation: %d\n", tags_xc[j], tags_yc[j], tags_distance[j], tags_orientation[j]);
		}
		
	
		ros::spinOnce(); 	// receive published messages constantly
	}
	return 0; 	
}



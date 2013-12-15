/*
Birkir Oskarsson
University of Southern Denmark
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

float tagx = 0; 
float angZ;
uint32_t tags_count;
uint32_t tags_xc[10];
// nav_callback: This function is called every time a message is published
//		 and the subscriber object picks it up. We can use the callback
// 		 function to store the data and make sense of it and use it
//		 in our state feedback controller.
void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	/*
	// Take in state of ardrone	
	printf("vector: %f\n", msg_in.tags_xc.size());
	//vx = msg_in.vx;
	tags_count=msg_in.tags_count;
	// Write it to the terminal
	for (uint32_t i=0; i < tags_count; i++)
    {
		tags_xc[i]=msg_in.tags_xc[i];
    }
    * */
    angZ = msg_in.rotZ;
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
		printf("angle: %f\n", angZ);	
		//ROS_INFO("getting sensor reading: %f [mm/s]", vx);
		//ROS_INFO("Tag Distance: X: %f", tagx/*, msg_in.tags_yc, msg_in.tags_distance*/);	
		ros::spinOnce(); 	// receive published messages constantly
	}
	return 0; 	
}



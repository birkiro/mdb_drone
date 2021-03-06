/*
Birkir Oskarsson
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

int32_t tags_count;
int32_t tags_xc[10];
int32_t tags_yc[10];
int32_t tags_distance[10];
int32_t tags_orientation[10];
float alt;
float angY, angX;
const int32_t xc_des  		= 500;
const int32_t yc_des  		= 500;
const int32_t tagDist_des  	= 150; // cm
const int32_t tagOrient_des  	= 90;

double takeoff_time	= 5.0;
double fly_time		= 50.0;
double land_time	= 2.0;
double kill_time 	= 3.0;	
double Tnow		= 0;
double Told		= 0;

float tiltY, tiltX;

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
		tags_yc[i] = msg_in.tags_xc[i];
		tags_xc[i] = msg_in.tags_yc[i];
		tags_distance[i] = msg_in.tags_distance[i];
		tags_orientation[i] = msg_in.tags_orientation[i];
   	 }
	alt = msg_in.altd;
	angY = msg_in.rotX;
	angX = msg_in.rotY;
}

geometry_msgs::Twist tag_controller(float Kx, float Ky, float Ki, float Kz, float Kangle)
{
	geometry_msgs::Twist twist_msg_gen;
	
	float vx_pub, vy_pub, vz_pub, yaw_pub;
	if(tags_count){
		yaw_pub = -Kangle*(tagOrient_des - tags_orientation[0]);
		//vz_pub = Kz*(tagDist_des - tags_distance[0]);
		vz_pub = Kz*(1850 - alt);;
		tiltX = tags_xc[0] - 185*sin(angX);
		tiltY = tags_yc[0] - 185*sin(angY);
		vy_pub = Ky*(yc_des - tiltY) + Ki*(yc_des - tiltY)*(Tnow - Told);					// tag detection X parameter is Y axis of velocity
		vx_pub = Kx*(xc_des - tiltX) + Ki*(xc_des - tiltX)*(Tnow - Told);					// tag detection Y parameter is X axis of velocity
		if (yaw_pub > 0.3) yaw_pub = 0.3;
		if (yaw_pub < -0.3) yaw_pub = -0.3;
		if (vz_pub > 0.3) vz_pub = 0.3;
		if (vz_pub < -0.3) vz_pub = -0.3;
		if (vy_pub > 0.15) vy_pub = 0.15;
		if (vy_pub < -0.15) vy_pub = -0.15;
		if (vx_pub > 0.15) vx_pub = 0.15;
		if (vx_pub < -0.15) vx_pub = -0.15;

		twist_msg_gen.linear.x  = vx_pub;
		twist_msg_gen.linear.y  = vy_pub;
		twist_msg_gen.linear.z  = vz_pub;
		twist_msg_gen.angular.x = 1.0; 
		twist_msg_gen.angular.y = 1.0;
		twist_msg_gen.angular.z = yaw_pub;
		printf("xc:%d,yc:%d,zc:%d,ang:%d,vx:%f,vy:%f,vz:%f,angZ:%f,tiltY:%f,tiltX:%f\n", tags_xc[0],tags_yc[0],tags_distance[0],tags_orientation[0],vx_pub,vy_pub,vz_pub, yaw_pub,tiltY, tiltX);
	}
	else {
		vz_pub = Kz*(1850 - alt);
		if (vz_pub < -0.3) vz_pub = -0.3;
		if (vz_pub > 0.3) vz_pub = 0.3;
		twist_msg_gen.linear.x = 0.0;
		twist_msg_gen.linear.y  = 0.0;
		twist_msg_gen.linear.z  = vz_pub;
		twist_msg_gen.angular.x = 0.0; 		// 0 means auto-hover mode
		twist_msg_gen.angular.y = 0.0;		// 0 means auto-hover mode
		twist_msg_gen.angular.z = 0.0;
	}	
	
	return twist_msg_gen;
}
			
int main(int argc, char** argv)
{
	system ("rosservice call /ardrone/flattrim"); sleep(4);

	ros::init(argc, argv,"Subscriber_Tester"); // can be seen using rxgraph or rqt_graph
	ros::NodeHandle node;
	ros::Subscriber nav_sub;	
	ros::Rate loop_rate(50);
	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_empty_reset;
	
	double start_time;
	float Kx = 0.00006;
	float Ky = 0.00006;
	float Ki = 0.09;
	float Kz = 0.0005;
	float Kangle = 0.025;


	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); 
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); 
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1);

	start_time =(double)ros::Time::now().toSec();	
	
	system("rosservice call /ardrone/imu_recalib");	sleep(2);
	while (ros::ok()) 
	{
		// ardrone_autonomy/Navdata publishes on /ardrone/navdata
		nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);	
			/* For Debugging purpose */
						//		for (uint32_t j=0; j<tags_count; j++){
						//			printf("tags_xc: %d => tags_yc: %d => tags_distance: %d => tags_orientation: %d\n",
						//					 tags_xc[j], tags_yc[j], tags_distance[j], tags_orientation[j]);
						//		}

		// Takeoff start
		while ((double)ros::Time::now().toSec() < start_time + takeoff_time)
		{
			pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat

			ros::spinOnce();
			loop_rate.sleep();
		}	//END takeoff

		// Landing Start
		while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time)
		{
			printf("HOVERING\n");
			pub_twist.publish(twist_msg_hover); //drone is flat

			if ((double)ros::Time::now().toSec() > takeoff_time + start_time + fly_time + land_time + kill_time)
			{
				pub_empty_land.publish(emp_msg); //lands the drone
				ROS_INFO("Closing Node");
				exit(0); 	
			}
			ros::spinOnce();
			loop_rate.sleep();			
		}	// END Landing
		
		// Start Fly
		while ( ((double)ros::Time::now().toSec() > start_time+takeoff_time) && ((double)ros::Time::now().toSec() < start_time + takeoff_time + fly_time))
		{	
			if (!Told) {
				Told = (double)ros::Time::now().toSec();
				Tnow = (double)ros::Time::now().toSec();
			}
			else{
				Told = Tnow;
				Tnow = (double)ros::Time::now().toSec();
			}
			// ardrone_autonomy/Navdata publishes on /ardrone/navdata
			nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);

			twist_msg = tag_controller(Kx, Ky, Ki, Kz, Kangle);

			pub_twist.publish(twist_msg);
			
			ros::spinOnce();
			//loop_rate.sleep();
		}	//END Fly
		ros::spinOnce(); 	
		loop_rate.sleep();
	}
	return 0; 	
}



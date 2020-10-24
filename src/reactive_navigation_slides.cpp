#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// Some global variables to "perceive" the world around the robot:
double obstacle_distance;
bool robot_stopped;
//...

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	// Acess the scan ranges array via: msg->ranges[0..size-1]
	// Note that:
	// msg->ranges[0] corresponds to the range with angle_min (msg->angle_min)
	// msg->ranges[size-1] corresponds to the range with angle_max (msg->angle_max)
	// msg->ranges[i] corresponds to the range with angle_min + i*angle_increment (msg->angle_increment)
	// All these angles are in radians!

	if(!robot_stopped){ // only prints when the robot is moving...
		ROS_INFO("Received a LaserScan with %i samples", (int) msg->ranges.size());

		// For simplicity, let's save the distance of the closer obstacle to the robot:
		obstacle_distance = *std::min_element (msg->ranges.begin(), msg->ranges.end());
		ROS_INFO("Minimum distance to the obstacle: %f", obstacle_distance);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reactive_navigation_basic");

	ros::NodeHandle n;

	// Publisher for /cmd_vel
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	// Subscriber for /base_scan
	ros::Subscriber laser_pub = n.subscribe("base_scan", 100, laserCallback);

	ros::Rate loop_rate(10); // 10Hz

	// Initializations:
	geometry_msgs::Twist cmd_vel_msg;
	robot_stopped = true;
	obstacle_distance = 1.0;

	while(ros::ok()){
		// Fill the "cmd_vel_msg" data according to some conditions (depending on laser data)

		if(obstacle_distance > 0.5){
			// Move forward:
			cmd_vel_msg.linear.x = 1.0;
			cmd_vel_msg.angular.z = 0.0;

			if(robot_stopped){ // Just print once:
				ROS_INFO("Moving Forward");
				robot_stopped =false;
			}
		}else{ // Stop
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = 0.0;

			if(!robot_stopped){ // Just print once
				ROS_INFO("Stopping");
				robot_stopped = true;
			}
		}

		cmd_vel_pub.publish(cmd_vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;

}
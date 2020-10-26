#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <random>


class ReactiveController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

    double obstacle_distance;
    // double* ranges;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();

        // srand(time(NULL));

        if(obstacle_distance > 0.5) msg.linear.x = 1;

		if(obstacle_distance <= 0.5)
		{
			msg.angular.z = -1;

			// float rand_time = ((float)rand() / RAND_MAX) * (1.0 - 0.1) + 0.1;
        	// ROS_INFO("Random time: %f", rand_time);

        	// Wait this long while rotating and hope we're clear
			ros::Duration(0.1).sleep();
		}		

        return msg;
    }


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        // std::vector<float> laser_ranges = msg->ranges;
        // ranges = &laser_ranges[0];
        ROS_INFO("Min distance to obstacle: %f", obstacle_distance);
    }


public:
    ReactiveController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a subscriber for laser scans 
        this->laser_sub = n.subscribe("base_scan", 10, &ReactiveController::laserCallback, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "reactive_controller");

    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();

    // And make good on our promise
    return 0;
}
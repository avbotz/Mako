#include <ros/ros.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_node");
	ros::NodeHandle node;

	ros::ServiceServer service = node.advertiseService()
}

#include <ros/ros.h>
#include <control/ControlAlive.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "restart_node");
	ros::NodeHandle node;   
	
	// Setup control clients.
	control::ControlAlive srv;
	ros::ServiceClient client = 
		node.serviceClient<control::ControlAlive>("control_alive");

	// Wait until kill switch is flipped.
	bool prev = false;
	while (ros::ok())
	{
		client.call(srv);
		bool alive = srv.response.data;
		
		// Previous state was killed, now unkilled. Start mission.
		if (!prev && alive)
		{
			ROS_INFO("Beginning mission node.");
			system("rosrun mission mission_node &");
		}

		// Previous state was alive, now killed. End mission.
		else if (prev && !alive)
		{
			ROS_INFO("Ending mission node.");
			system("rosnode kill mission_node");
		}

		prev = alive;
		ros::Duration(0.5).sleep();
	}
}

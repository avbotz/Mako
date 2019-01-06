#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vision/Observation.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_mock_detect");
    ros::NodeHandle node;   
    ros::ServiceClient client = node.serviceClient<vision::Observation>("observation");    
    
    vision::Observation srv;
    srv.request.task = 0;
    if (client.call(srv))
    {
        ROS_INFO("%f %f", srv.response.x, srv.response.y);
    }
}

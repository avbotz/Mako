#include "ros/ros.h"
#include "std_msgs/String.h"

#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <sstream>

#include "control/atmega.hpp"
#include "control/state.hpp"

const int REMOTE_PORT = 8080;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "remote");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("wiimote", 1000);
    ros::Rate loop_rate(10);

    int server_fd, new_socket, valread; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons(REMOTE_PORT); 
    if (bind(server_fd, (struct sockaddr *)&address,  
                sizeof(address))<0) 
    { 
        perror("bind failed"); 
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *) &address, (socklen_t*) &addrlen)) < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    char buffer[1024] = {0};
    while (true) 
    {
        valread = read(new_socket, buffer, 1024);
        if (buffer[18] == '1') break;
        ROS_INFO("Command: %s", buffer);
        atmega::write(buffer);
    }
}

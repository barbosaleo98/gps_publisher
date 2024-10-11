#ifndef GPS_LISTENER_H
#define GPS_LISTENER_H

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include "geometry_msgs/msg/point.hpp"

namespace gps_publisher
{
class GPSListener : public rclcpp::Node
{
public:
    GPSListener(const rclcpp::NodeOptions &options);
    void publish(const geometry_msgs::msg::Point point_msg);
    

private:        
    void initialize();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;

    std::vector<std::string> splitByDelimiter(const char* charArray, char delimiter);
    bool isChecksumValid(const char* charArray);
    double convert2Degrees(const std::string &value, std::string direction);

};

} // namespace gps_publisher
#endif // GPS_LISTENER_H

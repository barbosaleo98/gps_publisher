#include <string>
#include "gps_listener.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#define SENDER_IP "172.16.2.1"
#define UDP_PORT 5000

namespace gps_publisher
{
GPSListener::GPSListener(const rclcpp::NodeOptions &options)
    : Node("gps_publisher", options)
{
    RCLCPP_INFO(this->get_logger(), "GPSListener constructor start");
    initialize();
    RCLCPP_INFO(this->get_logger(), "GPSListener constructor end");
}

void GPSListener::initialize()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GPS Listener initialized");
    
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error creating UDP socket");
        return;
    }

    struct sockaddr_in host_addr, sender_addr;
    host_addr.sin_family = AF_INET;
    host_addr.sin_port = htons(UDP_PORT);  // host_port
    host_addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(sockfd, (struct sockaddr *)&host_addr, sizeof(host_addr)) == -1) {
        close(sockfd);
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error binding socket");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized and starting to receive UDP packets in port %d", UDP_PORT);

    char buffer[1024];
    socklen_t addr_len = sizeof(sender_addr);

    while (true) {
        ssize_t recv_len = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&sender_addr, &addr_len);

        if (recv_len < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error receiving data");
            close(sockfd);
            break;
        }

        if (sender_addr.sin_addr.s_addr == inet_addr(SENDER_IP)) {

            char senderIPAddr[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(sender_addr.sin_addr.s_addr ), senderIPAddr, INET_ADDRSTRLEN);
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Obtained buffer from IP (%s): \n %s", senderIPAddr, buffer);

            for (std::string sentence : GPSListener::splitByDelimiter(buffer, '$')){

                if (sentence.find("GPRMC") != std::string::npos || sentence.find("GPGGA") != std::string::npos){

                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sentence: $%s", sentence.c_str());

                    std::tuple<std::string, std::string> lat, lon;
                    
                    std::vector sentenceData = GPSListener::splitByDelimiter(sentence.c_str(), ',');

                    for(int i = 0; i < int(sentenceData.size()); i++){

                        if(sentenceData[i] == "N" || sentenceData[i] == "S"){
                            lat = std::make_tuple(sentenceData[i-1], sentenceData[i]);

                        }else if(sentenceData[i] == "E" || sentenceData[i] == "W"){
                            lon = std::make_tuple(sentenceData[i-1], sentenceData[i]);
                        }
                    }

                    auto point_msg = geometry_msgs::msg::Point(); 

                    point_msg.x = GPSListener::convert2Degrees(std::get<0>(lat), std::get<1>(lat));
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtained lat: %f", point_msg.x);

                    point_msg.y = GPSListener::convert2Degrees(std::get<0>(lon), std::get<1>(lon));
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Obtained lon: %f", point_msg.y);

                    publish(point_msg);

                }
            }

        }
    }
 
}

std::vector<std::string> GPSListener::splitByDelimiter(const char* charArray, char delimiter){
    
    std::vector<std::string> result;
    std::string token;

    for(int i = 0; charArray[i] != '\0'; i++){
        if(charArray[i] == delimiter){
            if(!token.empty()){
                result.push_back(token);
                token.clear();
            }
        }else{
            token += charArray[i];
        }
    }

    if(!token.empty()){
        result.push_back(token);
    }
    
    return result;
}

double GPSListener::convert2Degrees(const std::string &value, std::string direction){
    double degrees = std::stoi(value) / 100;
    double minutes = std::stod(value) - (degrees * 100);
    double result = degrees + (minutes / 60);

    if(direction == "S" || direction == "W"){
        return -result;
    }

    return result;
}

void GPSListener::publish(const geometry_msgs::msg::Point point_msg)
{
    node_ = std::make_shared<rclcpp::Node>("gps_listener_node");
    publisher_ = node_->create_publisher<geometry_msgs::msg::Point>("gps_coordinates", 10);

    publisher_->publish(point_msg);
}

} // namespace gps_publisher

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting GPS Listener Node");

  rclcpp::spin(std::make_shared<gps_publisher::GPSListener>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

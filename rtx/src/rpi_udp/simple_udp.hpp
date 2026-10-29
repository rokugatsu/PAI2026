#ifndef _ROS2_UDP_HPP_
#define _ROS2_UDP_HPP_
/*
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring>

class Ros2UDP{
    public:
        //コンストラクタ
        Ros2UDP(const std::string& f7_address, int f7_port);

        //ソケットにアドレスをバインド(関連付け)
        void udp_bind();

        //パケットを送信
        void send_packet(uint8_t *packet, uint8_t size);

        //受信する
        ssize_t udp_recv(uint8_t *buf, uint8_t size);
    private:
        int sock;
        struct sockaddr_in f7_addr;
};
*/
#include <rclcpp/rclcpp.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include "robo_msgs/msg/traffic_light.hpp"

class simple_udp: public rclcpp::Node{
	int sock;
	struct sockaddr_in addr;
    
    public:
	// simple_udp(std::string address, int port){
	// 	sock = socket(AF_INET, SOCK_DGRAM, 0);
	// 	addr.sin_family = AF_INET;
	// 	addr.sin_addr.s_addr = inet_addr(address.c_str());
	// 	addr.sin_port = htons(port);		
	// }
    
    simple_udp();
	
    void udp_send(std::string word){
		sendto(sock, word.c_str(), word.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
	}
	
	void udp_bind(){
		bind(sock, (const struct sockaddr *)&addr, sizeof(addr));

	}
	std::string udp_recv(){
		#define BUFFER_MAX 400
		char buf[BUFFER_MAX];
		memset(buf, 0, sizeof(buf));
		recv(sock, buf, sizeof(buf), 0);
		return std::string(buf);
	}
	void udp_recv(char *buf, int size){
		memset(buf, 0, size);
		recv(sock, buf, size, 0);
	}
	
	~simple_udp(){
		close(sock);
	}

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robo_msgs::msg::TrafficLight>::SharedPtr pub_TrafficLight_;
    robo_msgs::msg::TrafficLight trafficLight_;
};


#endif
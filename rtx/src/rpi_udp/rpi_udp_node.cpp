/*
#include "ros2_udp.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <sys/socket.h>
#include <string>
#include <cstring>
#include <stdio.h>

#define BUFFER_SIZE 20
#define ANY_ADDR "192.168.0.10"
//#define F7_PORT 4001
#define F7_PORT 60000

class ReceiveUDP : public rclcpp::Node{
	public:
		ReceiveUDP(const std::string& f7_address, int f7_port)
			:Node("receive_udp"),f7_udp(f7_address,f7_port){
				f7_udp.udp_bind();
                while(1){receive_data();}//無限ループして受信処理
			}
        void receive_data(){
            uint8_t buf[BUFFER_SIZE] = {0};
            ssize_t received_length = f7_udp.udp_recv(buf, sizeof(buf));
            //ssize_t received_length = 0;
            printf("receive_data ---- \n");
            if(received_length > 0){
                //RCLCPP_INFO(this->get_logger(), "Received Data = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15],buf[16],buf[17],buf[18],buf[19]);
                printf("Received Data = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15],buf[16],buf[17],buf[18],buf[19]);
            
            }else{
                //RCLCPP_INFO(this->get_logger(), "Received Data = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15],buf[16],buf[17],buf[18],buf[19]);
                printf("Received Data = %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15],buf[16],buf[17],buf[18],buf[19]);
            }
        }
	private:
        Ros2UDP f7_udp;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ReceiveUDP>(ANY_ADDR, F7_PORT);
    //while(1){node->receive_data();}//無限ループして受信処理
    rclcpp::shutdown();
    return 0;
}
*/
/*
#include <stdio.h>
#include <string.h>
#include "simple_udp.h"
simple_udp udp0("0.0.0.0",4001);

int main(int argc, char **argv){
  udp0.udp_bind();
  while (1){
	std::string rdata=udp0.udp_recv();
	printf("recv:%s\n", rdata.c_str());
  }
  return 0;
}
*/

#include "simple_udp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <sys/socket.h>
#include <string>
#include <cstring>
#include <stdio.h>
#include <chrono>

//#include "robo_msgs/msg/traffic_light.h"

#define BUFFER_SIZE 20
#define ANY_ADDR "0.0.0.0"
//#define F7_PORT 4001
#define F7_PORT 60000

using namespace std::chrono_literals;
using std::placeholders::_1;

//Waypoint_planner::Waypoint_planner(): Node("waypoint_planner_node")
simple_udp::simple_udp(): Node("udp_receiver"){
   
    std::string address = ANY_ADDR; 
    int port = 60000;
    int d,e;
    printf("simple_udp() \n");

    sock = socket(AF_INET, SOCK_DGRAM, 0);
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(address.c_str());
	addr.sin_port = htons(port);		
        
    udp_bind();
    // while(1){
    //     std::string rdata=udp_recv();
	//     printf("recv:%s\n", rdata.c_str());
    //     //sscanf(rdata.c_str(),"(%d,%d)",&d,&e);
    //     //printf("d=%d,e=%d\n", d,e);
    // }//無限ループして受信処理        
    
    auto receive_routine =
      [this]() -> void  // ラムダ式による関数オブジェクトの定義
    {
        std::string rdata=udp_recv();
	    int greenCount = 0;
        int redCount = 0;

        sscanf(rdata.c_str(),"(%d,%d)",&greenCount,&redCount);
        //printf("recv:%s\n::green=%d,red=%d", rdata.c_str(),greenCount,redCount);

        trafficLight_.green = greenCount;
        trafficLight_.red = redCount;

        pub_TrafficLight_->publish(trafficLight_);
    };

    // publish_messageのXミリ秒周期でのタイマー実行
    timer_ = create_wall_timer(50ms, receive_routine);
    pub_TrafficLight_ = create_publisher<robo_msgs::msg::TrafficLight>("trafficlight", rclcpp::QoS(1).reliable().durability_volatile());
      
};

int main(int argc, char * argv[]){

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc,argv);

    printf("rpi_udp_node:1 \n");
    auto node = std::make_shared<simple_udp>();
    //while(1){node->receive_data();}//無限ループして受信処理

    printf("rpi_udp_node:2 \n");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
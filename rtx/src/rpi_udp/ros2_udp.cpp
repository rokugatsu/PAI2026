/*
#include <cstdint>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include "ros2_udp.hpp"

//コンストラクタ
Ros2UDP::Ros2UDP(const std::string& f7_address, int f7_port){
    //ソケットを作成する。引数:(int domain, int type, int protocol)
    //domain: プロトコルファミリ(今回はAF_INET(IPv4)を選択)
    //type: ソケットのタイプ(今回はUDPを使うため、データグラムソケットを作成)
    //protocol: 使用プロトコルを設定。基本ソケットのタイプに従う0を入力。　
    
    printf("Now constructor \n");
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    f7_addr.sin_family = AF_INET;                               //f7のアドレスのプロトコルファミリを選択
    f7_addr.sin_addr.s_addr = inet_addr(f7_address.c_str());    //f7のアドレスを設定。
    f7_addr.sin_port = htons(f7_port);
}

//ソケットにアドレスをバインド
void Ros2UDP::udp_bind(){
    printf("Now udp_bind \n");
    bind(sock, (const struct sockaddr *)&f7_addr, sizeof(f7_addr));
}

//パケットを送信する
void Ros2UDP::send_packet(uint8_t *packet, uint8_t size){
    sendto(sock, packet, size, 0, (struct sockaddr *)&f7_addr, sizeof(f7_addr));
}

//パケットを受信する
ssize_t Ros2UDP::udp_recv(uint8_t *buf, uint8_t size){
    printf("Now udp_recv \n");
    memset(buf, 0, size);
    return recv(sock, buf, size, 0);
}
*/
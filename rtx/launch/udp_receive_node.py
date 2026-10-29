import rclpy
from rclpy.node import Node
import socket
import struct

# 受信用のIPアドレスとポート番号を設定
UDP_IP = "192.168.0.10"  # すべてのネットワークインターフェースを受け付ける
UDP_PORT = 60000  # 送信元のポート番号と合わせる

# UDPソケットを作成
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))


class UdpReceive(Node):
    def __init__(self):
        super().__init__('udp_receive_node')
        timer_period = 0.01
        self.timer = self.create_timer(timer_period,self.timer_callback)
        
    def timer_callback(self):
        print('xxxx....')

         # データを受信（バッファサイズを指定）
        data, addr = sock.recvfrom(12)  # float型の配列のバッファサイズを指定
        print(addr)

        # 受信したデータを解析
        #self.float_array = struct.unpack('ffff', data)  # float型の配列としてデータを解釈
        self.float_array = struct.unpack('>d', data)  # float型の配列としてデータを解釈
        
        #self.get_logger().info(f'Received Float Array:{self.float_array}')
        #print(f'Received Float Array:{self.float_array}')
        print(self.float_array)
        

def main():
    rclpy.init()
    node = UdpReceive()
    print('udp_receive_node is started...')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# ★ 방금 빌드한 커스텀 서비스 메시지 가져오기
from duckie_interfaces.srv import SetColor 

class LEDServiceBridge(Node):
    def __init__(self):
        super().__init__('duckie_led_bridge')
        
        # 1. Isaac Sim으로 쏘는 Publisher (Topic -> /duckie_led)
        # (Isaac Sim Action Graph가 이 토픽을 듣고 색을 바꿈)
        self.publisher_ = self.create_publisher(Twist, '/duckie_led', 10)
        
        # 2. 터미널 명령을 받는 Service Server (Service -> /duckie_led_control)
        # (사용자가 'red'라고 요청하면 여기서 받음)
        self.srv = self.create_service(SetColor, '/duckie_led_control', self.led_callback)
        
        self.get_logger().info('Ready! Service Server Started: /duckie_led_control')

    def led_callback(self, request, response):
        # request.color에 "red", "green", "blue" 문자열이 들어옵니다.
        color_cmd = request.color.lower()
        msg = Twist()
        
        # Isaac Sim이 알아듣는 RGB 값(Linear XYZ)으로 변환
        # (아까 그래프에서 x=Red, y=Green, z=Blue로 연결했음)
        if color_cmd == 'red':
            msg.linear.x = 1.0
        elif color_cmd == 'green':
            msg.linear.y = 1.0
        elif color_cmd == 'blue':
            msg.linear.z = 1.0
        else:
            # 이상한 색깔이 들어오면 실패 처리
            response.success = False
            response.message = f"Unknown color: {color_cmd}"
            self.get_logger().warn(f"Failed request: {color_cmd}")
            return response

        # Isaac Sim으로 토픽 발사!
        self.publisher_.publish(msg)
        self.get_logger().info(f"Changing LED color to: {color_cmd.upper()}")
        
        # 성공 응답 채우기
        response.success = True
        response.message = f"Successfully changed to {color_cmd}"
        return response

def main():
    rclpy.init()
    node = LEDServiceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 안내 메시지
msg = """
---------------------------
Duckiebot Control (Arrow Keys)
---------------------------
    ↑
 ←  ↓  →

Space: Stop (정지)
CTRL-C to quit
---------------------------
"""

class ArrowTeleop(Node):
    def __init__(self):
        super().__init__('duckie_arrow_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        print(msg)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        # 터미널 설정을 'Raw' 모드로 변경하여 키 입력 즉시 반응
        tty.setraw(sys.stdin.fileno())
        # 한 글자를 읽음
        key = sys.stdin.read(1)
        # 만약 방향키(Escape 시퀀스)라면 뒤에 오는 2글자를 더 읽음
        if key == '\x1b':
            additional = sys.stdin.read(2)
            key += additional
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            # 기본 속도 설정
            target_linear_vel = 0.0
            target_angular_vel = 0.0
            
            while True:
                key = self.get_key()
                
                # 방향키 코드 매핑
                # '\x1b[A': 위쪽 화살표
                # '\x1b[B': 아래쪽 화살표
                # '\x1b[C': 오른쪽 화살표
                # '\x1b[D': 왼쪽 화살표
                
                if key == '\x1b[A':   # UP (전진)
                    target_linear_vel = 0.5
                    target_angular_vel = 0.0
                    print("Forward")
                    
                elif key == '\x1b[B': # DOWN (후진)
                    target_linear_vel = -0.5
                    target_angular_vel = 0.0
                    print("Backward")
                    
                elif key == '\x1b[D': # LEFT (좌회전)
                    # 제자리 회전을 위해 선속도는 줄이거나 0으로
                    target_linear_vel = 0.2 
                    target_angular_vel = 1.0
                    print("Left")
                    
                elif key == '\x1b[C': # RIGHT (우회전)
                    target_linear_vel = 0.2
                    target_angular_vel = -1.0
                    print("Right")
                    
                elif key == ' ':      # SPACE (정지)
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0
                    print("Stop")
                    
                elif key == '\x03':   # Ctrl+C (종료)
                    break

                # 메시지 생성 및 발행
                twist = Twist()
                twist.linear.x = float(target_linear_vel)
                twist.angular.z = float(target_angular_vel)
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            # 종료 시 로봇 멈춤
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            # 터미널 설정 복구
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = ArrowTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

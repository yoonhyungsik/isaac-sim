import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class DuckieChaser(Node):
    def __init__(self):
        super().__init__('duckie_vision_chaser')
        
        # 1. Subscriber: Isaac Sim에서 보내는 토픽 이름 ('/rgb')
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10)
            
        # 2. Publisher: 로봇 이동 명령 ('/cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. CV Bridge: ROS -> OpenCV 변환기
        self.bridge = CvBridge()
        
        # 설정값
        self.target_color_lower = np.array([0, 100, 100])   # 빨간색 (HSV) 하한
        self.target_color_upper = np.array([10, 255, 255])  # 빨간색 (HSV) 상한
        
        # [수정됨] 멈춤 기준 면적 (값이 클수록 더 가까이 다가감)
        self.stop_area_threshold = 80000  
        
        self.get_logger().info("Duckie Chaser Started! Looking for RED objects...")

    def image_callback(self, msg):
        try:
            # A. 이미지 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, _ = cv_image.shape
            
            # B. 이미지 처리 (HSV 변환 및 마스킹)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.target_color_lower, self.target_color_upper)
            
            # C. 객체 검출
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            twist = Twist() # 기본값: 정지
            
            if len(contours) > 0:
                # 가장 큰 덩어리 찾기
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                # 시각화: 박스 그리기
                x, y, bw, bh = cv2.boundingRect(c)
                cv2.rectangle(cv_image, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                
                # 중심점 계산
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 중심점 표시
                    cv2.circle(cv_image, (cx, cy), 5, (255, 0, 0), -1)
                    
                    # --- D. 제어 로직 ---
                    
                    # 1. 조향 (Steering)
                    center_x = w // 2
                    error_x = center_x - cx
                    # 오차만큼 회전 (P제어 계수 0.005)
                    twist.angular.z = 0.005 * error_x 
                    
                    # 2. 속도 (Throttle) - 면적 기반 거리 조절
                    if area < self.stop_area_threshold:
                        twist.linear.x = 0.5  # 멀면 전진 (속도 약간 높임)
                        action = "Forward"
                    else:
                        twist.linear.x = 0.0  # 가까우면 정지
                        action = "Stop (Arrived)"
                        
                    # 상태 텍스트 출력
                    cv2.putText(cv_image, f"Action: {action}", (10, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    
                    # [추가됨] 현재 면적(거리) 값 출력
                    cv2.putText(cv_image, f"Area: {int(area)}", (10, 90), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            else:
                # 못 찾았을 때
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cv2.putText(cv_image, "Searching...", (10, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 명령 발행
            self.publisher_.publish(twist)
            
            # 화면 출력
            cv2.imshow("Duckie Vision", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DuckieChaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

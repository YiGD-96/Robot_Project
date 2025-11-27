import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import SetRobotMode
import time 

# 전역 변수 설정
global g_node

if not rclpy.ok():
    rclpy.init()
    
# ROS2 노드 생성
g_node = rclpy.create_node("dsr_control_node", namespace="dsr01")
print(f"[DEBUG] g_node before setting service: {g_node}")

# DR_init에 g_node 전달 (DSR_ROBOT2에서 사용 가능하도록 설정)
import DR_init
DR_init.__dsr__node = g_node  # 명확하게 설정하여 `None` 방지

# 서비스 절대 경로 설정
_srv_name_prefix = "/dsr01/system/"
set_robot_mode_service = _srv_name_prefix + "set_robot_mode"# 클라이언트 생성 (DSR_ROBOT2를 임포트하기 전에 설정)
client = g_node.create_client(SetRobotMode, set_robot_mode_service)# 서비스가 실행될 때까지 대기
wait_attempts = 5

for attempt in range(wait_attempts):
    if client.wait_for_service(timeout_sec=2.0):
        break
    g_node.get_logger().warning(f"Attempt {attempt+1}: Service {set_robot_mode_service} not available. Retrying...")
    
    if not client.wait_for_service(timeout_sec=2.0):
        g_node.get_logger().error(f"Service {set_robot_mode_service} still not available after {wait_attempts} attempts! Exiting.")
    exit(1)# `DSR_ROBOT2.py`를 임포트하기 전에 `DR_init.__dsr__node`를 설정했으므로 `g_node`가 유지됨!

import DSR_ROBOT2

# `DSR_ROBOT2.g_node`도 명확하게 설정하여 `None` 방지
DSR_ROBOT2.g_node = g_node

# `g_node`가 올바르게 설정되었는지 확인
print(f"[DEBUG] g_node after importing DSR_ROBOT2: {DSR_ROBOT2.g_node}")

# DR_init에 ID 및 모델 정보 설정
DR_init.__dsr__id = "dsr01"
DR_init.__dsr__model = "m0609"

# ROS2 노드 클래스 정의
class RobotController(Node):
    def __init__(self):
        super().__init__("force_control_node")        
        
        # API 불러오기
        try:
            from DSR_ROBOT2 import movej, movel, set_tool, set_tcp
            from DR_common2 import posx
        except ImportError as e:
            self.get_logger().error(f"Error importing modules: {e}")
            rclpy.shutdown()
            return        
        self.movej = movej
        self.movel = movel
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.posx = posx        
        
        set_tool("ToolWeight")
        set_tcp("GripperDA_v1")    
        
        self.JReady = [0, 0, 90, 0, 90, 0]
        self.pos1 = self.posx([496.06, 93.46, 296.92, 20.75, 179.00, 19.09])
        self.pos2 = self.posx([548.70, -193.46, 96.92, 20.75, 179.00, 19.09])
        self.pos3 = self.posx([596.70, -7.46, 196.92, 20.75, 179.00, 19.09])        
        self.timer = self.create_timer(1.0, self.pick_and_place)    

    def pick_and_place(self):
        """로봇을 초기 위치로 이동 후, 목표 위치로 선형 이동"""
        print("movej")
        self.movej(self.JReady, vel=60, acc=60)
        print("movel")
        self.movel(self.pos1, vel=60, acc=60)
        print("movel")
        self.movel(self.pos2, vel=60, acc=60)
        print("movel")
        self.movel(self.pos3, vel=60, acc=60)
        
# 메인 실행 함수
def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)    
        
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
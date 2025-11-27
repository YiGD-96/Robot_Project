import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage, QPixmap
from PyQt5 import uic
import cv2, imutils
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import numpy as np
import json
import queue
import time
from threading import Thread
import os

# 현재 스크립트 파일의 디렉토리 경로를 가져옵니다.
# (필요한 경우 current_dir 변수를 활용할 수 있습니다.)
current_dir = os.path.dirname(os.path.realpath(__file__))

# UI 파일의 절대경로를 생성합니다.
ui_file = os.path.join('/home/rokey/gui_ws/src/gui_run/simple_qt_ros.ui')

# 절대경로를 이용하여 UI 파일을 로드합니다.
form_class = uic.loadUiType(ui_file)[0]

# Author: Karl.Kwon
# Email: mrthinks@gmail.com

job_list = ['{"red": 2, "blue": 1, "goal": 1}', 
            '{"red": 1, "blue": 2, "goal": 2}', 
            '{"red": 1, "goal": 3}']

class WindowClass(QMainWindow, form_class):
	def __init__(self, operation_queue, node):
		super().__init__()
		self.setupUi(self)
		self.node = node

		self.btn_conveyorStart.clicked.connect(self.conveyorStartCallback)
		self.btn_conveyorStop.clicked.connect(self.conveyorStopCallback)
		self.btn_job.clicked.connect(self.btnjobCallback)
		# self.label_1.setText('HAHA')

		self.isClosed = False
		self.operation_queue = operation_queue

		for d in job_list:
			self.comboBox.addItem(d)
		self.selectdStrCb = job_list[0]

		self.comboBox.activated[str].connect(self.onActivate)

	def conveyorStartCallback(self):
		print("conveyorStartCallback clicked")
		self.node.send_start_conveyor()

	def conveyorStopCallback(self):
		print("conveyorStopCallback clicked")
		self.node.send_stop_conveyor()

	def btnjobCallback(self):
		print(self.selectdStrCb)
		self.node.send_job_command(self.selectdStrCb)

	def onActivate(self, text):
		print(text)
		self.selectdStrCb = text

	def editAddFunction(self, text):
		self.edit_1.append(text)

	def showImage(self, image_np):
		image = self.cvimage_to_label(image_np)
		self.label_1.setPixmap(QPixmap.fromImage(image))

	def cvimage_to_label(self,image):
		image = imutils.resize(image,width = 640)
		image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
		image = QImage(image,
			image.shape[1],
			image.shape[0],
			QImage.Format_RGB888)

		return image

	def closeEvent(self, event):
		self.operation_queue.put(' ')
		self.operation_queue.put(' ')

		print("closeEvent")

		time.sleep(0.1)

		self.isClosed = True


class GuiNode(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_np = None
        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            '/yolo_image/compressed',
            self.listener_callback_rgb,
            10)
        self.subscription_rgb  # prevent unused variable warning

        self.conveyor_pub = self.create_publisher(String, 'conveyor/control', 10)
        # 구독 시 topic 이름 및 콜백 설정
        self.conveyor_sub = self.create_subscription(
            String,
            'conveyor/status',
            self.conveyor_status_callback,
            10)
        self.conveyor_status = None

        self.gui_pub = self.create_publisher(String, 'gui/command', 10)

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def conveyor_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # status 키에 해당하는 값만 저장
            self.conveyor_status = data.get("status", "Unknown")
        except json.JSONDecodeError:
            self.conveyor_status = "Invalid data"

    def get_conveyor_status(self):
        ret = self.conveyor_status
        self.conveyor_status = None
        return ret

    def send_start_conveyor(self):
        print('send_start_conveyor')
        msg = String()
        msg.data = '{"control":"go", "distance.mm": 1000}'
        self.conveyor_pub.publish(msg)

    def send_stop_conveyor(self):
        print('send_stop_conveyor')
        msg = String()
        msg.data = '{"control":"stop"}'
        self.conveyor_pub.publish(msg)

    def send_job_command(self, text):
        print('send_job_command')
        msg = String()
        msg.data = text
        self.gui_pub.publish(msg)
		
    def get_node_connectivity_status(self):
        nodes = self.get_node_names_and_namespaces()
        active_names = [name for name, ns in nodes]
        
        monitored_nodes = {
            "controller_manager": "Turtlebot3 Bringup",
            "move_group": "MoveIt2"
        }
        
        status_str = ""
        for key, display_name in monitored_nodes.items():
            if key in active_names:
                status_str += f"{display_name}: Connected\n"
            else:
                status_str += f"{display_name}: Not Connected\n"
        return status_str




def ros_thread(operation_queue, event_queue, sleep_time):
	time_p = time.time()

	while(True):
		try:
			d = operation_queue.get_nowait()
			if d is not None:
				break
		except queue.Empty:
			pass

		time_c = time.time()
		if (time_c - time_p) > (sleep_time):
			event_queue.put('sleep_time: ', sleep_time)
			time_p = time_c

		time.sleep(0.01)
		# time.sleep(sleep_time)

	print('exit : ', sleep_time)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    operation_queue = queue.Queue()

    rclpy.init(args=None)
    image_node = GuiNode()
    myWindow = WindowClass(operation_queue, image_node)
    myWindow.show()

    event_queue1 = queue.Queue()
    Thread(target=ros_thread, args=(operation_queue, event_queue1, 1), daemon=True).start()
    event_queue2 = queue.Queue()
    Thread(target=ros_thread, args=(operation_queue, event_queue2, 2), daemon=True).start()

    # 노드 상태 업데이트 주기 관련 변수 선언 (초 단위)
    last_node_check = time.time()
    node_check_interval = 1.0  # 1초마다 업데이트

    while not myWindow.isClosed:
        try:
            d1 = event_queue1.get_nowait()
            if d1 is not None:
                myWindow.editAddFunction('1' + d1)
        except queue.Empty:
            pass

        try:
            d2 = event_queue2.get_nowait()
            if d2 is not None:
                myWindow.editAddFunction('2' + d2)
        except queue.Empty:
            pass

        rclpy.spin_once(image_node, timeout_sec=0)
        if image_node.image_np is not None:
            myWindow.showImage(image_node.image_np)

        # conveyor 상태 업데이트: 상태에 따라 색상 및 텍스트 설정
        if image_node.conveyor_status is not None:
            state = image_node.get_conveyor_status().upper()  # 대문자로 통일
            if state == "RUN":
                myWindow.label_conveyorStatus.setText("conveyor state: RUN")
                myWindow.label_conveyorStatus.setStyleSheet("color: green;")
            elif state == "READY":
                myWindow.label_conveyorStatus.setText("conveyor state: READY")
                myWindow.label_conveyorStatus.setStyleSheet("color: blue;")
            else:
                myWindow.label_conveyorStatus.setText("conveyor state:ERROR")
                myWindow.label_conveyorStatus.setStyleSheet("color: black;")

        else:
            myWindow.label_conveyorStatus.setText("conveyor state: NONE")
            myWindow.label_conveyorStatus.setStyleSheet("color: black;")

        # 노드 연결 상태 업데이트 (1초마다)
        # 노드 상태를 일정 주기로 업데이트
        current_time = time.time()
        if current_time - last_node_check > node_check_interval:
            node_status = image_node.get_node_connectivity_status()
            
            # 여러 줄 표시 가능
            myWindow.label_nodeStatus.setWordWrap(True)
            myWindow.label_nodeStatus.setText(node_status)
            
            # 여러 노드를 모니터링해서 "Not Connected", "Connected"가 섞여 있을 수 있으므로
            # 문자열 전체에 대해 포함 여부로 색상 결정
            if "Not Connected" in node_status:
                myWindow.label_nodeStatus.setStyleSheet("color: red;")
            elif "Connected" in node_status:
                myWindow.label_nodeStatus.setStyleSheet("color: green;")
            else:
                myWindow.label_nodeStatus.setStyleSheet("color: black;")

            last_node_check = current_time



        app.processEvents()

    image_node.destroy_node()
    rclpy.shutdown()

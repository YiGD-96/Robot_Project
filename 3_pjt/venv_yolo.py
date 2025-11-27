import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # 객체 정보 퍼블리시를 위한 String 메시지
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # YOLOv8 라이브러리

class YOLOv8ImageProcessor(Node):
    def __init__(self):
        super().__init__('yolov8_image_processor')

        # YOLOv8 모델 로드
        self.model = YOLO("/home/yigd/Downloads/best.pt")  # best.pt 경로 지정
        self.model.conf = 0.01  # confidence threshold 설정

        # ROS2 설정
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/fixed_camera_3/camera/image_raw',
            self.image_callback,
            10)
        
        # 결과 이미지 퍼블리시
        self.image_publisher = self.create_publisher(Image, '/yolo_result/image', 10)

        # 객체 정보 퍼블리시
        self.object_publisher = self.create_publisher(String, '/yolo_result/object', 10)

        self.get_logger().info("YOLOv8 Image Processor Node Initialized")

    def image_callback(self, msg):
        self.get_logger().info("Received Image")

        # ROS2 Image 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv8 모델 예측 실행
        results = self.model.predict(cv_image)

        # 객체 정보 저장할 리스트
        detected_objects = []

        # Bounding Box 및 Label 추가
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 바운딩 박스 좌표 변환
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = f"{self.model.names[cls]} {conf:.2f}"

                # 바운딩 박스 그리기
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 감지된 객체 정보 저장
                detected_objects.append(f"Object: {self.model.names[cls]}, Confidence: {conf:.2f}")

        # 감지된 객체 정보 퍼블리시
        if detected_objects:
            object_info = "\n".join(detected_objects)
            object_msg = String()
            object_msg.data = object_info
            self.object_publisher.publish(object_msg)
            self.get_logger().info(f"Published Object Info:\n{object_info}")

        # 변환된 이미지를 퍼블리시
        result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_publisher.publish(result_msg)
        self.get_logger().info("Published Processed Image")


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import random

class TurtleBotSpawner(Node):
    def __init__(self):
        super().__init__('turtlebot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        # 오브젝트 이름 증가용 카운터
        self.object_count = 1

        # 10초마다 터틀봇 생성
        self.timer = self.create_timer(1.0, self.spawn_random_turtlebot)

    def spawn_turtlebot(self, name, x, y, z, color, shape):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Gazebo spawn service not available!')
            return

        shape_geometry = self.get_shape_geometry(shape)

        request = SpawnEntity.Request()
        request.name = name
        request.xml = f"""<?xml version='1.0'?>
        <sdf version='1.6'>
            <model name='{name}'>
                <static>false</static>
                <link name='chassis'>
                    <pose>0 0 0 0 0 0</pose>
                    <inertial>
                        <mass>1.0</mass>
                        <inertia>
                            <ixx>0.1</ixx> <iyy>0.1</iyy> <izz>0.1</izz>
                        </inertia>
                    </inertial>
                    <collision name='collision'>
                        <geometry>
                            {shape_geometry}
                        </geometry>
                    </collision>
                    <visual name='visual'>
                        <geometry>
                            {shape_geometry}
                        </geometry>
                        <material>
                            <ambient>{color} 1</ambient>
                        </material>
                    </visual>
                </link>
            </model>
        </sdf>"""

        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        future = self.cli.call_async(request)
        future.add_done_callback(lambda future: self.spawn_callback(future, name, x, y, z, color, shape))

    def spawn_callback(self, future, name, x, y, z, color, shape):
        if future.result() is not None:
            self.get_logger().info(f"Spawned {name} at ({x}, {y}, {z}) with color {color} and shape {shape}")
        else:
            self.get_logger().error(f"Failed to spawn {name}")

    def spawn_random_turtlebot(self):
        name = f"object{self.object_count}"
        self.object_count += 1

        # 랜덤 색상 선택
        colors = {
            "빨강": "1.0 0.0 0.0",
            "초록": "0.0 1.0 0.0",
            "파랑": "0.0 0.0 1.0",
            "노랑": "1.0 1.0 0.0"
        }
        color_name, color_value = random.choice(list(colors.items()))

        # 랜덤 모양 선택 및 지정된 위치 설정
        shape_positions = {
            "capsule": (-2.5, 3.0, 0.3),
            "circle": (-2.5, 2.0, 0.3),
            "box": (-2.5, 1.0, 0.3),
            "rectangle": (-2.5, 0.0, 0.3)
        }
        shape = random.choice(list(shape_positions.keys()))
        x, y, z = shape_positions[shape]

        self.spawn_turtlebot(name, x, y, z, color_value, shape)

    def get_shape_geometry(self, shape):
        if shape == "circle":
            return '<cylinder><radius>0.1</radius><length>0.1</length></cylinder>'
        elif shape == "box":
            return '<box><size>0.25 0.25 0.2</size></box>'
        elif shape == "capsule":
            return '<mesh><uri>model://object_conveyor/meshes/capsule.stl</uri><scale>0.7 0.7 0.7</scale></mesh>'
        elif shape == "rectangle":
            return '<box><size>0.25 0.10 0.1</size></box>'
        else:
            return '<box><size>0.25 0.25 0.2</size></box>'


def main():
    rclpy.init()
    node = TurtleBotSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

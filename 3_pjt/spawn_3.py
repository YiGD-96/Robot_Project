import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import random
import time

class TurtleBotSpawner(Node):
    def __init__(self):
        super().__init__('turtlebot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.del_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.object_count = 1
        self.last_spawned_object = None  # 마지막으로 소환된 오브젝트 저장
        self.last_spawned_info = None  # (color, shape) 저장
        self.last_detection_time = 0  # 마지막 YOLO 탐지 시간
        self.last_spawn_time = time.time()  # 마지막 소환 시간
        
        self.timer = self.create_timer(6.0, self.spawn_random_turtlebot)
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(String, '/yolo_result/object', self.yolo_callback, qos_profile)

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
            self.last_spawned_object = name
            self.last_spawned_info = (color, shape)
            self.last_spawn_time = time.time()
            self.get_logger().info(f"Spawned {name} at ({x}, {y}, {z}) with color {color} and shape {shape}")
        else:
            self.get_logger().error(f"Failed to spawn {name}")

    def spawn_random_turtlebot(self):
        if time.time() - self.last_spawn_time > 3.0:
            name = f"object{self.object_count}"
            self.object_count += 1
            positions = [(-4.5, 1.5, 1.0)]
            x, y, z = random.choice(positions)

            #colors = {"빨강": "1.0 0.0 0.0", "초록": "0.0 1.0 0.0", "파랑": "0.0 0.0 1.0", "노랑": "1.0 1.0 0.0"}
            colors = {"초록": "0.0 1.0 0.0"}
            color_name, color_value = random.choice(list(colors.items()))
            #shapes = ["circle", "box", "capsule", "rectangle"]
            shapes = ["capsule"]
            shape = random.choice(shapes)
            
            self.spawn_turtlebot(name, x, y, z, color_value, shape)

    def get_shape_geometry(self, shape):
        if shape == "circle":
            return '<cylinder><radius>0.1</radius><length>0.1</length></cylinder>'
        elif shape == "box":
            return '<box><size>0.25 0.25 0.2</size></box>'
        elif shape == "capsule":
            return '<mesh><uri>model://object_conveyor/meshes/capsule.stl</uri><scale>0.7 0.7 0.7</scale></mesh>'
        elif shape == "rectangle":
            return '<box><size>0.10 0.10 0.25</size></box>'
        else:
            return '<box><size>0.25 0.25 0.2</size></box>'

    def yolo_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_detection_time > 2.0:
            if 'Object: capsule' in msg.data:
                self.get_logger().info("Capsule detected! Moving last spawned object.")
                self.last_detection_time = current_time
                self.reposition_or_respawn()

    def reposition_or_respawn(self):
        target_x, target_y, target_z = -2.5, 3.0, 0.3

        if self.last_spawned_object:
            self.get_logger().info(f"Deleting {self.last_spawned_object} and respawning at ({target_x}, {target_y}, {target_z})")
            delete_request = DeleteEntity.Request()
            delete_request.name = self.last_spawned_object
            future = self.del_cli.call_async(delete_request)
            future.add_done_callback(lambda _: self.respawn_last_object(target_x, target_y, target_z))
        else:
            self.get_logger().error("No object available to reposition or respawn!")

    def respawn_last_object(self, x, y, z):
        if self.last_spawned_info:
            color, shape = self.last_spawned_info
            name = f"object{self.object_count}"
            self.object_count += 1
            self.spawn_turtlebot(name, x, y, z, color, shape)


def main():
    rclpy.init()
    node = TurtleBotSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
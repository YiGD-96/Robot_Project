# pick, place, and throw module @20241104

import rclpy
import DR_init
import numpy as np

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 500, 500

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def initialize_robot():
    """ROS2 노드 초기화 및 로봇 설정"""
    rclpy.init()
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            set_tool,
            set_tcp,
            movej,
            movel,
            amovej,
            amovel,
            movesx,
            wait,
        )
        from DR_common2 import posx

        globals().update(locals())  # 로드된 모듈을 글로벌 범위로 설정

    except ImportError as e:
        print(f"Error importing modules: {e}")
        return None

    # 로봇 툴 설정
    set_tool("ToolWeight")
    set_tcp("GripperDA_v1")

    # 초기 위치 이동
    JReady = [0, 0, 90, -10, 90, 0]
    #movej(JReady, vel=VELOCITY, acc=ACC)

    return node

def release():
    """그리퍼 OFF (물체 놓기)"""
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait(0.1)

def grip():
    """그리퍼 ON (물체 잡기)"""
    release()
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1.0)

def movesx_o(pos1, pos2, y_offset=0, above_offset=100, velocity=200, acc=200):
    """
    Pick & Place 작업 수행

    :param pos1: 픽업 위치 (posx 객체)
    :param pos2: 배치 위치 (posx 객체)
    :param above_offset: 안전거리 (기본값: 100)
    :param velocity: 이동 속도 (기본값: 400)
    :param acc: 가속도 (기본값: 400)
    """
    # 안전 거리 확보 좌표 설정
    pos1_above = posx([pos1[0], pos1[1] + y_offset, pos1[2] + above_offset, pos1[3], pos1[4], pos1[5]])
    pos2_above = posx([pos2[0], pos2[1], pos2[2] + above_offset, pos2[3], pos2[4], pos2[5]])

    # 1. 시작 위치 -> pos1_above -> pos1 (픽업 지점으로 이동)
    #path_to_pick = [pos1_above, pos1]
    #movesx(path_to_pick, vel=velocity, acc=acc)
    movel(pos1, vel=velocity, acc=acc)

    # 2. Gripper ON (픽업)
    grip()
    rclpy.spin_once(DR_init.__dsr__node, timeout_sec=1.0)  # 응답 대기

    # 3. pos1 -> pos1_above -> pos2_above -> pos2 (이동 후 배치)
    path_to_place = [pos1_above, pos2_above, pos2]
    movesx(path_to_place, vel=velocity, acc=acc)

    # 4. Gripper OFF (물체 놓기)
    release()
    rclpy.spin_once(DR_init.__dsr__node, timeout_sec=1.0)  # 응답 대기

    # 5. pos2 -> pos2_above (다시 안전 거리로 올라감)
    movesx([pos2_above], vel=velocity, acc=acc)

def movesx_p(pos1, pos1_above, pos2_above, pos2, velocity=100, acc=100):
    """
    사용자 지정 Pick & Place 작업 수행 (4개의 점을 직접 입력받음)

    :param pos1_above: 픽업 위치 위쪽 (posx 객체)
    :param pos1: 픽업 위치 (posx 객체)
    :param pos2_above: 배치 위치 위쪽 (posx 객체)
    :param pos2: 배치 위치 (posx 객체)
    :param velocity: 이동 속도 (기본값: 400)
    :param acc: 가속도 (기본값: 400)
    """
    # 1. 시작 위치 pos1 (픽업 지점으로 이동)
    movel(pos1, vel=velocity, acc=acc)

    # 2. Gripper ON (픽업)
    grip()
    rclpy.spin_once(DR_init.__dsr__node, timeout_sec=1.0)  # 응답 대기

    # 3. pos1 -> pos1_above -> pos2_above -> pos2 (이동 후 배치)
    path_to_place = [pos1_above, pos2_above, pos2]
    movesx(path_to_place, vel=velocity, acc=acc)

    # 4. Gripper OFF (물체 놓기)
    release()
    rclpy.spin_once(DR_init.__dsr__node, timeout_sec=1.0)  # 응답 대기

    # 5. pos2 -> pos2_above (다시 안전 거리로 올라감)
    movesx([pos2_above], vel=velocity, acc=acc)

def throw_cup():
    #끝나고 난후 포즈
    JReady = [0, 0, 90, 0, 90, 0]
    movej(JReady, vel=200, acc=100)

    #일단 악수 할수 있게 몸을틈
    pos1 = [-38, 0, 90, 0, 90, 0]
    movej(pos1, vel=200, acc=100)
    #악수 아래,위아래
    #posj 기준 : [-30, 0, 90, 0, 50, 0]
    pos2 = posx([473.958, -357.882, 274.595, 142.381, -139.999, -179.941])
    #posj 기준 : [-30, 0, 90, 0, 75, 0] 
    pos3 = posx([366.902, -275.186, 204.201, 142.477, -164.998, -179.851])
    path_to_place = [pos2, pos3, pos2, pos3]
    movesx(path_to_place, vel=800, acc=800)

    wait(1)
    grip()
    movej(pos1, vel=200, acc=200)
    wait(4)
    pos_throw = [-38, 0, 90, 40, 90, 0]

    amovej(pos_throw, vel=1600, acc=1600)
    wait(0.08)
    release()

def main():
    # 1.초기 위치 이동
    node = initialize_robot()
    if node is None:
        return

    throw_cup()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

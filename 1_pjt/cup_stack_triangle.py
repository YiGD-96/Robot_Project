import rclpy
import math
import DR_init
import numpy as np

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def initialize_robot():
    rclpy.init()
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output, set_tool, set_tcp, movej, movel, amovej, amovel, movesx, wait,
        )
        from DR_common2 import posx
        globals().update(locals())
    except ImportError as e:
        print(f"Error importing modules: {e}")
        return None

    set_tool("ToolWeight")
    set_tcp("GripperDA_v1")
    release()
    JReady = [0, 0, 90, -10, 90, 0]
    movej(JReady, vel=VELOCITY, acc=ACC)
    return node

def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait(0.1)

def grip():
    release()
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(0.3)

def movesx_o(pos1, pos2, ry_offset=0, above_offset=100, velocity=400, acc=200, gripper=True):
    pos1_above = posx([pos1[0], pos1[1] + ry_offset, pos1[2] + above_offset, pos1[3], pos1[4] + ry_offset, pos1[5]])
    pos2_above = posx([pos2[0], pos2[1], pos2[2] + above_offset, pos2[3], pos2[4], pos2[5]])
    movel(pos1, vel=velocity, acc=acc)
    if gripper:
        grip()
    movesx([pos1_above, pos2_above, pos2], vel=velocity, acc=acc)
    release()
    movesx([pos2_above], vel=velocity, acc=acc)

def stack(c_x, c_y, z, columns, radius):
    positions = []
    for row in range(columns):
        row_positions = row + 1
        x_offset = -row * radius / 2
        for col in range(row_positions):
            x = c_x + x_offset + col * radius
            y = c_y - row * (radius * 0.866)
            positions.append([x, y, z, 94.76, 130.00, 94.42])
    return sorted(positions, key=lambda p: (-p[1], p[0]))

def stack_objects(pickup_pos, stack_positions, base_z_bozung):
    cnt, z_bozung = 0, 0
    for place_pos in stack_positions:
        if cnt == 0:
            next_pos = posx(place_pos)
            movesx_o(pickup_pos, next_pos, ry_offset=-10)
            print(cnt, ':', place_pos)
        else:
            up_pos = posx([next_pos[0], next_pos[1], next_pos[2] + 8 - z_bozung, next_pos[3], next_pos[4], next_pos[5]])
            next_pos = posx(place_pos)
            movesx_o(up_pos, next_pos, ry_offset=-10, acc=1000)
            print(cnt, ':', place_pos)
        z_bozung = base_z_bozung * cnt
        cnt += 1

def main():
    node = initialize_robot()
    if node is None:
        return
    
    pickup_pos = posx([367.320, 200.000, 60.000, 94.76, 130.00, 94.42])
    stack_positions_1 = stack(550, 80, -3, 3, 80)
    stack_objects(pickup_pos, stack_positions_1, base_z_bozung=1.65)

    release()
    pickup_pos = posx([367.320, 200.000, 27.000, 94.76, 130.00, 94.42])
    movesx_o(stack_positions_1[-1], pickup_pos, acc=1000, gripper=False)
    stack_positions_2 = stack(550, 40, 90, 2, 80)
    stack_objects(pickup_pos, stack_positions_2, base_z_bozung=10)
    
    release()
    pickup_pos = posx([367.320, 200.000, 5.000, 94.76, 130.00, 94.42])
    top_pos = posx([550.000, 0, 185.000, 94.76, 130.00, 94.42])
    movesx_o(stack_positions_2[-1], pickup_pos, acc=1000, gripper=False)
    movesx_o(pickup_pos, top_pos, ry_offset=-10, acc=1000)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

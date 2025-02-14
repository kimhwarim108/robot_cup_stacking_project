# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
OFF, ON = 0, 1
up_height = 90
OFFSET = 28
plate1 = [
    [518.26+OFFSET , 148.02, up_height, 169.48, 178.92, 170.64],
    [520.88+OFFSET , 96.12, up_height, 98.23, 179.56, 99.38],
    [520.96+OFFSET, 44.17, up_height, 105.46, 179.56, 106.76],
    [571.70+OFFSET, 147.12, up_height, 179.17, -179.18, 179.17],
    [572.81+OFFSET, 96.52, up_height, 37.48, 179.72, 37.56],
    [573.55+OFFSET, 45.75, up_height, 69.82, 179.48, 71.11],
    [623.62+OFFSET, 150.84, up_height, 172.26, -179.49, 171.88],
    [623.21+OFFSET, 96.06, up_height, 61.54, 179.66, 61.72],
    [623.29+OFFSET, 46.95, up_height, 76.46, 179.47, 77.87]
    ]
plate2 = [
    [520.72+OFFSET, -54.98, up_height, 50.46, 179.71, 51.99],
    [520.07+OFFSET, -107.23, up_height, 25.89, 179.63, 27.65],
    [520.44+OFFSET, -158.66, up_height, 149.86, -179.44, 150.13],
    [571.05+OFFSET, -55.87, up_height, 114.04, 179.64, 114.22],
    [570.58+OFFSET, -106.27, up_height, 129.74, 179.70, 129.95],
    [571.36+OFFSET, -157.68, up_height, 152.91, -179.48, 153.07],
    [622.31+OFFSET, -54.43, up_height, 75.22, 179.62, 75.34],
    [622.01+OFFSET, -106.67, up_height, 96.24, 179.55, 96.4],
    [623.24+OFFSET, -156.75,up_height, 175.10, -179.50, 175.24]
]
pos = [plate1, plate2]
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            get_current_posx,
            mwait,
            wait,
        )
        from DR_common2 import posx
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    def move_down(depth):
        movel([0, 0, -depth, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    def move_up(height):
        movel([0, 0, height, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            #print("Wait for digital input")
            pass
    def pick_and_place(pick_number, place_number, pick_plate=None, place_plate=None,):
        # pick 과정
        movel(pos[pick_plate][pick_number], vel=VELOCITY, acc=ACC)
        move_down(65)
        grip_with_delay()
        move_up(65)
        movel(pos[place_plate][place_number], vel=VELOCITY, acc=ACC)
        move_down(50)
        release()
        move_up(50)
    def grip_with_delay():
        #release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
    def check_bar_height(plate_number, bar_index):
        """
        해당 위치로 이동 후 막대들의 높이를 측정
        plate_number와 bar_index를 입력하면 해당 위치의 바의 높이를 측정 합니다
        plate_number : 아래 고정 판의 번호
        bar_index : 고정 판에 꽂혀있는 바의 좌표
        """
        # 1,1 위치로 이동
        movel(pos[plate_number][bar_index], vel=VELOCITY, acc=ACC)
        grip()
        # 로봇 버티는 힘 설정화0
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        # 로봇이 아래 방향으로 줄 힘 설정
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        # 현재 로봇에 입력되는 힘 측정
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        # 로봇이 아래 방향으로 주는 힘 초기화
        release_compliance_ctrl()
        mwait()
        curr_pos = get_current_posx()[0][2]
        if curr_pos > 60:
            bar_height_level = 2
        elif curr_pos > 50:
            bar_height_level = 1
        else:
            bar_height_level = 0
        release()
        return bar_height_level
    def sort_bar_by_level(plate_condition, level):
        """
        input
        plate_condition : 목적지 plate에 막대가 차지하고 있는 상태 표현
        level : 해당 막대의 높이 [0, 1, 2]로 type을 표현
        return
        index : 목적지의 좌표을
        """
        index = 3*level + len(plate_condition[level])
        plate_condition[level].append(0)
        print(plate_condition[0])
        print(plate_condition[1])
        print(plate_condition[2])
        return plate_condition, index
    def rotated_sort_bar_by_level(plate_condition, level):
        index = level + 3*len(plate_condition[level])
        plate_condition[level].append(0)
        print(plate_condition[0])
        print(plate_condition[1])
        print(plate_condition[2])
        return plate_condition, index
        # 0, 3, 6
        # 1, 4, 7
        # 2, 5, 8
    PICK_PLATE = 0
    PLACE_PLATE = 1
    # 초기 위치
    JReady = [0, 0, 90, 0, 90, 0]
    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")
    pick_plate_pose_condition = []
    place_plate_condition = [[],[],[]]
    if rclpy.ok():
        release()
        # 초기 위치로 이동
        movej(JReady, vel=VELOCITY, acc=ACC)
        for i in range(9):
            plate1_pose_condition.append(check_bar_height(PICK_PLATE,i))
            # print(plate1_pose_condition)
        for num, lev in enumerate(plate1_pose_condition):
            place_plate_condition, index = rotated_sort_bar_by_level(place_plate_condition, lev)
            pick_and_place(num, index, pick_plate=PICK_PLATE, place_plate=PLACE_PLATE)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
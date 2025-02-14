        
import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY, ACC = 60, 60
MOVE_UP_GRIP_CUP = 100
MOVE_DOWN_DROP_CUP = 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

all_cup_count = 8  # 블록이 몇 층 쌓여 있는지

def main(args=None):
    rclpy.init(args=args)  # 초기화
    node = rclpy.create_node("stacking_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            amove_periodic,
            set_digital_output,
            get_digital_input,
            wait,
            mwait,
            get_current_posx,
            DR_MV_MOD_REL,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            # checkget_current_posx_force_condition,
            check_force_condition,
            DR_AXIS_Z,
            release_compliance_ctrl,
            DR_TOOL,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def release():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def grip():
        set_digital_output(1, 1)
        set_digital_output(2, 0)

    def release():
        set_digital_output(2, 1)
        set_digital_output(1, 0)

    def wait_digital_input(sig_num): 
        while not get_digital_input(sig_num):
            wait(0.5)
            pass
            
    def grip_with_delay(): #물체를 잡고 확인
        print("gripping OK")
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait_digital_input(1)
        if not wait_digital_input(1):
                print("grip failed T.T")

    # 아래로 이동
    def move_down(depth):
        movel([0, 0, -depth, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    # 위로 이동
    def move_up(height):
        movel([0, 0, height, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    def drop_cup_slowly():
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        
        # 로봇이 아래 방향으로 줄 힘
        set_desired_force(fd=[0, 0, -8, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        # 현재 로봇에 입력되는 힘 측정
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        
        # 로봇이 아래 방향으로 주는 힘 초기화
        release_compliance_ctrl()
        move_up(1) 
        wait(0.3) # 기다려
        release() # 놓아

    # 컵 높이 감지
    def check_cup_height(): 
        grip()
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100]) 
        set_desired_force(fd=[0, 0, -8, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        while not check_force_condition(DR_AXIS_Z, max=5):
            pass

        release_compliance_ctrl()
        mwait()
        curr_height = get_current_posx()[0][2]
        release()
        return curr_height

    # 컵을 원하는 위치로 이동
    def move_cup(destination):
        move_up(MOVE_UP_GRIP_CUP)
        movel(destination, vel=VELOCITY, acc=ACC)
        move_down(MOVE_DOWN_DROP_CUP)
        drop_cup_slowly()
        release()
        wait(0.3)

    # 3개 동시에 잡기
    def grip_three_cups():
        print("Gripping three cups")
        move_down(50)
        grip_with_delay()
        move_up(120)

    # 3개 컵에서 2개만 놓기
    def grip_two_from_three():
        print("Releasing 1 cup, keeping 2 cups")
        release() # 열어서 3개의 컵을 놓고
        wait(0.3) # 기다려
        move_up(10) # 1cm 정도 올라와
        grip() # 2개의 컵을 잡기

    # 2개 컵에서 1개만 놓기
    def grip_one_from_two():
        print("Releasing 1 more cup, keeping 1")
        release() # 놓고 컵이 떨어지면
        wait(0.3) # 잠깐 기다리고
        move_up(15) # 1.5cm 정도 올라와서
        grip() # 1개의 컵을 잡기

    # 컵 3개를 이동 후 분배
    def divide_three_cups(stack_positions):
        grip_two_from_three()
        destination = stack_positions.pop()
        move_cup(destination)

        grip_one_from_two()
        destination = stack_positions.pop()
        move_cup(destination)

    # 초기 위치에서 컵 3개 이동
    def move_three_from_init_pose(stack_positions):
        movel(START_POSITION, vel=VELOCITY, acc=ACC)
        grip_three_cups()
        destination = stack_positions.pop()
        move_cup(destination)

    # 초기 위치 설정
    JReady = [0, 0, 90, 0, 90, 0]
    
    START_POSITION = posx([600, 200, 120, 90, 90, 90])

    # 1층, 2층, 3층 자동 배치
    base_x, base_y, base_z = 500, 200, 120
    cup_diameter = 70  # 컵 지름 (mm)
    height_step = 70    # 층별 높이 증가 (mm)

    stack_positions = [
        posx([base_x, base_y, base_z, 90, 90, 90]),
        posx([base_x + cup_diameter, base_y, base_z, 90, 90, 90]),
        posx([base_x + 2 * cup_diameter, base_y, base_z, 90, 90, 90]),

        posx([base_x + cup_diameter / 2, base_y + 35, base_z + height_step, 90, 90, 90]),
        posx([base_x + 1.5 * cup_diameter, base_y + 35, base_z + height_step, 90, 90, 90]),

        posx([base_x + cup_diameter, base_y + 70, base_z + 2 * height_step, 90, 90, 90])
    ]

    # 로봇 동작 실행
    if rclpy.ok():
        movej(JReady, vel=VELOCITY, acc=ACC)
        move_three_from_init_pose(stack_positions)
        divide_three_cups(stack_positions)

    rclpy.shutdown()

if __name__ == "__main__":
    main()








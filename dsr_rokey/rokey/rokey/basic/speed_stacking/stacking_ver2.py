'''import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args) #초기화
    node = rclpy.create_node("stacking_robot", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    
    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            set_digital_output,
            get_digital_input,
            wait,
            DR_MV_MOD_REL,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            check_force_condition,
            DR_AXIS_Z,
            release_compliance_ctrl,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def release(): # 열기
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        
    def grip():
        set_digital_output(1, 1)
        set_digital_output(2, 0) # 잡기
        
    def release():
        set_digital_output(2, 1)
        set_digital_output(1, 0) # 놓기
        
    def wait_digital_input(sig_num): #잡기 전까지 기다리기
        while not get_digital_input(sig_num):
            wait(0.5)
            #print("Wait for digital input")
            pass
        
    def grip_with_delay(): #물체를 잡고 확인
        print("gripping OK")
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait_digital_input(1)
        if not wait_digital_input(1):
                print("grip failed T.T")
                
    # 아래로 이동 - 지정한 만큼
    def move_down(depth):
        movel([0, 0, -depth, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        
    #위로 이동 - 지정한 만큼
    def move_up(height):
        movel([0, 0, height, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        
    def drop_cup_slowly():
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        # 로봇이 아래 방향으로 줄 힘 설정
        set_desired_force(fd=[0, 0, -8, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        # 현재 로봇에 입력되는 힘 측정
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        # 로봇이 아래 방향으로 주는 힘 초기화
        release_compliance_ctrl()
        move_up(1)
        wait(0.3)
        release()
        
    # 홈 위치
    JReady = [0, 0, 90, 0, 90, 0]
    
    # 픽업 위치
    pose_pickup = posx([600, 200, 120, 90, 90, 90])
    pose_z_value = 20
    
    # 층별 자동 배치
    base_x, base_y, base_z = 500, 200, 150
    cup_diameter = 80 #컵의 지름
    height_step = 100 #층별 높이 증가
    row_gap = cup_diameter * 1.7 # 줄 간격 (삼각형 정렬)
    #cup_height = 11 # 컵을 놓고 11mm 올라가기
    #move_up_extra = 100 # 컵 놓고 10cm 올라가기
    
    # 층별 컵 배치 위치 자동 계산
    stack_positions = []
    
    def calc_destination_pose():
        # 1층 정렬
        row1_x = [base_x + cup_diameter * 0 , base_x + cup_diameter * 1, base_x + cup_diameter * 2]
        row2_x = [base_x + cup_diameter * 0.5, base_x + cup_diameter * 1.5]
        row3_x = [base_x + cup_diameter * 1]
        for x_1 in row1_x:
            stack_positions.append(posx([x_1, base_y, base_z, 90, 90, 90]))
        for x_2 in row2_x:
            stack_positions.append(posx([x_2, base_y + row_gap, base_z, 90, 90, 90]))
        for x_3 in row3_x:
            stack_positions.append(posx([x_3, base_y + row_gap * 2, base_z, 90, 90, 90]))
            
        # 2층 정렬
        base_z_2 = base_z + height_step
        row1_x_2 = [base_x + cup_diameter * 0.5, base_x + cup_diameter * 1.5]
        row2_x_2 = [base_x + cup_diameter * 1]
        for x in row1_x_2:
            stack_positions.append(posx([x, base_y, base_z_2, 90, 90, 90]))
        for x in row2_x_2:
            stack_positions.append(posx([x, base_y + row_gap, base_z_2, 90, 90, 90]))
            
        # 3층 정렬
        base_z_3 = base_z_2 + height_step
        top_pose = posx([base_x + cup_diameter, base_y, base_z_3, 90, 90, 90])
        stack_positions.append(top_pose)
        return stack_positions
    
    if rclpy.ok():
        movej(JReady, vel=VELOCITY, acc=ACC)
        movel(pose_pickup, vel=VELOCITY, acc=ACC)
        pose_pickup[2] = pose_z_value
        move_down(98)
        grip_with_delay()
        move_up(100)
        stack_positions = calc_destination_pose()
        
        for pos in stack_positions:
            movel(pos, vel=VELOCITY, acc=ACC)  # 다음 컵 위치로 이동
            move_down(90)
            drop_cup_slowly()
            release()
            pose_pickup[0], pose_pickup[1] = pos[0], pos[1]
            movel(pose_pickup, vel=VELOCITY, acc=ACC)
            grip_with_delay()
            move_up(140)
            
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()'''
    
import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)  # 초기화
    node = rclpy.create_node("stacking_robot_simulation", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movej, movel, DR_MV_MOD_REL
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    # 📌 **로봇 초기 설정**
    JReady = [0, 0, 90, 0, 90, 0]
    
    # 초기 이동 시작 위치 (첫 번째 컵 위치)
    current_pickup_position = posx([600, 200, 120, 90, 90, 90])

    # 📌 **컵 배치 기준점**
    base_x, base_y, base_z = 500, 200, 150
    cup_diameter = 80
    height_step = 100
    row_gap = cup_diameter * 1.7

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    # 🚀 **이동 함수**
    def move_down(depth):
        movel([0, 0, -depth, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    def move_up(height):
        movel([0, 0, height, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    # 📌 **목적지 좌표 자동 생성**
    def calc_destination_pose():
        stack_positions = []

        # 1층 배치
        row1_x = [base_x, base_x + cup_diameter, base_x + 2 * cup_diameter]
        row2_x = [base_x + cup_diameter / 2, base_x + 1.5 * cup_diameter]
        row3_x = [base_x + cup_diameter]

        for x in row1_x:
            stack_positions.append(posx([x, base_y, base_z, 90, 90, 90]))
        for x in row2_x:
            stack_positions.append(posx([x, base_y + row_gap, base_z, 90, 90, 90]))
        for x in row3_x:
            stack_positions.append(posx([x, base_y + row_gap * 2, base_z, 90, 90, 90]))

        # 2층 배치
        base_z_2 = base_z + height_step
        row1_x_2 = [base_x + cup_diameter / 2, base_x + 1.5 * cup_diameter]
        row2_x_2 = [base_x + cup_diameter]

        for x in row1_x_2:
            stack_positions.append(posx([x, base_y, base_z_2, 90, 90, 90]))
        for x in row2_x_2:
            stack_positions.append(posx([x, base_y + row_gap, base_z_2, 90, 90, 90]))

        # 3층 배치
        base_z_3 = base_z_2 + height_step
        top_pose = posx([base_x + cup_diameter, base_y, base_z_3, 90, 90, 90])
        stack_positions.append(top_pose)

        return stack_positions

    # 🏗 **이동 경로 시뮬레이션 (놓은 위치에서 다음 이동 시작)**
    def simulate_movement(destination):
        """ 목표 위치로 이동 (놓은 위치에서 시작) """
        nonlocal current_pickup_position  # 현재 위치를 업데이트
        
        # 현재 위치에서 이동 시작
        movel(current_pickup_position, vel=VELOCITY, acc=ACC)
        move_up(100)  # 컵을 들었다고 가정
        movel(destination, vel=VELOCITY, acc=ACC)  # 목표 위치로 이동
        move_down(90)  # 컵을 놓는 동작을 시뮬레이션
        
        # 새로운 잡기 위치를 업데이트 (현재 놓은 위치가 다음 이동 시작 위치가 됨)
        current_pickup_position = destination

    # ✅ **로봇 동작 실행**
    if rclpy.ok():
        movej(JReady, vel=VELOCITY, acc=ACC)
        stack_positions = calc_destination_pose()

        for pos in stack_positions:
            simulate_movement(pos)

    rclpy.shutdown()

if __name__ == "__main__":
    main()

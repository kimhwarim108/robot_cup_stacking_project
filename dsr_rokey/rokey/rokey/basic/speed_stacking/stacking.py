import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

all_cup_count = 8  # 블록이 몇 층 쌓여 있는지
# p0 = posj(-0.79,28.3,47.26,-0.16,104.44,-0.59) # 초기 위
# p1 = posj(-1.22,5.2,59.44,-0.05,115.53,-0.96) # 두번째 위

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
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
            DR_MV_MOD_REL,
            task_compliance_ctrl,
            set_desired_force,
            DR_FC_MOD_REL,
            # checkget_current_posx_force_condition,
            DR_AXIS_Z,
            release_compliance_ctrl,
            mwait,
            get_current_posx,
            check_force_condition,
            DR_TOOL,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def grip():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        
    def release():
        set_digital_output(2, 1)
        set_digital_output(1, 0)
        
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            
    def grip_with_delay():
        set_digital_output(1, 1)
        set_digital_output(2, 0)
        wait_digital_input(1)
    def move_down(depth):
        movel([0, 0, -depth, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    
    def move_up(height):
        movel([0, 0, height, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    
    def pick_and_place_cup(offset_x=0, offset_y=0, offset_z=0, cup_ea = 1):
        global all_cup_count
        # p1 = posj(0,-3.27,94.52,0,88.82,0) # 두번째 위
        # p2 = posj(0,-1.22,77.23,0,104.47,0) # 두번째 위
        # p3 = posj(0,-1.17,74.94,0,106.31,0) # 두번째 위
        
        p0 = posj(-0.79,26.98,53.2,-0.16,99.70,-0.59) # 초기 위
        if offset_z <=0 :
            p1 = posj(0,-3.27,94.52,0,88.82,0) # 두번째 위
        elif offset_z <=100 :
            p1 = posj(0,-1.22,77.23,0,104.47,0) # 두번째 위
        elif offset_z <=200 :
            p1 = posj(0,-1.17,74.94,0,106.31,0)  # 두번째 위
        print(f'p1 = {p1}')
        
        pick_pos = posx([550, 0, all_cup_count * 11 + 90 + 30  - (50 + (cup_ea-1)*25), 90, 180, 90])
        if offset_z ==200 :
            place_pos = posx([300 + offset_x, -80 + offset_y, 100 + offset_z, 90, 180, 90])
        else:
            place_pos = posx([300 + offset_x, -80 + offset_y, 180 + offset_z, 90, 180, 90])
        
        # 컵 초기 위치 로 이동
        movej(p0, v=VELOCITY, a=ACC)
        movel(pick_pos, vel=VELOCITY, acc=ACC)
        print("position : pick_pos")
        
        # 내려가movel서 집기
        # move_down(50 + (cup_ea-1)*20)
        grip_with_delay()
        wait(0.2)
        if offset_z <=0 :
            move_up(130)
            # 컵 둘 위치로 이동
            movej(p1, v=VELOCITY, a=ACC)
            movel(place_pos, vel=VELOCITY, acc=ACC)
            # 내려가서 두기
            move_down(100)
        elif offset_z <=100 :
            move_up(100)# 컵 둘 위치로 이동
            movej(p1, v=60, a=60)
            movel(place_pos, vel=VELOCITY, acc=ACC)
            # 내려가서 두기
            move_down(100)
        elif offset_z <=200 :
            move_up(160)# 컵 둘 위치로 이동
            movej(p1, v=60, a=60)
            movel(place_pos, vel=VELOCITY, acc=ACC)
            # 내려가서 두기
            move_down(20)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        
        release_compliance_ctrl()
        release()
        all_cup_count -= 1
        move_up(30)
        # movej(p0, v=60, a=60)
    
    def create_cup_top(num_layers):
        offsets = [] # 좌표를 저장할 리스트
        # 오프셋 계산 및 리스트 저장
        for layer in range(num_layers, 0, -1): # 최상층부터 하층으로
            for row in range(layer, 0, -1): # 각 층의 행(row) 수
                for col in range(row, 0, -1): # 각 행의 열(col) 수
                    offset_y = (row - col) * 80 + (layer - row)  * 40         + (num_layers - layer ) * 40
                    offset_x =                    (layer - row ) * 40 * 1.7 + (num_layers - layer ) * 20 * 1.7
                    offset_z =                                                  (num_layers - layer ) * 100
                    offsets.append((offset_x, offset_y, offset_z)) # 리스트에 추가
        # 저장된 좌표를 사용하여 pick_and_place_cup 실행
        i = 0
        for offset_x, offset_y, offset_z in offsets:
            print(f'offset_x  {offset_x}   offset_y  {offset_y}    offset_z  {offset_z}')
            if i == 9:
                cup_ea = 2
            else:
                cup_ea = 1
            pick_and_place_cup(offset_x=offset_x, offset_y=offset_y, offset_z=offset_z , cup_ea = cup_ea )
            i+=1
    
    def last_cup_top():
        pos1 = posx([300 + 40 * 1.7, -80 + 80 + 20  , 310 , 90, 180, 89.9])
        pos2 = posx([300 + 40 * 1.7, -80 + 80 + 20 , 310 , 90, 90, 89.9])
        movel(pos1, vel=VELOCITY, acc=ACC)
        movel(pos2, vel=VELOCITY, acc=ACC)
        move_down(20)
        grip_with_delay()
        wait(0.2)
        move_up(100)
        # move_down(50)
        pos3 = posx([300 + 40 * 1.7, -80 + 80 + 15, 210 + 40 -5 + 100, 90, 90, -89.9])
        movel(pos3, vel=VELOCITY/2, acc=ACC)
        move_down(40)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(DR_AXIS_Z, max=5):
            pass
        release_compliance_ctrl()
        release()
        global all_cup_count
        all_cup_count -= 1
        # move_up(110)
    
    def check_floor(floor_height=10):
        print('test1')
        # pick_pos = posx([550, 0, 240, 90, 180, 90])   # 체크 초기위치
        # movel(pick_pos, vel=VELOCITY, acc=ACC)
        print('test2')
        current_z_pos = move_down_until_force(20, axis=DR_AXIS_Z, max_force=5)
        current_z_pos -= 100
        floor = current_z_pos // floor_height +  1
        print(f'floor = {floor}')
        return int(floor)
    
    def move_down_until_force(target_force, axis=DR_AXIS_Z, max_force=4):
        grip()
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -target_force, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while not check_force_condition(axis, max=max_force):
            pass
        current_z_pos = get_current_posx()[0][2]
        release_compliance_ctrl()
        move_up(10)
        release()
        print(f"current_z_pos = {current_z_pos}")
        return current_z_pos
    
    global all_cup_count
    JReady = [0, 0, 90, 0, 90, 0]
    lower_bound = 5
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    if rclpy.ok():
        release()
        # p1 = posj(0,-3.27,94.52,0,88.82,0) # 두번째 위
        # p2 = posj(0,-1.22,77.23,0,104.47,0) # 두번째 위
        # p3 = posj(0,-1.17,74.94,0,106.31,0) # 두번째 위
        # movej(p1, vel=VELOCITY, acc=ACC)
        # movej(p2, vel=VELOCITY, acc=ACC)
        # movej(p3, vel=VELOCITY, acc=ACC)
        movej(JReady, vel=VELOCITY, acc=ACC)
        p0 = posj(-0.79,28.3,47.26,-0.16,104.44,-0.59) # 초기 위
        movej(p0, v=VELOCITY, a=ACC)
        pick_pos = posx([550, 0, 220, 90, 180, 90])
        movel(pick_pos, vel=VELOCITY, acc=ACC)
        all_cup_count = check_floor()
        create_cup_top(3)
        last_cup_top()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
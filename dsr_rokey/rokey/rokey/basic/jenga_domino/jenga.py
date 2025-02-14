# pick and place in 1 method. from pos1 to pos2 
import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY, ACC = 45, 45

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node("rokey_simple_move", namespace = ROBOT_ID)
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
            check_force_condition,
            DR_AXIS_Z,
            release_compliance_ctrl,
            mwait,
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
        
    def move_down(depth):
        movel([0, 0, -depth, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        
    def move_up(height):
        movel([0, 0, height, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        
    def set_up_wood_bar(lay_down_pos,set_up_pos):
        movel(lay_down_pos, vel=VELOCITY, acc=ACC)
        move_down(25)
        wait(2.0)
        
        grip_with_delay()
        move_up(50)
        
        print("target moving....")
        movel(set_up_pos, vel=VELOCITY, acc=ACC)
        move_down(40)
        
        # 로봇이 아래 방향으로 줄 힘 설정
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        # 현재 로봇에 입력되는 힘 측정
        while not check_force_condition(DR_AXIS_Z, max=8):
            pass
        
        # 로봇이 아래 방향으로 주는 힘 초기화
        release_compliance_ctrl()
        release()
        move_up(30)
        
    JReady = [0, 0, 90, 0, 90, 0] #홈위치
    pos1 = posx([600, 0, 85, 90, 180, 90]) #5개의 높이 +5(바닥)
    pos2 = posx([400, 200, 120, 90, 90, 90])
    gap = 45 #도미노 간격
    
    lower_bound = 5
    wood_bar_height = 15 #도미노 한 개당 높이
    #wood_bar_length = 75 
    
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")
    
    
    if rclpy.ok(): #재정렬
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        current_pickup_height = pos1[2]
        
        # 계속 반복함
        for i in range(5): # 도미노 5개 배치
                # 픽업 위치
                pos1 = posx([pos1[0], pos1[1], current_pickup_height, pos1[3], pos1[4], pos1[5]])
                # 배치 위치
                pos2 = posx([pos2[0], pos2[1] - (gap), pos2[2], 90, 90, 90])
                print(f"domino placement {i+1} at {pos2}")
                set_up_wood_bar(pos1, pos2)
                current_pickup_height -= wood_bar_height
        
    rclpy.shutdown() #나가기
    
if __name__ == "__main__":
    main()
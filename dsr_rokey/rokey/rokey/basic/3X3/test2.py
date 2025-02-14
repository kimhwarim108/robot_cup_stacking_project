import rclpy
import DR_init

# 로봇 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
OFF, ON = 0, 1
h = 100

# 좌표 데이터
plate1 = [
    [562.06, 163.808, h, 4.687, -179.965, 4.471],
    [665.013, 164.9, h, 165.042, 179.992, 164.829],
    [616.017, 74.208, h, 1.114, -179.959, 0.882],
]

plate2 = [
    [594.069, -52.696, h, 6.142, -179.961, 5.902],
    [699.43, -46.041, h, 96.541, -179.99, 96.345],
    [651.668, -139.928, h, 179.108, 179.958, 178.914],
]

# 마지막 가운데 위치 좌표
middle_start = [614.023, 133.864, h, 4.083, -179.945, 3.873]  # plate1 가운데 위치
middle_end = [648.972, -79.882, h, 14.396, -179.985, 14.202]  # plate2 가운데 위치

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pick_and_place_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            task_compliance_ctrl,
            set_desired_force,
            check_force_condition,
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            amove_periodic,
            DR_MV_MOD_REL,
            wait,
            DR_AXIS_Z,
            release_compliance_ctrl,
            posj,
            DR_TOOL
        )
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2: {e}")
        return

    # 보조 함수들 정의
    def move_down(depth):
        """현재 위치에서 지정된 깊이만큼 아래로 이동"""
        movel([0, 0, -depth, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    def move_up(height):
        """현재 위치에서 지정된 높이만큼 위로 이동"""
        movel([0, 0, height, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

    def wait_digital_input(sig_num):
        """지정된 디지털 입력 신호가 들어올 때까지 대기"""
        while not get_digital_input(sig_num):
            wait(0.5)

    def grip_with_delay():
        """그리퍼로 물체를 집음"""
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    def release():
        """그리퍼로 물체를 놓음"""
        set_digital_output(2, ON)
        set_digital_output(1, OFF)

    def pick_and_place(pick_pos, place_pos):
        """주어진 위치들 간에 픽 앤 플레이스 작업 수행"""
        # plate1에서 물체 집기
        movel(pick_pos, vel=VELOCITY, acc=ACC)
        move_down(55)
        grip_with_delay()
        move_up(55)

        # plate2에 물체 놓기
        movel(place_pos, vel=VELOCITY, acc=ACC)
        move_down(55)
        release()
        move_up(55)

    def move_and_amove_periodic():
        """가운데 위치로 이동 후 주기적 모션을 수행하며 물체 놓기"""
        # 가운데 위치에서 물체 집기
        movel(middle_start, vel=VELOCITY, acc=ACC)
        move_down(55)
        grip_with_delay()
        move_up(55)

        # 가운데 위치로 물체 이동
        movel(middle_end, vel=VELOCITY, acc=ACC)

        # 힘 제어 및 주기적 모션 수행
        print("Setting compliance control...")
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

        print("Starting periodic motion...")
        amove_periodic(
            amp=[0, 0, -10, 0, 0, 0],  # Z축 방향으로 -10mm 진폭
            period=1,
            atime=0.5,
            repeat=3,
            ref=DR_TOOL
        )

        # 주기적 모션 후 대기 시간을 추가하여 로봇이 완전히 멈추도록 함
        wait(1.0)

        # 힘 조건을 체크하며 로그 출력 추가
        print("체크 완료")
        while not check_force_condition(DR_AXIS_Z, max=15):
            print("다시 시도")

        # 힘 제어 해제
        print("미션 완료.")
        release_compliance_ctrl()

        # 물체 놓기
        print("놓음")
        release()
        move_up(50)

    # 초기 동작 수행
    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")

    if rclpy.ok():
        release()
        # 초기 위치로 이동
        P0 = posj(0, 0, 90, 0, 90, 0)
        movej(P0, vel=VELOCITY, acc=ACC)

        # plate1의 모든 물체를 plate2로 옮기기
        for pick_pos, place_pos in zip(plate1, plate2):
            pick_and_place(pick_pos, place_pos)

        # 가운데 위치로 이동하여 주기적 모션 수행 및 물체 놓기
        move_and_amove_periodic()

    # ROS 종료
    rclpy.shutdown()

if __name__ == "__main__":
    main()

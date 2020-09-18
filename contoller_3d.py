import os
import time
import cv2
from k4a_marker_det import K4aMarkerDet
from xarm.wrapper import XArmAPI
from pynput.mouse import Button
from pynput.mouse import Listener as mListener
from pynput.keyboard import Key, KeyCode
from pynput.keyboard import Listener as kListener


def setup_xarm5():
    time.sleep(0.5)
    print('=' * 50)
    print("conneting xARM, please double check the settings, and make sure no human or obstacles in working area")
    os.system('pause')  # press any key to continue
    arm = XArmAPI('192.168.1.110', do_not_open=True, is_radian=False)
    #time.sleep(0.5)
    #if arm.warn_code != 0:
    #    arm.clean_warn()
    #if arm.error_code != 0:
    #    arm.clean_error()
    arm.motion_enable(enable=True)
    arm.set_mode(0) # 0 for position control mode. do not use other mode like 1 for servo motion mode, 2 for joint teaching mode
    arm.set_state(state=0) # 0: enable motion, 3: pause state, 4: stop state
    arm.reset(wait=True)
    print('version:', arm.get_version())
    print('state:', arm.get_state())  # 1 in motion, 2 ready but no motion(hibernate), 3 pause(could have instruction cache), 4 stop 
    print('err_warn_code:', arm.get_err_warn_code())
    print('position(°):', arm.get_position(is_radian=False))
    print('angles(°):', arm.get_servo_angle(is_radian=False))
    print('angles(°)(servo_id=1):', arm.get_servo_angle(servo_id=1, is_radian=False))

    code = arm.set_gripper_mode(0) # location mode, xarm gripper only support location mode for now
    print('set gripper mode: location mode, code={}'.format(code))
    code = arm.set_gripper_enable(True)
    print('set gripper enable, code={}'.format(code))
    code = arm.set_gripper_speed(500)  # unit:r/min
    print('set gripper speed, code={}'.format(code))
    code = arm.set_gripper_position(800, wait=True) # -10 to 850
    print('[wait]set gripper pos, code={}'.format(code))
    code, pos = arm.get_gripper_position() # tuple((code, pos)), only when code is 0, the returned result is correct
    print('get gripper pos, pos={}'.format(pos))
    return arm


if __name__ == "__main__":
    exiting_flag = 0
    kill_server = True
    arm = None
    enable_k4a_pos_control = False
    enable_k4a_gripper_control = False  # if this is false, use scroll funtion of mouse
    mouse_scroll_value = 0
    
    def on_move(x, y):
        pass

    def on_click(x, y, button, pressed):
        global enable_k4a_pos_control, enable_k4a_gripper_control
        print('{0} {1} at {2}'.format(button, 'Pressed' if pressed else 'Released', (x, y)))
        if pressed and button == Button.left:
            enable_k4a_pos_control = not enable_k4a_pos_control
            print('enable_k4a_pos_control: {0}'.format(enable_k4a_pos_control))
        if pressed and button == Button.middle:
            enable_k4a_gripper_control = not enable_k4a_gripper_control
            print('enable_k4a_gripper_control: {0}'.format(enable_k4a_gripper_control))

    def on_scroll(x, y, dx, dy):
        global arm, mouse_scroll_value
        mouse_scroll_value += dy
        if mouse_scroll_value < 0: mouse_scroll_value = 0
        if mouse_scroll_value > 10: mouse_scroll_value = 10
        gripper_val = mouse_scroll_value * 85
        if arm and arm.connected:
            code = arm.set_gripper_position(gripper_val, wait=False)
            print('set gripper pos, code={}'.format(code))
        print('Scrolled {0}, scroll value {1}'.format((dx, dy), mouse_scroll_value))

    def on_press(key):
        print('{0} pressed'.format(key))

    def on_release(key):
        global exiting_flag
        print('{0} release'.format(key))
        if isinstance(key, KeyCode):
            if key.char == 'q':
                exiting_flag = 2
        # if key is a special key
        if isinstance(key, Key):
            if key == Key.esc:
                exiting_flag = 1

    listener_m = mListener(on_move=on_move, on_click=on_click, on_scroll=on_scroll)
    listener_m.start()
    listener_k = kListener(on_press=on_press, on_release=on_release)
    listener_k.start()

    marker_detector = K4aMarkerDet(logging=True)

    arm = setup_xarm5()
    _, xarm5_init_pos = arm.get_position(is_radian=False)
    # could be [201.5, 0, 140.5, -180, 0, 0] for home position
    # xArm5 roll pitch should be roll=±180° pitch=0°。
    print(xarm5_init_pos)  
    while True:
        ir_bin, depth_img = marker_detector.request_image()
        marker_pos = marker_detector.get_marker_from_img(ir_bin, depth_img)
        if arm.connected and arm.get_cmdnum() <= 2:
            print('cmdnum:', arm.get_cmdnum())
            if enable_k4a_pos_control and len(marker_pos) >= 1:
                x, y, z = xarm5_init_pos[0] + marker_pos[0][0], xarm5_init_pos[1] + marker_pos[0][1], xarm5_init_pos[2] + marker_pos[0][2]
                arm.set_position(x=x, y=y, z=z, roll=180, pitch=0, yaw=0, radius=50, speed=100, mvacc=200, wait=False)  # speed in mm/s,rad/
            if enable_k4a_gripper_control and len(marker_pos) == 2:
                distance = np.linalg.norm(np.array(marker_pos[1][:2])-np.array(marker_pos[0][:2]))
                code = arm.set_gripper_position(min(distance, 849), wait=False) # -10 to 850
                print('set gripper pos, code={}'.format(code))
        time.sleep(0.01)
        # cv2.imshow("gray_bin", ir_bin)
        if exiting_flag == 1: # Esc key to stop
            break
        if exiting_flag == 2: # q key to exit without kill server process
            kill_server = False
            break

    print('=' * 50)
    print("reseting xARM, please wait..")
    arm.reset(wait=True)
    # arm.motion_enable(enable=False)
    arm.disconnect()
    marker_detector.close(kill_server=kill_server)
    listener_m.stop()
    listener_k.stop()


"""
标定时可以试试直接控制servo角度然后get position以获取正向运动学坐标:
speed = 20
arm.set_servo_angle(angle=[0, -30, -60, 0, 0], speed=speed, wait=True)
或者
speed = math.radians(20)
arm.set_servo_angle(angle=[0, math.radians(-30), math.radians(-60), 0, 0], speed=speed, is_radian=True, wait=True)
然后
print(arm.get_servo_angle(), arm.get_servo_angle(is_radian=True))
arm.get_position(is_radian=False)
_, angles = arm.get_servo_angle()

XARM5在关节空间时，机械臂有5个自由度可以控制，在需要末端使用其他姿态的情况下，可以切换至关节指令。而后再次使用关节指令回到法兰与底座水平的姿态即可以切换回笛卡尔控制。有一个快速的方法是：仅需要将J4的角度设为-(J2角度+J3角度)即可。

# 其他常用API
    arm.connect(...)
    arm.reset(...)
    arm.move_gohome(...)

    arm.set_position(...)
    arm.set_servo_angle(...)
    arm.set_servo_angle_j(...)
    arm.set_servo_cartesian(...)
    arm.set_position_aa(...)
    arm.set_servo_cartesian_aa(...)

    arm.set_tcp_offset(...)
    arm.set_servo_attach(...)
    arm.set_servo_detach(...)
    arm.set_state(...)
    arm.set_mode(...)
    arm.motion_enable(...)
    arm.set_pause_time(...)

    arm.set_tcp_jerk(...)
    arm.set_tcp_maxacc(...)
    arm.set_joint_jerk(...)
    arm.set_joint_maxacc(...)
    arm.set_tcp_load(...)
    arm.set_collision_sensitivity(...)
    arm.set_teach_sensitivity(...)
    arm.set_gravity_direction(...)
    arm.config_tgpio_reset_when_stop(...)
    arm.config_cgpio_reset_when_stop(...)
    arm.clean_conf()
    arm.save_conf()

    arm.set_gripper_enable(...)
    arm.set_gripper_mode(...)
    arm.set_gripper_speed(...)
    arm.set_gripper_position(...)
    arm.get_gripper_position()
    arm.get_gripper_err_code()
    arm.clean_gripper_error()
"""
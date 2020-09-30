import os
import time
import cv2
from pynput.mouse import Button
from pynput.mouse import Listener as mListener
from pynput.keyboard import Key, KeyCode
from pynput.keyboard import Listener as kListener
from k4a_marker_det import K4aMarkerDet
from arm_wrapper import DexArmWrapper, xArm5Wrapper

if __name__ == "__main__":
    exiting_flag = 0
    enable_k4a = False
    kill_server = True
    arm = None
    arm_init_pos = None
    pos_control_flag = 0  # 0 disable; 1 use mouse; 2 use kinect, mid key to switch 
    enable_k4a_gripper_control = False  # right key to switch, if this is false, use scroll funtion of mouse, other wise for height control
    mouse_scroll_value = 6  # 0-10, init with 6 wide open a little bit
    mouse_init_xy_val = [0,0]
    
    def on_move(x, y):
        global mouse_init_xy_val, pos_control_flag, arm_init_pos
        if pos_control_flag == 1:
            posx = x - mouse_init_xy_val[0]
            posy = y - mouse_init_xy_val[1]
            # map 1000 pixel to 5cm , 50mm
            posx, posy = posx/20.0, posy/20.0
            arm.set_pos(x=posx, y=posy)
            print(posx)
            print(posy)

    def on_click(x, y, button, pressed):
        global pos_control_flag, enable_k4a_gripper_control, mouse_init_xy_val
        print('{0} {1} at {2}'.format(button, 'Pressed' if pressed else 'Released', (x, y)))
        if pressed and button == Button.middle:
            pos_control_flag += 1
            if pos_control_flag == 3: pos_control_flag = 0
            print('pos_control_flag: {0}'.format(pos_control_flag))
            mouse_init_xy_val = [x, y]
        if pressed and button == Button.right:
            enable_k4a_gripper_control = not enable_k4a_gripper_control
            print('enable_k4a_gripper_control: {0}'.format(enable_k4a_gripper_control))

    def on_scroll(x, y, dx, dy):
        global arm, mouse_scroll_value
        if pos_control_flag == 1:
            mouse_scroll_value += dy*2
            if mouse_scroll_value < 0: mouse_scroll_value = 0
            if mouse_scroll_value > 10: mouse_scroll_value = 10
            if arm:
                code = arm.set_gripper(mouse_scroll_value / 10.0)
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
    if enable_k4a:
        marker_detector = K4aMarkerDet(logging=True)

    arm = DexArmWrapper() # xArm5Wrapper()
    arm.go_home()
    arm_init_pos = arm.get_pos()
    # could be [201.5, 0, 140.5, -180, 0, 0] for home position for xarm 5
    # xArm5 roll pitch should be roll=±180° pitch=0°。
    print(arm_init_pos)  
    while True:
        if enable_k4a:
            ir_bin, depth_img = marker_detector.request_image()
            marker_pos = marker_detector.get_marker_from_img(ir_bin, depth_img)
            if pos_control_flag == 2 and len(marker_pos) >= 1:
                x, y, z = arm_init_pos[0] + marker_pos[0][0], arm_init_pos[1] + marker_pos[0][1], arm_init_pos[2] + marker_pos[0][2]
                arm.set_pos(x=x, y=y, z=z)
            if enable_k4a_gripper_control and len(marker_pos) == 2:
                distance = np.linalg.norm(np.array(marker_pos[1][:2])-np.array(marker_pos[0][:2]))
                arm.set_gripper(distance/100.0) # normalized to 100mm, i.e., 10cm
        time.sleep(0.01)
        # cv2.imshow("gray_bin", ir_bin)
        if exiting_flag == 1: # Esc key to stop
            break
        if exiting_flag == 2: # q key to exit without kill server process
            kill_server = False
            break

    arm.close()
    if enable_k4a: marker_detector.close(kill_server=kill_server)
    listener_m.stop()
    listener_k.stop()

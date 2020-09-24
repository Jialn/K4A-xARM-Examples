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

    marker_detector = K4aMarkerDet(logging=True)

    arm = xArm5Wrapper()
    arm_init_pos = arm.get_pos()
    # could be [201.5, 0, 140.5, -180, 0, 0] for home position for xarm 5
    # xArm5 roll pitch should be roll=±180° pitch=0°。
    print(arm_init_pos)  
    while True:
        ir_bin, depth_img = marker_detector.request_image()
        marker_pos = marker_detector.get_marker_from_img(ir_bin, depth_img)
        if enable_k4a_pos_control and len(marker_pos) >= 1:
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
    marker_detector.close(kill_server=kill_server)
    listener_m.stop()
    listener_k.stop()

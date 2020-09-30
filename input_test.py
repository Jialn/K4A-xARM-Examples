import cv2
from k4a_marker_det import K4aMarkerDet
from pynput.mouse import Listener as mListener
from pynput.keyboard import Key, KeyCode
from pynput.keyboard import Listener as kListener

if __name__ == "__main__":
    exiting_flag = 0
    kill_server = True
    
    def on_move(x, y):
        pass

    def on_click(x, y, button, pressed):
        print('{0} {1} at {2}'.format(button, 'Pressed' if pressed else 'Released', (x, y)))

    def on_scroll(x, y, dx, dy):
        print('Scrolled {0}'.format((dx, dy)))

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

    while True:
        ir_bin, depth_img = marker_detector.request_image()
        marker_pos = marker_detector.get_marker_from_img(ir_bin, depth_img)
        # cv2.imshow("gray_bin", ir_bin)
        if exiting_flag == 1: # Esc key to stop
            break
        if exiting_flag == 2: # q key to exit without kill server process
            kill_server = False
            break

    marker_detector.close(kill_server=kill_server)
    listener_m.stop()
    listener_k.stop()




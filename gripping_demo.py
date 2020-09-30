import os
import time
import cv2
import numpy as np
from k4a_marker_det import K4aMarkerDet
from arm_wrapper import DexArmWrapper, xArm5Wrapper

def transform_to_arm_base(pos, offset=None):
    transformed_pos = np.array([pos[0], -pos[2], -pos[1]])
    if offset is not None:
        transformed_pos += offset
    return transformed_pos

if __name__ == "__main__":
    exiting_flag = 0
    enable_k4a = False
    arm = None
    arm_init_pos = None

    
    marker_detector = K4aMarkerDet(logging=False)

    arm = DexArmWrapper() # xArm5Wrapper()
    arm.go_home()
    time.sleep(2)
    arm_init_pos = arm.get_pos()
    print(arm_init_pos)
    ir_bin, depth_img = marker_detector.request_image()
    marker_pos = marker_detector.get_marker_from_img(ir_bin, depth_img)
    cv2.imshow("gray_bin", ir_bin)
    
    if len(marker_pos) != 2:
        print("no marker found, cali failed")
        arm.close()
        marker_detector.close()
        exit()
    init_arm_pos = transform_to_arm_base(marker_pos[1][:3])
    arm_to_marker_offset = np.array([0, 30, 20]) # marker to arm tip offset
    trans_offset = - init_arm_pos
    obj_pos = transform_to_arm_base(np.array(marker_pos[0][:3]), trans_offset) + arm_to_marker_offset
    arm_pos = transform_to_arm_base(np.array(marker_pos[1][:3]), trans_offset)
    control_pos = obj_pos
    while True:
        ir_bin, depth_img = marker_detector.request_image()
        marker_pos = marker_detector.get_marker_from_img(ir_bin, depth_img)
        # translate marker pose to robot arm pose
        if len(marker_pos) == 2:
            obj_pos = transform_to_arm_base(np.array(marker_pos[0][:3]), trans_offset) + arm_to_marker_offset
            arm_pos = transform_to_arm_base(np.array(marker_pos[1][:3]), trans_offset)
            error_fb = obj_pos - arm_pos
            distance = np.linalg.norm(error_fb)
            print((arm_pos, obj_pos, distance))
            control_pos = control_pos - 0.2 * error_fb
            if distance > 15:
                arm.set_pos(obj_pos[0],obj_pos[1],obj_pos[2], wait=True)
            # arm.set_gripper(distance/100.0) # normalized to 100mm, i.e., 10cm

        # cv2.imshow("gray_bin", ir_bin)
        key = cv2.waitKey(5)
        if key == 27: # Esc key to stop
            break

    arm.close()
    marker_detector.close()

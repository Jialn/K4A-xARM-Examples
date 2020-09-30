import numpy as np
import cv2
import pyk4a
from pyk4a import Config, PyK4A

# parameters 
FRAME_WIDTH = 640  # The image size of depth/ir assuming depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED, change it otherwise
FRAME_HEIGHT = 576
# parameters for contours filter
contours_area_range = [15, 1200] # in num of pixel, 25 at 1.2m, 400 about 25cm
z_range = [150, 1300]  # a bit larger than 25cm - 1.2m
z_center_offset = 700
volume_factor_range = [3000, 12000] # z*sqrt(area), in mm*pixel_num depends on size of marker
edge_margin = 10

class K4aMarkerDet():
    """ Class for tracking a passive Marker 3D position using Azure Kinect
    The marker should be made of retro-reflecting material, ~ 1*1 cm^2 size
    IR power of kinect should be dimmed
    """

    def __init__(self,
                 logging=False):
        """
        Init the kinect.

        Args:
            logging (bool): log or not.
        """
        self._logging = logging
        self._k4a = self.init_k4a()

    def init_k4a(self):
        k4a = PyK4A(Config(color_resolution=pyk4a.ColorResolution.RES_720P, depth_mode=pyk4a.DepthMode.NFOV_UNBINNED,))
        k4a.start()
        return k4a


    def request_image(self):
        capture = self._k4a.get_capture()
        # Reshape to image
        depth_img_full = np.frombuffer(capture.depth, dtype=np.uint16).reshape(FRAME_HEIGHT, FRAME_WIDTH).copy()
        ir_img_full = np.frombuffer(capture.ir, dtype=np.uint16).reshape(FRAME_HEIGHT, FRAME_WIDTH).copy()
        # Type convert and threshold to ir
        depth_img = depth_img_full.astype(np.float32)  # in milimeter
        ir_img = np.array(ir_img_full // 64, np.uint8)
        thres, ir_img = cv2.threshold(ir_img, 10, 255, cv2.THRESH_BINARY)
        return ir_img, depth_img


    def get_marker_from_img(self, gray_bin, depth_img):
        marker_results = []
        # find contours
        contours, h = cv2.findContours(gray_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # if contours exits, run the filter
        if (len(contours) >= 1):
            marker_contours = []
            for marker_idx in range(len(contours)):
                area_i = cv2.contourArea(contours[marker_idx])
                if contours_area_range[0] < area_i < contours_area_range[1]:
                    cntou = contours[marker_idx]
                    moments = cv2.moments(cntou) # to extract centroid
                    if moments['m00'] != 0:
                        cxf = int(moments['m10'] / moments['m00'])
                        cyf = int(moments['m01'] / moments['m00'])
                        cx, cy = int(cxf), int(cyf)
                        if 2 < cx < (FRAME_WIDTH-edge_margin) and 2 < cy < (FRAME_HEIGHT-edge_margin):
                            czm0 = np.median(depth_img[cy - 2][cx - 2:cx + 1])
                            czm01 = np.median(depth_img[cy - 1][cx - 3:cx + 2])
                            czm1 = np.median(depth_img[cy][cx - 3:cx + 2])
                            czm2 = np.median(depth_img[cy + 1][cx - 3:cx + 2])
                            czm21 = np.median(depth_img[cy + 2][cx - 2:cx + 1])
                            cz = np.median((czm0, czm01, czm1, czm2, czm21))
                            volume_factor = np.sqrt(area_i) * cz
                            if volume_factor_range[0] < volume_factor < volume_factor_range[1] and z_range[0] < cz < z_range[1]:
                                cxf -= FRAME_WIDTH / 2.0
                                cyf -= FRAME_HEIGHT / 2.0
                                result_i = cxf, cyf, cz-z_center_offset, area_i
                                if self._logging:
                                    print("idx:" + str(marker_idx) + " xyz:" + str(result_i) + " vfactor:" + str(volume_factor))
                                marker_results.append(result_i)
                                marker_contours.append(contours[marker_idx])
            if len(marker_results) > 0:
                marker_results.sort(key=lambda x:x[-1], reverse=True) # sort by area in reversed order
                if self._logging:
                    cv2.drawContours(gray_bin, marker_contours, 0, 128, -1)
        return marker_results


    def get_marker():
        ir_bin, depth_img = self.k4a_marker_det.request_image()
        return self.get_marker_from_img(ir_bin, depth_img)
        

    def close(self):
        self._k4a.stop()


# test for k4a marker detection
if __name__ == "__main__":
    k4a_marker_det = K4aMarkerDet(logging=True)
    while True:
        ir_bin, depth_img = k4a_marker_det.request_image()
        marker_pos = k4a_marker_det.get_marker_from_img(ir_bin, depth_img)
        cv2.imshow("gray_bin", ir_bin)
        key = cv2.waitKey(2)
        if key == 27: # Esc key to stop
            k4a_marker_det.close()
            break


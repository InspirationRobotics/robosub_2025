"""
    Provides conversions between OpenCV and ROS image formats in a hard-coded way.  
    CV_Bridge, the module usually responsible for doing this, is not compatible with Python 3,
     - the language this all is written in.  So we create this module, and all is... well, all is not well,
     - but all works.  :-/
     https://zhuanlan.zhihu.com/p/652106267
"""
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image


class CvBridge:
    def __init__(self):
        self.haswarned = False
        
    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8")  # Hardcode to 8 bits
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')

        image_opencv = np.ndarray(
            shape=(img_msg.height, img_msg.width, 3),  # 3 channels
            dtype=dtype,
            buffer=img_msg.data
        )

        # Byte order fix
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()

        # Handle encoding
        if img_msg.encoding == "rgb8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
        elif img_msg.encoding != "bgr8":
            if not self.haswarned:
                rospy.logwarn(f"Unsupported encoding '{img_msg.encoding}'. This node assumes 'bgr8'. Image may not be interpreted correctly.")
                self.haswarned = True
            

        return image_opencv

    def cv2_to_imgmsg(self,cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg
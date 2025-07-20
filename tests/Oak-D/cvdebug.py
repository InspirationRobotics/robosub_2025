
import rospy
import time
from auv.utils.img_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

b = CvBridge()
rospy.init_node("cv_debugger", anonymous=True)

def callbacks(msg):
    try:
        cv_image = b.imgmsg_to_cv2(msg)
        rospy.loginfo('success')
        cv2.imshow("debug",cv_image)
    except Exception as e:
        rospy.logerr(e)
        return
    cv2.waitKey(1)

sub = rospy.Subscriber('/auv/camera/videoOAKdRawForward', Image, callbacks)
rospy.spin()
cv2.destroyAllWindows()

import roslib
import sys
import rospy
import cv2
import cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class VehicleCamera(object):
    
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_cb)
    
    def camera_cb(self, data):
        
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(e)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

def main():
    vehicle_camera_view = VehicleCamera()
    ospy.init_node('vehicle_camera_view_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
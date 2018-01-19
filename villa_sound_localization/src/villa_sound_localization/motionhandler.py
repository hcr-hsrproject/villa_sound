import rospy
import math
from hark_msgs.msg import HarkSource
from geometry_msgs.msg import PoseStamped
import tf

class MotionHandler(object):

    """
    start_motion_callback: motionhandler -> Bool is called before motion starts and should return whether to move or not
    end_motion_callback: motionhandler -> () is called after motion ends
    """
    def __init__(self, omni_base, start_motion_callback = lambda x: x, end_motion_callback = lambda x: x):
        rospy.Subscriber("/HarkSource", HarkSource, self.__localization_callback, queue_size=1)
        rospy.Subscriber("/hsrb/base_pose/", PoseStamped, self.__angle_callback)
        self.omni_base = omni_base
        self.start_motion_callback = start_motion_callback
        self.end_motion_callback = end_motion_callback
        self.prev_azimuth = 0.
        self.move = False
        self.orig_angle = None
        print ("ORIGINAL ANGLE:", self.orig_angle)
        self.curr_angle = 0


    """
    Tell the motion handler to ignore localization stream
    """
    def stop(self):
        self.move = False


    """
    Tell the motion handler to follow localization stream
    """
    def start(self):
        self.move = True


    def move_robot(self, azimuth):
        print("Begin moving robot")
        self.omni_base.go(0., 0., math.pi * azimuth / 180., 100., relative=True)

    def __angle_callback(self, data):
        quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.curr_angle = (euler[2] * (180./math.pi)) + 180
        if self.orig_angle == None:
            self.orig_angle = (euler[2] * (180./math.pi)) + 180

    def is_between(self, angle1, angle2, angle3):
        if angle1 < angle2:
            if angle1 < angle3 and angle3 < angle2:
                return True
            else:
                return False
        else:
            if angle3 > angle1 or angle3 < angle2:
                return True
            else:
                return False


    def __localization_callback(self, data):
        if len(data.src) <= 0:
            return

        # Get max power source
        src = max(data.src, key=lambda x: x.power)

        azimuth = src.azimuth

        #if abs(azimuth - self.prev_azimuth) < 10.0:
            #print("REJECTED NEW ANGLE SINCE TOO CLOSE")
        #    return

        print("Detected source with power: " + str(src.power))

        if self.move:
            if self.is_between((self.orig_angle - 90)%180, (self.orig_angle + 90)%180, (self.curr_angle + 360 - (abs(azimuth - self.prev_azimuth)%360))%180):
                should_move = self.start_motion_callback(self)
                if should_move:
                    self.move_robot(azimuth)
                    self.end_motion_callback(self)
                else:
                    print("Handler told me not to move")
            else:
                print("Not turning because angle is out of range")

        self.prev_azimuth = azimuth

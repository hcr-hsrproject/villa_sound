from . import settings
import subprocess
import os.path
import rospy
from std_msgs.msg import Int32

"""
Generates correlation matrix for GEVD-MUSIC to whiten noise
"""
class Noise(object):

    def __init__(self):
        self.noise_calibration = None
        self.stopper = rospy.Publisher('/villa/sound_localization/noise/internal/stop', Int32, queue_size=1)


    """
    Start listening to noise
    TODO: Check if already running
    """
    def start(self):
        if self.noise_calibration is not None:
            rospy.logerr('Noise calibration already running!')
        else:
            args = [os.path.join(settings.NETWORK_FILES_DIR, 'noise_calibration.n'),
                    os.path.join(settings.DATA_DIR, 'NOISEr.noise'),
                    os.path.join(settings.DATA_DIR, 'NOISEi.noise')]
            self.noise_calibration = subprocess.Popen(args)


    """
    Stop listening and generate correlation matrix
    """
    def stop(self):
        if self.noise_calibration is None:
            rospy.logerr('Noise calibration is not running!')
        else:
            self.stopper.publish(1)
            import time; time.sleep(3) # THIS IS TERRIBLE - TODO: Check when calibration file is modified
            self.noise_calibration.terminate()
            self.noise_calibration = None


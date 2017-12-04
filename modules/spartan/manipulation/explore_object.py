# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils
from spartan.utils.taskrunner import TaskRunner

# ROS
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

# ROS custom
import robot_msgs.srv

class ExploreObject(object):

    def __init__(self, robotSystem, handFrame='palm', removeFloatingBase=True, cameraSerialNumber=1112170110):
        self.robotSystem = robotSystem
        self.jointNames = self.robotSystem.ikPlanner.robotModel.model.getJointNames()
        if removeFloatingBase:
            self.jointNames = self.jointNames[6:]
        self.robotService = spartanROSUtils.RobotService(self.jointNames)
        self.handFrame = handFrame
        self.cameraSerialNumber = cameraSerialNumber

        # self.taskRunner = TaskRunner()

    def move(self, q):
    	maxJointDegreesPerSecond = 15
    	self.robotService.moveToJointPosition(q, maxJointDegreesPerSecond)

    def getCurrentJointPosition(self):
    	return self.robotSystem.robotStateJointController.q

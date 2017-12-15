# director
# probably need to use ros tf now instead of this
# from director import transformUtils

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils
# from spartan.utils.taskrunner import TaskRunner

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

# ROS custom
import robot_msgs.srv

class ExploreObject(object):

    def __init__(self, handFrame='palm', stopChannel="/stop", removeFloatingBase=True, cameraSerialNumber=1112170110):
        self.jointNames = (u'iiwa_joint_1', u'iiwa_joint_2', u'iiwa_joint_3', u'iiwa_joint_4', u'iiwa_joint_5', u'iiwa_joint_6', u'iiwa_joint_7')
        self.robotService = spartanROSUtils.RobotService(self.jointNames)
        self.handFrame = handFrame
        self.cameraSerialNumber = cameraSerialNumber
        self.removeFloatingBase = removeFloatingBase
        self.maxJointDegreesPerSecond = 15
        self.currentJointPosition = [0]*len(self.jointNames)

        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.storeCurrentJointPosition)
        rospy.Subscriber(stopChannel, std_msgs.msg.Bool, self.stopCurrentPoint)

    def stopCurrentPoint(self, msg):
        if msg.data:
            rospy.loginfo("stopping robot at position:")
            rospy.loginfo(self.currentJointPosition)
            self.robotService.moveToJointPosition(self.currentJointPosition, self.maxJointDegreesPerSecond)
            # stop the robot
            # move on to the next point

    def storeCurrentJointPosition(self, msg):
    	self.currentJointPosition = msg.position

    def getCurrentJointPosition(self):
        return self.currentJointPosition

    def moveJoint(self, jointIndex, q):
        nextPosition = self.getCurrentJointPosition()
        nextPosition[jointIndex] = q
        self.taskRunner.callOnThread(self.robotService.moveToJointPosition, nextPosition, self.maxJointDegreesPerSecond)

    def moveToPoint(self, x, y, z):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = transformUtils.rollPitchYawToQuaternion([0, -3.14/2, 0])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = pose
        poseStamped.header.frame_id = "base"

        response = self.robotService.runIK(poseStamped)
        if not response.success:
            rospy.loginfo("ik was not successful, returning without moving robot")
            return

        rospy.loginfo("ik was successful, moving to joint position")
        self.robotService.moveToJointPosition(response.joint_state.position, self.maxJointDegreesPerSecond)
        # self.robotService.moveToCartesianPosition(poseStamped, self.maxJointDegreesPerSecond)

    '''
    Robot first goes to starting point, and then returns there after every
    point in the given list

    Each point is a list of 3 numbers for x, y, and z
    This is temporary for testing until I know the format of points
    calculated from the point cloud.
    '''
    def exploreObject(self, starting_point, points_to_touch):
        self.moveToPoint(starting_point[0], starting_point[1], starting_point[2])

        for point in points_to_touch:
            self.moveToPoint(point[0], point[1], point[2])

            self.moveToPoint(starting_point[0], starting_point[1], starting_point[2])

    def testExplore(self):
        start = [0.39, -0.12, 0.69]
        point_a = [0.61, -0.1, 0.1]
        point_b = [0.61, 0.1, 0.1]
        points = [point_a, point_b]
        self.taskRunner.callOnThread(self.exploreObject, start, points)

# Just for testing standalone code

def main():
    rospy.init_node('explore_object_node')
    start = [0.39, -0.12, 0.69]
    point_a = [0.61, -0.1, 0.1]
    point_b = [0.61, 0.1, 0.1]
    points = [point_a, point_b]

    explore = ExploreObject()
    # while not rospy.is_shutdown():
    # 	rospy.sleep(0.1)
    
    rospy.spin()

if __name__ == "__main__":
    main()

'''
Joint names passed into the ExploreObject class
(u'iiwa_joint_1', u'iiwa_joint_2', u'iiwa_joint_3', u'iiwa_joint_4', u'iiwa_joint_5', u'iiwa_joint_6', u'iiwa_joint_7')
'''
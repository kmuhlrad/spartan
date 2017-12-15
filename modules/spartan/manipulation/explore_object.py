# director
from director import transformUtils

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils
from spartan.utils.taskrunner import TaskRunner

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
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
        self.removeFloatingBase = removeFloatingBase
        self.maxJointDegreesPerSecond = 15

        self.taskRunner = TaskRunner()

    def getCurrentJointPosition(self):
        if self.removeFloatingBase:
            return self.robotSystem.robotStateJointController.q[6:]
        return self.robotSystem.robotStateJointController.q

    def moveJoint(self, jointIndex, q):
        nextPosition = self.getCurrentJointPosition()
        nextPosition[jointIndex] = q
        self.taskRunner.callOnThread(self.robotService.moveToJointPosition, nextPosition, self.maxJointDegreesPerSecond)

    def moveToPoint(self, x, y, z):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        [0, 0, -3.14/2]

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

# def main():
#     rospy.init_node('explore_object_node')
#     start = [0.39, -0.12, 0.69]
#     point_a = [0.61, -0.1, 0.1]
#     point_b = [0.61, 0.1, 0.1]
#     points = [point_a, point_b]

#     explore = ExploreObject()
#     rospy.spin()

# # if __name__ == "__main__":
# #     main()
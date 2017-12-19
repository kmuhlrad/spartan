# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

# ROS custom
import robot_msgs.srv

# LCM
import lcm
from robotlocomotion import robot_plan_t

class ExploreObject(object):

    def __init__(self, handFrame='palm', stopChannel="/stop", removeFloatingBase=True, cameraSerialNumber=1112170110):
        self.jointNames = (u'iiwa_joint_1', u'iiwa_joint_2', u'iiwa_joint_3', u'iiwa_joint_4', u'iiwa_joint_5', u'iiwa_joint_6', u'iiwa_joint_7')
        self.robotService = spartanROSUtils.RobotService(self.jointNames)
        self.handFrame = handFrame
        self.cameraSerialNumber = cameraSerialNumber
        self.removeFloatingBase = removeFloatingBase
        self.maxJointDegreesPerSecond = 15
        self.currentJointPosition = [0]*len(self.jointNames)
        self.currentPointIndex = 0

        # DON'T CHEAT LIKE THIS
        # pass in a list of these things to the constructor and then run a method to find the
        # joint positions before doing anything else.
        self.startCartesian = [0.39, -0.12, 0.69]
        pointACartesian = [0.61, -0.1, 0.1]
        pointBCartesian = [0.61, 0.1, 0.1]
        self.touchPointsCartesian = [pointACartesian, pointBCartesian]

        self.startJoints = (-0.12058246347243111, 0.023127143399094278, -0.18276864549751937, -1.4576494043747763, 0.004569309154218585, 1.6602323914142667, 2.838266963907831)
        pointAJoint = (-2.6611123996571506, -1.5782094055315747, 1.909011570774591, -1.3806518913076944, 1.512714133799286, 1.2366488088279932, 2.249443804497624)
        pointBJoint = (2.661198963675827, -1.5779888027013913, -1.909305574590655, -1.3806423678606443, -1.5124293697415148, 1.2364375033534523, -2.249490972237493)
        self.touchPointsJoints = [pointAJoint, pointBJoint]

        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.storeCurrentJointPosition)
        rospy.Subscriber(stopChannel, std_msgs.msg.Bool, self.stopCurrentPoint)

    def stopCurrentPoint(self, msg):
        if msg.data:
            rospy.loginfo("stopping robot")
            lcm_msg = robot_plan_t()
            lcm_msg.utime = 0
            lc = lcm.LCM()
            lc.publish("STOP", lcm_msg.encode())
            
            # # stop the robot
            # # self.robotService.moveToJointPosition(self.startJoints, self.maxJointDegreesPerSecond)
            # self.robotService.moveToJointPosition(self.currentJointPosition, self.maxJointDegreesPerSecond)
            # rospy.loginfo("stopping robot and returning to home position")
            
            # # move on to the next point
            # rospy.loginfo("moving to next point")
            # self.currentPointIndex += 1
            # if self.currentPointIndex < len(self.touchPointsJoints):
            #     exploreObjectStartingFromIndex(self.currentPointIndex)
            
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

    def setJointVelocities(self, velocities):
        rospy.wait_for_service('/robot_control/SendJointTrajectory')
        s = rospy.ServiceProxy('robot_control/SendJointTrajectory', robot_msgs.srv.SendJointTrajectory)

        traj = trajectory_msgs.msg.JointTrajectory()

        traj.joint_names = self.jointNames

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.velocities = velocities # I might still need to set the position to the current joint position

        traj.points.append(p)

        response = s(traj)

        if not response.success:
            rospy.loginfo("trajectory was not successful, returning without moving robot")
            return

        rospy.loginfo("trajectory was successful, moving to joint velocity")

    '''
    Robot first goes to starting point, and then returns there after every
    point in the given list

    Each point is a list of 3 numbers for x, y, and z
    This is temporary for testing until I know the format of points
    calculated from the point cloud.
    '''
    def exploreObject(self):
        self.robotService.moveToJointPosition(self.startJoints, self.maxJointDegreesPerSecond)

        for point in self.touchPointsJoints:
            self.robotService.moveToJointPosition(point, self.maxJointDegreesPerSecond)
            rospy.loginfo("joint position:")
            rospy.loginfo(point)
            self.currentPointIndex += 1
            self.robotService.moveToJointPosition(self.startJoints, self.maxJointDegreesPerSecond)

    def exploreObjectStartingFromIndex(self, index):
        self.robotService.moveToJointPosition(self.startJoints, self.maxJointDegreesPerSecond)

        for i in range(self.currentPointIndex, len(self.touchPointsJoints)):
            self.robotService.moveToJointPosition(self.touchPointsJoints[i], self.maxJointDegreesPerSecond)
            rospy.loginfo("joint position:")
            rospy.loginfo(point)
            self.currentPointIndex += 1
            self.robotService.moveToJointPosition(self.startJoints, self.maxJointDegreesPerSecond)

# Just for testing standalone code

def main():
    rospy.init_node('explore_object_node')
    start = [0.39, -0.12, 0.69]
    point_a = [0.61, -0.1, 0.1]
    point_b = [0.61, 0.1, 0.1]
    points = [point_a, point_b]

    explore = ExploreObject()
    # explore.exploreObject()
    # while not rospy.is_shutdown():
    #   rospy.sleep(0.1)
    
    rospy.spin()

if __name__ == "__main__":
    main()

'''
Joint names passed into the ExploreObject class
(u'iiwa_joint_1', u'iiwa_joint_2', u'iiwa_joint_3', u'iiwa_joint_4', u'iiwa_joint_5', u'iiwa_joint_6', u'iiwa_joint_7')
'''
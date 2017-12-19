# system
from numpy import pi

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as spartanROSUtils

# ROS
import rospy
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from tf.transformations import quaternion_from_euler

# LCM
import lcm
from robotlocomotion import robot_plan_t

class ExploreObject(object):

    def __init__(self, homeCartesianPoint, touchCartesianPoints):
        self.jointNames = (u'iiwa_joint_1', u'iiwa_joint_2', u'iiwa_joint_3', u'iiwa_joint_4', u'iiwa_joint_5', u'iiwa_joint_6', u'iiwa_joint_7')
        self.robotService = spartanROSUtils.RobotService(self.jointNames)
        self.maxJointDegreesPerSecond = 15
        self.currentJointPosition = [0]*len(self.jointNames)

        self.homeJointPosition = self.getJointPositions(homeCartesianPoint)
        self.touchJointPositions = [self.getJointPositions(point) for point in touchCartesianPoints]

        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.storeCurrentJointPosition)
        rospy.Subscriber("/stop", std_msgs.msg.Bool, self.stopCurrentPoint)

    def storeCurrentJointPosition(self, msg):
        self.currentJointPosition = msg.position
        
    def stopCurrentPoint(self, msg):
        if msg.data:
            rospy.loginfo("stopping robot")
            lcm_msg = robot_plan_t()
            lcm_msg.utime = 0
            lc = lcm.LCM()
            lc.publish("STOP", lcm_msg.encode())

    '''
    cartesian_point is a list of 3 floats for x, y, and z (ex: [0.1, 0.2, 0.3])
    This format can be changed to reflect how the points are estimated from the point cloud.
    '''
    def getJointPositions(self, cartesian_point):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = cartesian_point[0]
        pose.position.y = cartesian_point[1]
        pose.position.z = cartesian_point[2]

        # The gripper should be pointing straight down.
        quat = quaternion_from_euler(0, pi/2, 0)
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

        rospy.loginfo("ik was successful, returning joint position")
        return response.joint_state.position

    '''
    The robot first goes to starting point, then goes towards the first point in the touch points list,
    and then returns to the starting point before moving on to the next touch point.
    '''
    def exploreObject(self):
        self.robotService.moveToJointPosition(self.homeJointPosition, self.maxJointDegreesPerSecond)

        for point in self.touchJointPositions:
            self.robotService.moveToJointPosition(point, self.maxJointDegreesPerSecond)

            self.robotService.moveToJointPosition(self.homeJointPosition, self.maxJointDegreesPerSecond)

def main():
    rospy.init_node('explore_object_node')
    homePoint = [0.39, -0.12, 0.69]
    pointA = [0.61, -0.1, 0.2]
    pointB = [0.61, 0.1, 0.2]
    touchPoints = [pointA, pointB]

    explore = ExploreObject(homePoint, touchPoints)
    explore.exploreObject()
    
    rospy.spin()

if __name__ == "__main__":
    main()
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

class ExploreDrawer(object):

    def __init__(self, start_x, start_y, desired_z, drawer_width, drawer_height, end_effector_width, end_effector_height):
        self.jointNames = (u'iiwa_joint_1', u'iiwa_joint_2', u'iiwa_joint_3', u'iiwa_joint_4', u'iiwa_joint_5', u'iiwa_joint_6', u'iiwa_joint_7')
        self.robotService = spartanROSUtils.RobotService(self.jointNames)
        self.currentJointPosition = [0]*len(self.jointNames)

        self.maxJointDegreesPerSecond = 15

        self.start_x = start_x
        self.start_y = start_y
        self.desired_z = desired_z

        self.should_stop = False

        self.num_rows, self.num_cols, self.cell_width, self.cell_height = self.calcGridInfo(drawer_width, drawer_height, end_effector_width, end_effector_height)

        grid_goal_points = self.calcGridGoalPoints()


        knot_points = self.calcCartesianKnotPoints(grid_goal_points)

        all_knot_points = [self.getJointPositions(point) for point in knot_points]
        self.joint_knot_points = [point for point in all_knot_points if point is not None]

        print len(all_knot_points), len(self.joint_knot_points)

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

            self.should_stop = True

    def calcGridInfo(self, drawer_width, drawer_height, end_effector_width, end_effector_height):
        num_cols = int(drawer_width / end_effector_width)
        num_rows = int(drawer_height / end_effector_height)

        cell_width = float(drawer_width) / num_cols
        cell_height = float(drawer_height) / num_rows

        return num_rows, num_cols, cell_width, cell_height

    def calcGridGoalPoints(self):
        goal_points = []

        if self.num_cols % 2 == 0:
            last_point = self.num_cols - 1
        else:
            last_point = self.num_rows*self.num_cols - 1

        point = 0
        direction = 1
        goal_points.append(point)

        while point != last_point:
            point += direction * self.num_rows * self.num_cols
            goal_points.append(point)

            if point == last_point:
                break
            
            point += 1
            goal_points.append(point)

            direction = -direction
        
        return goal_points

    def calcCartesianKnotPoints(self, grid_goal_points):
        knot_points = []

        for point in grid_goal_points:
            row_idx, col_idx = self.getRowColIdx(point, self.num_cols)

            x = self.start_x + self.cell_width*col_idx + self.cell_width/2.0
            y = self.start_y - self.cell_height*row_idx - self.cell_height/2.0
            z = self.desired_z

            knot_points.append([x, y, z])

        return knot_points

    def getRowColIdx(self, cell, num_cols):
        row_idx = 0
        cur_cell = cell

        while cur_cell > num_cols - 1:
            cur_cell -= num_cols
            row_idx += 1

        col_idx = cell - row_idx*num_cols

        return row_idx, col_idx

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

    def exploreDrawer(self):
        for point in self.joint_knot_points:
            if self.should_stop:
                break
            self.robotService.moveToJointPosition(point, self.maxJointDegreesPerSecond)
        print "done sweeping"

'''
How to run this demo:

Once spartan is running, open up a new terminal window (remember to run use_ros and use_spartan
in each new window you open). In the main spartan directory, run

python modules/spartan/manipulation/explore_object_standalone.py

This will start the robot through the demo of altenating between going to the given home
point and every point in the touch points list. If at any time you want to stop the robot from
going to the current point and want it to move on to the next point, open up a new terminal window
and run

rostopic pub /stop std_msgs/Bool true

If you want to send this command programmatically, make a publisher that publishes a boolean message
to the /stop topic with data=true.



Things to change in this file:

- The points that are hardcoded in the main function below can be read in from anywhere else, including
from a ROS topic or a ROS param file.

- The getJointPositions method takes in the points as a list of 3 numbers, but that can be changed
to match whatever format the points are input in.

- maxJointDegreesPerSecond should be tuned based on testing on the real robot.

'''
def main():
    rospy.init_node('explore_object_node')

    start_x = 0.355
    start_y = -0.29
    desired_z = 0.93

    drawer_width = 0.49177
    drawer_height = 0.60329

    end_effector_width = 0.012285
    end_effector_height = 0.019265

    explore = ExploreDrawer(start_x, start_y, desired_z, drawer_width, drawer_height, end_effector_width, end_effector_height)
    explore.exploreDrawer()
    
    rospy.spin()

if __name__ == "__main__":
    main()
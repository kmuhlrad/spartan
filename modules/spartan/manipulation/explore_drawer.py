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

import pdb

class ExploreDrawer(object):

    def __init__(self, start_x, start_y, desired_z, drawer_width, drawer_height, end_effector_width, end_effector_height, num_waypoints = 0):
        self.jointNames = (u'iiwa_joint_1', u'iiwa_joint_2', u'iiwa_joint_3', u'iiwa_joint_4', u'iiwa_joint_5', u'iiwa_joint_6', u'iiwa_joint_7')
        self.robotService = spartanROSUtils.RobotService(self.jointNames)
        self.currentJointPosition = [0]*len(self.jointNames)

        self.maxJointDegreesPerSecond = 15

        self.start_x = start_x
        self.start_y = start_y
        self.desired_z = desired_z

        self.should_stop = False
        self.should_switch_modes = False

        self.num_rows, self.num_cols, self.cell_width, self.cell_height = self.calcGridInfo(drawer_width, drawer_height, end_effector_width, end_effector_height)

        grid_goal_points, directions = self.calcGridGoalPoints(num_waypoints)

        #print grid_goal_points

        knot_points = self.calcCartesianKnotPoints(grid_goal_points)

        self.all_knot_points = [self.getJointPositions(point, directions[i]) for (i, point) in enumerate(knot_points)]
        self.joint_knot_points = [point for point in self.all_knot_points if point is not None]
        self.directions = [directions[i] for i in range(len(self.all_knot_points)) if self.all_knot_points[i] is not None]
        self.all_cartesian_points = [knot_points[i] for i in range(len(self.all_knot_points)) if self.all_knot_points[i] is not None]

        #pdb.set_trace()

        print "total knot points:", len(self.all_knot_points), "feasible knot points:", len(self.joint_knot_points)

        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.storeCurrentJointPosition)
        rospy.Subscriber("/stop", std_msgs.msg.Bool, self.stopCurrentPoint)
        rospy.Subscriber("/hit_object", std_msgs.msg.Bool, self.switchToFindObject)

    def storeCurrentJointPosition(self, msg):
        self.currentJointPosition = msg.position

        if abs(msg.effort[0]) > 9000 and abs(msg.effort[2]) > 9000:
            self.should_switch_modes = True
            print "hit something"
            # should also stop current plan

    def stopCurrentPoint(self, msg):
        if msg.data:
            rospy.loginfo("stopping robot")
            lcm_msg = robot_plan_t()
            lcm_msg.utime = 0
            lc = lcm.LCM()
            lc.publish("STOP", lcm_msg.encode())

            self.should_stop = True

    def switchToFindObject(self, msg):
        # gripper contact: effort[2] becomes 10000, 0,1 also become large (order 9000 instead of 2/90)
        if msg.data:
            rospy.loginfo("stopping robot")
            lcm_msg = robot_plan_t()
            lcm_msg.utime = 0
            lc = lcm.LCM()
            lc.publish("STOP", lcm_msg.encode())

            self.should_switch_modes = True

    def calcGridInfo(self, drawer_width, drawer_height, end_effector_width, end_effector_height):
        num_cols = int(drawer_width / end_effector_width)
        num_rows = int(drawer_height / end_effector_height)

        cell_width = float(drawer_width) / num_cols
        cell_height = float(drawer_height) / num_rows

        return num_rows, num_cols, cell_width, cell_height

    def calcGridGoalPoints(self, num_waypoints = 0):
        goal_points = []
        directions = []

        if 0 < num_waypoints < (self.num_rows - 1):
            inc = int((self.num_rows - 2) / num_waypoints)
        elif num_waypoints != 0:
            inc = 1
        else:
            inc = 0

        if self.num_cols % 2 == 0:
            last_point = self.num_cols - 1
        else:
            last_point = self.num_rows*self.num_cols - 1

        point = 0
        direction = 1
        goal_points.append(point)
        directions.append(direction)

        while point != last_point:
            endpoint = point + direction * (self.num_rows - 1) * self.num_cols
            if inc > 0:
                while True:
                    point += direction*inc*self.num_cols
                    if direction > 0 and point >= endpoint:
                        break
                    if direction < 0 and point <= endpoint:
                        break
                    goal_points.append(point)
                    directions.append(direction)

            point = endpoint
            goal_points.append(point)
            directions.append(direction)

            if point == last_point:
                break
            
            point += 1
            goal_points.append(point)
            directions.append(direction)

            direction = -direction
        
        return goal_points, directions

    def calcCartesianKnotPoints(self, grid_goal_points):
        knot_points = []

        for point in grid_goal_points:
            row_idx, col_idx = self.getRowColIdx(point, self.num_cols)

            x = self.start_x + self.cell_width*col_idx + self.cell_width/2.0
            y = self.start_y + self.cell_height*row_idx + self.cell_height/2.0
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

    def calcNextFindObjectPoints(self, current_cartesian_point, direction):
        next_cartesian_points = []
        print direction

        i = 2
        ik_failed = True
        while ik_failed:
            # 2 cells away from the contact point
            next_cartesian_points = []

            next_point = list(current_cartesian_point)
            next_point[1] = current_cartesian_point[1] - direction*2*i*self.cell_height
            next_cartesian_points.append(next_point)

            # 2 cells down from the contact points
            next_point[0] = next_point[0] + i*self.cell_width
            next_cartesian_points.append(next_point)

            # 4 cells towards the object at the new x-value
            next_point[1] = next_point[1] + direction*2*2*i*self.cell_height
            next_cartesian_points.append(next_point)

            joint_positions = [self.getJointPositions(point, direction) for point in next_cartesian_points]

            for point in joint_positions:
                if point is not None:
                    ik_failed = False
                else:
                    ik_failed = True
                    i += 1
                    break

        return joint_positions

    '''
    cartesian_point is a list of 3 floats for x, y, and z (ex: [0.1, 0.2, 0.3])
    This format can be changed to reflect how the points are estimated from the point cloud.
    '''
    def getJointPositions(self, cartesian_point, direction):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = cartesian_point[0]
        pose.position.y = cartesian_point[1]
        pose.position.z = cartesian_point[2]

        # The gripper should be pointing straight down.
        if direction >= 0:
            quat = quaternion_from_euler(pi/7, pi/2, 0)
        else:
            quat = quaternion_from_euler(pi + pi/7, pi/2, 0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = pose
        poseStamped.header.frame_id = "base"

        response = self.robotService.runIK(poseStamped)
        if not response.success:
            #rospy.loginfo("ik was not successful, returning without moving robot")
            return

        # rospy.loginfo("ik was successful, returning joint position")
        return response.joint_state.position

    def exploreDrawer(self):
        new_points = []
        for (i, point) in enumerate(self.joint_knot_points):
            if self.should_switch_modes:
                new_points = self.calcNextFindObjectPoints(self.all_cartesian_points[i], self.directions[i])
                break
            if self.should_stop:
                break
            self.robotService.moveToJointPosition(point, self.maxJointDegreesPerSecond)
        for point in new_points:
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

    explore = ExploreDrawer(start_x, start_y, desired_z, drawer_width, drawer_height, end_effector_width, end_effector_height, num_waypoints = 20)
    explore.exploreDrawer()
    
    rospy.spin()

if __name__ == "__main__":
    main()
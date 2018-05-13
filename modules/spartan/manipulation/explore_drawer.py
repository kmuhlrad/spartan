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
        self.robot_service = spartanROSUtils.RobotService.makeKukaRobotService()
        self.cur_joint_position = [0]*self.robot_service.numJoints

        self.max_joint_deg_per_sec = 15

        self.start_x = start_x
        self.start_y = start_y
        self.desired_z = desired_z

        self.should_stop = False
        self.should_switch_modes = False
        self.in_contact = False

        self.num_rows, self.num_cols, self.cell_width, self.cell_height = self.calcGridInfo(drawer_width, drawer_height, end_effector_width, end_effector_height)

        grid_goal_points, directions = self.calcGridGoalPoints(num_waypoints)
        all_cartesian_knot_points = self.calcCartesianKnotPoints(grid_goal_points)
        all_joint_positions = [self.getJointPositions(point, directions[i]) for (i, point) in enumerate(all_cartesian_knot_points)]

        self.good_joint_positions = [point for point in all_joint_positions if point is not None]
        self.good_directions = [directions[i] for i in range(len(all_joint_positions)) if all_joint_positions[i] is not None]
        self.good_cartesian_points = [all_cartesian_knot_points[i] for i in range(len(all_joint_positions)) if all_joint_positions[i] is not None]
        
        self.cur_cartesian_point = self.good_cartesian_points[0]
        self.cur_direction = self.good_directions[0]

        print "total knot points:", len(all_joint_positions), "feasible knot points:", len(self.good_joint_positions)

        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.storeCurrentJointPosition)
        rospy.Subscriber("/stop", std_msgs.msg.Bool, self.stopCurrentPoint)
        rospy.Subscriber("/hit_object", std_msgs.msg.Bool, self.switchToFindObjectMode)

    def sendStopRobotCommand(self):
        rospy.loginfo("stopping robot")
        lcm_msg = robot_plan_t()
        lcm_msg.utime = 0
        lc = lcm.LCM()
        lc.publish("STOP", lcm_msg.encode())

    def storeCurrentJointPosition(self, msg):
        self.cur_joint_position = msg.position

        contact_force_threshold = 7000

        if abs(msg.effort[0]) > contact_force_threshold and abs(msg.effort[2]) > contact_force_threshold and not self.in_contact:
            rospy.loginfo("hit something")
            self.sendStopRobotCommand()

            self.should_switch_modes = True
            self.in_contact = True

    def stopCurrentPoint(self, msg):
        if msg.data:
            self.sendStopRobotCommand()
            self.should_stop = True

    def switchToFindObjectMode(self, msg):
        if msg.data:
            self.sendStopRobotCommand()
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

        num_grid_cells = 1
        cells_away = 2
        cells_towards = 3

        next_point = list(current_cartesian_point)

        # First go away from the object
        for j in range(cells_away):
            next_point[1] = next_point[1] - direction*num_grid_cells*self.cell_height
            next_cartesian_points.append(tuple(next_point))
            self.cur_cartesian_point = tuple(next_point)

        # Then go down
        next_point[0] = next_point[0] + num_grid_cells*self.cell_width
        next_cartesian_points.append(tuple(next_point))
        self.cur_cartesian_point = tuple(next_point)

        # Finally, go back towards the object
        for j in range(cells_towards):
            next_point[1] = next_point[1] + direction*num_grid_cells*self.cell_height
            next_cartesian_points.append(tuple(next_point))
            self.cur_cartesian_point = tuple(next_point)

        return next_cartesian_points

    '''
    cartesian_point is a list/tuple of 3 floats for x, y, and z (ex: [0.1, 0.2, 0.3])
    direction is an integer, +1 or -1
    '''
    def getJointPositions(self, cartesian_point, direction):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = cartesian_point[0]
        pose.position.y = cartesian_point[1]
        pose.position.z = cartesian_point[2]

        # The gripper should be pointing straight down with the GelSight
        # towards the center of the table.
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

        response = self.robot_service.runIK(poseStamped)
        if not response.success:
            return

        return response.joint_state.position

    def exploreDrawer(self):
        for i, joint_position in enumerate(self.good_joint_positions):
            if not self.should_stop and not self.should_switch_modes:
                self.cur_cartesian_point = self.good_cartesian_points[i]
                self.cur_direction = self.good_directions[i]
                self.robot_service.moveToJointPosition(joint_position, self.max_joint_deg_per_sec)
            else:
                break
        
        while not rospy.is_shutdown():
            if self.should_stop:
                continue

            if self.should_switch_modes:
                search_points = self.calcNextFindObjectPoints(self.cur_cartesian_point, self.cur_direction)
                self.should_switch_modes = False

                for point in search_points:
                    if self.should_stop or self.should_switch_modes:
                        break
                        
                    self.cur_cartesian_point = point
                    joint_position = self.getJointPositions(point, self.cur_direction)

                    

                    self.robot_service.moveToJointPosition(joint_position, self.max_joint_deg_per_sec)
                    self.in_contact = False
            
        print "done"

'''
How to run this demo:

Once the spartan docker is running, open up a new terminal window (remember to run use_ros and use_spartan
in each new window you open). In the main spartan directory, run

python modules/spartan/manipulation/explore_drawer.py

This will start the robot through the demo of altenating between going to the given home
point and every point in the touch points list. If at any time you want to stop the robot from
going to the current point and want it to move on to the next point, open up a new terminal window
and run

rostopic pub /stop std_msgs/Bool true

If you want to send this command programmatically, make a publisher that publishes a boolean message
to the /stop topic with data=true.

'''
def main():
    rospy.init_node('explore_object_node')

    start_x = 0.5 #0.355
    start_y = -0.1 #-0.29
    desired_z = 0.925

    drawer_width = 0.3 #0.49177
    drawer_height = 0.3 #0.60329

    end_effector_width = 0.012285
    end_effector_height = 0.019265

    explore = ExploreDrawer(start_x, start_y, desired_z, drawer_width, drawer_height, end_effector_width, end_effector_height, num_waypoints = 20)
    explore.exploreDrawer()
    
    rospy.spin()

if __name__ == "__main__":
    main()
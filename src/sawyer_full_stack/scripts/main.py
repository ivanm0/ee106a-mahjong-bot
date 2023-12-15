#!/usr/bin/env python
"""
Starter script for 106a lab7. 
Author: Chris Correa
"""
import sys
import argparse
import numpy as np
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
from utils.utils import *

from tf.transformations import euler_from_quaternion
from trac_ik_python.trac_ik import IK

from intera_interface import gripper as robot_gripper
import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from geometry_msgs.msg import Point
from sawyer_pykdl import sawyer_kinematics
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_commander import MoveGroupCommander


def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('sawyer_full_stack')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def get_trajectory(limb, kin, ik_solver, tag_pos, desired_orientation, total_time, args):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    num_way = args.num_way
    task = args.task

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)
    # current_orientation_quat = np.array([getattr(trans.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')])    

    if task == 'line':
        target_pos = [tag_pos.x, tag_pos.y, tag_pos.z]
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, desired_orientation=desired_orientation, total_time=total_time)
    else:
        raise ValueError('task {} not recognized'.format(task))
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(num_way, True)

def get_controller(controller_name, limb, kin):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'pid':
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller


def planning_callback(tile_pos):
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t line -ar_marker 3 -c torque --log
 
    You can also change the rate, timeout if you want
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle.  Defaulget_trajectoryt: line'
    )
    parser.add_argument('-ar_marker', '-ar', nargs='+', help=
        'Which AR marker to use.  Default: 1'
    )
    parser.add_argument('-controller_name', '-c', type=str, default='pid', 
        help='Options: moveit, open_loop, pid.  Default: pid'
    )
    parser.add_argument('-rate', type=int, default=200, help="""
        This specifies how many ms between loops.  It is important to use a rate
        and not a regular while loop because you want the loop to refresh at a
        constant rate, otherwise you would have to tune your PD parameters if 
        the loop runs slower / faster.  Default: 200"""
    )
    parser.add_argument('-timeout', type=int, default=None, help=
        """after how many seconds should the controller terminate if it hasn\'t already.  
        Default: None"""
    )
    parser.add_argument('-num_way', type=int, default=50, help=
        'How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300'
    )
    parser.add_argument('--log', action='store_true', help='plots controller performance')
    args = parser.parse_args()
    
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open()

    
    '''
    Setup the robot
    '''
    ik_solver = IK("base", "right_gripper_tip")
    # ik_solver = IK("base", "stp_022312TP99620_tip_1")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    planner = PathPlanner('right_arm')
    
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()

  
    input("move the robot?")

    above_table_z = tile_pos.z + 0.34
    above_tile_pos = Point(tile_pos.x, tile_pos.y, above_table_z)
    tile_pos = Point(tile_pos.x, tile_pos.y, tile_pos.z + 0.195)
    # forward_pos = Point(0.850, 0.319, 0.531)
    forward_pos = Point(0.70, 0.162, 0.460)


    robot_trajectory = get_trajectory(limb, kin, ik_solver, above_tile_pos, [0, 1, 0, 0], 9, args)
    position = robot_trajectory.joint_trajectory.points[0].positions
    # rotate down
    rotate_robo(planner, position, args)
    # move to above tile
    move_robo(limb, kin, ik_solver, disp_traj, pub, planner, robot_trajectory, args)

    robot_trajectory = get_trajectory(limb, kin, ik_solver, tile_pos, [0, 1, 0, 0], 5, args)
    # move to tile
    move_robo(limb, kin, ik_solver, disp_traj, pub, planner, robot_trajectory, args)

    user_input = input("Press y to close gripper: ")
    if user_input == "y":
        right_gripper = robot_gripper.Gripper('right_gripper')
        print("Closing gripper")
        right_gripper.close()
        rospy.sleep(2.0)

    robot_trajectory = get_trajectory(limb, kin, ik_solver, forward_pos, [0, 1, 0, 0], 9, args)
    # move to forward
    move_robo(limb, kin, ik_solver, disp_traj, pub, planner, robot_trajectory, args)    

    input("Place tile?")

    # show tile to user and move to above table
    last_position = forward_pos
    last_position.z = -0.1
    robot_trajectory = get_trajectory(limb, kin, ik_solver, last_position, [-.017, 0.707, -0.020, 0.707], 9, args)

    # rotate to face user
    position = robot_trajectory.joint_trajectory.points[0].positions
    rotate_robo(planner, position, args)

    # move to above table
    move_robo(limb, kin, ik_solver, disp_traj, pub, planner, robot_trajectory, args)

    user_input = input("Press y to open gripper: ")
    if user_input == "y":
        right_gripper = robot_gripper.Gripper('right_gripper')
        print("Opening gripper")
        right_gripper.open()
        rospy.sleep(2.0)


def rotate_robo(planner, position, args):
    # Move to the trajectory start position
    '''
    plan is a tuple:
        (success_flag : boolean, trajectory: RobotTrajectory, planning_time: float, error_code: MoveitErrorCodes)
    
    plan contains a trajectory from the current state to the robot_trajectory first position which is the pose:
        [current_position, desired_orientation]
    plan[1] is the RobotTrajectory
    '''
    plan = planner.plan_to_joint_pos(position)
    if args.controller_name != "moveit":
        plan = planner.retime_trajectory(plan, 0.3)
    
    # rotates gripper downward
    planner.execute_plan(plan[1])

def move_robo(limb, kin, ik_solver, disp_traj, pub, planner, robot_trajectory, args):
    # robot_trajectory = get_trajectory(limb, kin, ik_solver, desired_position, desired_orientation, args)

    disp_traj.trajectory.append(robot_trajectory)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)

    

    controller = get_controller(args.controller_name, limb, kin)
    try:
        input('Press <Enter> to execute the trajectory using YOUR OWN controller')
    except KeyboardInterrupt:
        sys.exit()
    # execute the path using your own controller.
    done = controller.execute_path(
        robot_trajectory, 
        rate=args.rate, 
        timeout=args.timeout, 
        log=args.log
    )
    if not done:
        print('Failed to move to position')
        sys.exit(0)
            
if __name__ == "__main__":
    rospy.init_node('controller', anonymous=True)
    
    rospy.Subscriber("goal_point", Point, planning_callback) ## TODO: what are we subscribing to here?
    
    rospy.spin()

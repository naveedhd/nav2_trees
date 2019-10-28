#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
##############################################################################
# Imports
##############################################################################

from geometry_msgs.msg import PoseStamped
import py_trees
import py_trees.console as console
import py_trees_ros
from nav2_msgs.action import ComputePathToPose, FollowPath
import rclpy

from typing import Any, Callable
import sys

from . import actions


##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")


##############################################################################
# Tree
##############################################################################

def create_tree():
    root = py_trees.composites.Sequence(
        name='root',
    )

    goal2bb = py_trees_ros.subscribers.ToBlackboard(
        name='Goal2BB',
        topic_name='/goal',
        topic_type=PoseStamped,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={'goal': None}
    )
    
    compute_path_to_pose_client = actions.ActionClientToBlackBoard(
        name='Compute Path To Pose',
        action_type=ComputePathToPose, 
        action_name='ComputePathToPose',
        action_goal_key='pose',
        blackboard_goal_key='goal',
        blackboard_result_key='path',
        generate_feedback_message=lambda msg: "received feedback"
    )    

    follow_path_client = actions.ActionClientFromBlackBoard(
        name='Follow Path',
        action_type=FollowPath, 
        action_name='FollowPath',
        action_goal_key='path',
        blackboard_goal_key='path',
        generate_feedback_message=lambda msg: "received feedback"
    )

    root.add_child(goal2bb)
    root.add_child(compute_path_to_pose_client)
    root.add_child(follow_path_client)

    return root
    

##############################################################################
# Main
##############################################################################

def main(args=None):
    rclpy.init(args=None)    

    tree = py_trees_ros.trees.BehaviourTree(
        root=create_tree(),
        unicode_tree_debug=True
    )

    try:
        tree.setup(timeout=15)
    except Exception as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass

    tree.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
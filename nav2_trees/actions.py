#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviours extended from py_trees_ros.actions.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros
import rclpy


from typing import Any, Callable


##############################################################################
# Behaviours
##############################################################################

class ActionClientFromBlackBoard(py_trees_ros.actions.ActionClient):
    """
    Picks up action goal from blackboard at initialise time
    """
    def __init__(self,
                 action_type: Any,
                 action_name: str,
                 action_goal_key: str,
                 blackboard_goal_key: str, 
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 generate_feedback_message: Callable[[Any], str]=None,
                 ):
        super().__init__(action_type, action_name, None, name, generate_feedback_message)
        self.action_goal_key = action_goal_key
        self.blackboard_goal_key = blackboard_goal_key
        self.blackboard.register_key(key=self.blackboard_goal_key, read=True)
        

    def initialise(self):
       self.action_goal = self.action_type.Goal()
       setattr(self.action_goal, self.action_goal_key, self.blackboard.get(self.blackboard_goal_key))
       super().initialise()


class ActionClientToBlackBoard(ActionClientFromBlackBoard):
    """
    Picks up action goal from blackboard at initialise time
    and writes action result to blackboard
    """
    def __init__(self,
                 action_type: Any,
                 action_name: str,
                 action_goal_key: Any,
                 blackboard_goal_key: str,
                 blackboard_result_key: str,
                 name: str=py_trees.common.Name.AUTO_GENERATED,
                 generate_feedback_message: Callable[[Any], str]=None,
                 ):
        super().__init__(action_type, action_name, action_goal_key, blackboard_goal_key, name, generate_feedback_message)
        self.blackboard_result_key = blackboard_result_key
        self.blackboard.register_key(key=self.blackboard_result_key, write=True)

    def get_result_callback(self, future: rclpy.task.Future):
        super().get_result_callback(future)
        self.blackboard.set(self.blackboard_result_key, self.result_message.result.path, overwrite=True)

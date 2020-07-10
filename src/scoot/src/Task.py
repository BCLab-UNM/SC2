#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy
import StringIO
import traceback
import threading
from std_msgs.msg import UInt8, String

from Scoot import *
from behaviors import *

task_lock = threading.Lock()

'''Node that coordinates the overall robot task'''


class Task:
    global scoot
    STATE_SCOUT_SEARCH = 0
    STATE_SCOUT_FINE_SEARCH = 1

    STATE_EXCAVATOR_GOTO_VOLATILE = 0
    STATE_EXCAVATOR_DIG = 1
    STATE_EXCAVATOR_DROPOFF = 2

    STATE_HAULER_GOTO_EXCAVATOR = 0
    STATE_HAULER_DUMP = 1
    STATE_HAULER_GOTO_PROCESSING_PLANT = 2

    # Function handles for all the behaviors so if the search param was set to searchRandomWalk
    # then PROG_SCOUT_SEARCH would be scout.searchRandomWalk.main this allows running alternative behaviors at launch
    PROG_SCOUT_SEARCH = getattr(scout, rospy.get_param('search', default='search')).main
    PROG_SCOUT_FINE_SEARCH = getattr(scout, rospy.get_param('fine_search', default='fine_search')).main

    PROG_EXCAVATOR_DIG = getattr(excavator, rospy.get_param('dig', default='dig')).main
    PROG_EXCAVATOR_DROPOFF = getattr(excavator, rospy.get_param('dropoff', default='dropoff')).main
    PROG_EXCAVATOR_GOTO_VOLATILE = getattr(excavator, rospy.get_param('goto_volatile', default='goto_volatile')).main

    PROG_HAULER_DUMP = getattr(hauler, rospy.get_param('dump', default='dump')).main
    PROG_HAULER_GOTO_EXCAVATOR = getattr(hauler, rospy.get_param('goto_excavator', default='goto_excavator')).main
    PROG_HAULER_GOTO_PROCESSING_PLANT = getattr(hauler, rospy.get_param('goto_processing_plant', default='goto_processing_plant')).main

    def __init__(self):
        self.state_publisher = rospy.Publisher('/infoLog', String, queue_size=2, latch=False)
        # Published regularly on a timer.
        self.status_pub = rospy.Publisher('status', String, queue_size=1, latch=True)
        # Published once when the status changes.
        self.task_pub = rospy.Publisher('task_state', String, queue_size=1, latch=True)
        self.status_timer = rospy.Timer(rospy.Duration(1), self.publish_status)

        if rospy.has_param('~task_state'):
            self.current_state = rospy.get_param('~task_state')
            rospy.logerr('Task manager restarted.')

        else:
            self.current_state = Task.STATE_SCOUT_SEARCH

        self.prev_state = None

        rospy.on_shutdown(self.save_state)

    def save_state(self):
        rospy.set_param('~task_state', self.current_state)

    def launch(self, prog):
        try:
            return_val = prog(scoot)
            if return_val is None:
                return_val = 0
        except SystemExit as e:
            return_val = e.code
        return return_val

    def run_next(self):
        # @TODO create flow chart of behaviors and ensure match with state mech
        try:
            if scoot.rover_type == 'scout':  # If all good then would go: search->fine_search->search
                if self.current_state == Task.STATE_SCOUT_SEARCH:
                    if self.launch(scoot.behaviors.search.main) == 0:
                        rospy.loginfo('Search succeeded. Do Fine Search')
                        self.current_state = Task.STATE_SCOUT_FINE_SEARCH
                    else:
                        rospy.loginfo('Search failed! Trying again')
                        self.current_state = Task.STATE_SCOUT_SEARCH
                elif self.current_state == Task.STATE_SCOUT_FINE_SEARCH:
                    if self.launch(scoot.behaviors.fine_search.main) == 0:
                        rospy.loginfo('Fine Search succeeded. Starting search.')
                        self.current_state = Task.STATE_SCOUT_SEARCH
                    else:
                        rospy.logwarn('Fine Search failed!')
                        self.current_state = Task.STATE_SCOUT_SEARCH
            elif scoot.rover_type == 'hauler':
                rospy.logwarn_once('Hauler behaviors are currently not implemented')
                '''
                PROG_HAULER_DUMP
                PROG_HAULER_GOTO_EXCAVATOR
                PROG_HAULER_GOTO_PROCESSING_PLANT
                
                STATE_HAULER_DUMP
                STATE_HAULER_GOTO_EXCAVATOR
                STATE_HAULER_GOTO_PROCESSING_PLANT
                '''
                pass
            elif scoot.rover_type == 'excavator':
                rospy.logwarn_once('Excavator behaviors are currently not implemented')
                '''
                PROG_EXCAVATOR_DIG
                PROG_EXCAVATOR_DROPOFF
                PROG_EXCAVATOR_GOTO_VOLATILE
                
                STATE_EXCAVATOR_DIG
                STATE_EXCAVATOR_DROPOFF
                STATE_EXCAVATOR_GOTO_VOLATILE
                '''
                pass
            else:
                rospy.logerr_throttle(20, "UNKNOWN rover type: " + scoot.rover_type)

        except AbortException as e:
            rospy.loginfo('STOP! Entering manual mode.')
            sys.exit(0)

        except Exception as e:
            # FIXME: What do we do with bugs in task code?
            rospy.logerr('Task caught unknown exception:\n' + traceback.format_exc())
            sys.exit(-2)

def main():
    global scoot
    scoot.start(node_name='task')
    taskman = Task()
    while not rospy.is_shutdown():
        taskman.run_next()


if __name__ == '__main__':
    main()

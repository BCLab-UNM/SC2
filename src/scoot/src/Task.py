#! /usr/bin/env python3

from __future__ import print_function

import sys
import math
import rospy
from io import StringIO
import traceback
import threading
from std_msgs.msg import UInt8, String

from Scoot import *
from behaviors import *

task_lock = threading.Lock()

'''Node that coordinates the overall robot task'''


class Task:
    STATE_SCOUT_SEARCH = 0
    STATE_SCOUT_FINE_SEARCH = 1

    STATE_SCOUT_GOTO_PROCESSING_PLANT = 1
    STATE_SCOUT_HOME_ALIGNMENT = 2

    STATE_EXCAVATOR_GOTO_VOLATILE = 0
    STATE_EXCAVATOR_DIG = 1
    STATE_EXCAVATOR_DROPOFF = 2
    STATE_EXCAVATOR_SEARCH = 3

    STATE_HAULER_GOTO_EXCAVATOR = 0
    STATE_HAULER_WAITING = 1
    STATE_HAULER_DUMP = 2
    STATE_HAULER_GOTO_PROCESSING_PLANT = 3

    # Function handles for all the behaviors so if the search param was set to searchRandomWalk
    # then PROG_SCOUT_SEARCH would be scout.searchRandomWalk.main this allows running alternative behaviors at launch
    PROG_SCOUT_SEARCH = getattr(scout, rospy.get_param('search', default='search')).main
    PROG_SCOUT_FINE_SEARCH = getattr(scout, rospy.get_param('fine_search', default='fine_search')).main
    PROG_SCOUT_HOME_ALIGNMENT = getattr(scout, rospy.get_param('home_alignment', default='home_alignment')).main
    PROG_SCOUT_GOTO_PROCESSING_PLANT = getattr(scout, rospy.get_param('goto_processing_plant', default='goto_processing_plant')).main
    
    PROG_EXCAVATOR_SEARCH = getattr(excavator, rospy.get_param('waypoint_search', default='waypoint_search')).main

    PROG_EXCAVATOR_DIG = getattr(excavator, rospy.get_param('dig', default='dig')).main
    PROG_EXCAVATOR_DROPOFF = getattr(excavator, rospy.get_param('dropoff', default='dropoff')).main
    PROG_EXCAVATOR_GOTO_VOLATILE = getattr(excavator, rospy.get_param('goto_volatile', default='goto_volatile')).main

    PROG_HAULER_DUMP = getattr(hauler, rospy.get_param('dump', default='dump')).main
    PROG_HAULER_GOTO_EXCAVATOR = getattr(hauler, rospy.get_param('goto_excavator', default='goto_excavator')).main
    PROG_GOTO_PROCESSING_PLANT = getattr(hauler,
                                         rospy.get_param('goto_processing_plant', default='goto_processing_plant')).main

    ROUND_NUMBER = rospy.get_param('round', default=1)

    def __init__(self):
        self.scoot = None
        self.state_publisher = rospy.Publisher('/infoLog', String, queue_size=2, latch=False)
        # Published regularly on a timer.
        self.status_pub = rospy.Publisher('status', String, queue_size=1, latch=True)
        # Published once when the status changes.
        self.task_pub = rospy.Publisher('task_state', String, queue_size=1, latch=True)

        if rospy.has_param('~task_state'):
            self.current_state = rospy.get_param('~task_state')
            rospy.logerr('Task manager restarted.')

        else:
            self.current_state = Task.STATE_SCOUT_SEARCH  # 0 should be the initial state for all rovers

        self.prev_state = None

        rospy.on_shutdown(self.save_state)

    def save_state(self):
        rospy.set_param('~task_state', self.current_state)

    def launch(self, prog):
        try:
            return_val = prog()
            if return_val is None:
                return_val = 0
        except SystemExit as e:
            return_val = e.code
        except Exception as e:
            rospy.logerr('Task caught ' + self.scoot.rover_type + ',' + str(
                self.current_state) + 'behavior exception:\n' + traceback.format_exc())
            return_val = -1
        return return_val

    def run_next(self):
        # @TODO create flow chart of behaviors and ensure match with state mech
        try:
            if self.scoot.rover_type == 'scout':

                if self.ROUND_NUMBER == 1:
                    # If all good then would go: search->fine_search->search
                    if self.current_state == Task.STATE_SCOUT_SEARCH:
                        if self.launch(self.PROG_SCOUT_SEARCH) == 0:
                            rospy.loginfo('Search succeeded. Do Fine Search')
                            self.current_state = Task.STATE_SCOUT_FINE_SEARCH
                        else:
                            rospy.loginfo('Search failed! Trying again')
                            self.current_state = Task.STATE_SCOUT_SEARCH
                    elif self.current_state == Task.STATE_SCOUT_FINE_SEARCH:
                        if self.launch(self.PROG_SCOUT_FINE_SEARCH) == 0:
                            rospy.loginfo('Fine Search succeeded. Starting search.')
                            self.current_state = Task.STATE_SCOUT_SEARCH
                        else:
                            rospy.logwarn('Fine Search failed!')
                            self.current_state = Task.STATE_SCOUT_SEARCH
                    else:
                        rospy.logerr_throttle(20, "UNKNOWN Scout state: " + str(self.current_state))
                elif self.ROUND_NUMBER == 3:
                    if self.current_state == Task.STATE_SCOUT_SEARCH:
                        if self.launch(self.PROG_SCOUT_SEARCH) == 0:
                            rospy.loginfo('Search succeeded. Do Fine Search')
                        else:
                            rospy.loginfo('Search failed! Trying again')
                    # If all good then would go: search->...
                    # @TODO: state mech for scout's round 3
                    #
                else:
                    rospy.logerr_throttle(20, 'Scout instance should not be running this round')

            elif self.scoot.rover_type == 'hauler':
                rospy.logwarn_once('Hauler behaviors are currently not implemented')
                if self.current_state == Task.STATE_HAULER_GOTO_EXCAVATOR:
                    if self.launch(self.PROG_HAULER_GOTO_EXCAVATOR) == 0:
                        rospy.loginfo('Hauler Arrived at Excavator')
                        self.current_state = Task.STATE_HAULER_WAITING
                    else:
                        rospy.logwarn('Hauler Failed to Arrived at Excavator')
                        self.current_state = Task.STATE_HAULER_GOTO_EXCAVATOR
                elif self.current_state == Task.STATE_HAULER_WAITING:
                    pass  # @TODO: not sure how we should decide to transition out of this state yet
                    # do we want Task to watch for a voltile to appear in a list or
                    # have a behavior wait and return 0 if the list is populated?
                    self.current_state = Task.STATE_HAULER_GOTO_PROCESSING_PLANT
                elif self.current_state == Task.STATE_HAULER_GOTO_PROCESSING_PLANT:
                    if self.launch(self.PROG_GOTO_PROCESSING_PLANT) == 0:
                        rospy.loginfo('Hauler Arrived at Processing Plant')
                        self.current_state = Task.STATE_HAULER_WAITING
                    else:
                        rospy.logwarn('Hauler Failed to Arrived at Processing Plant')
                        self.current_state = Task.STATE_HAULER_GOTO_EXCAVATOR
                elif self.current_state == Task.STATE_HAULER_DUMP:
                    if self.launch(self.PROG_HAULER_DUMP) == 0:
                        rospy.loginfo('Hauler Dump Succeeded')
                        self.current_state = Task.STATE_HAULER_GOTO_EXCAVATOR
                    else:
                        rospy.logwarn('Hauler Dump Failed ')
                        self.current_state = Task.STATE_HAULER_DUMP
                else:
                    rospy.logerr_throttle(20, "UNKNOWN Hauler state: " + str(self.current_state))

            elif self.scoot.rover_type == 'excavator':
                rospy.logwarn_once('Excavator behaviors are currently not implemented')

                if self.current_state == Task.STATE_EXCAVATOR_SEARCH:
                    if self.launch(self.PROG_EXCAVAVTOR) == 0:
                        rospy.loginfo('Search succeeded.')
                if self.current_state == Task.STATE_EXCAVATOR_GOTO_VOLATILE:
                    if self.launch(self.PROG_EXCAVATOR_GOTO_VOLATILE) == 0:
                        rospy.loginfo('Excavator Arrived at Volatile')
                        self.current_state = Task.STATE_EXCAVATOR_DIG
                    else:
                        rospy.logwarn('Excavator Failed to Arrived at Volatile')
                        self.current_state = Task.STATE_EXCAVATOR_GOTO_VOLATILE
                elif self.current_state == Task.STATE_EXCAVATOR_DIG:
                    if self.launch(self.PROG_EXCAVATOR_DIG) == 0:
                        rospy.loginfo('Excavator Dugout Volatile')
                        # @TODO again we have a possible waiting state for the Hauler to arrive
                        self.current_state = Task.PROG_EXCAVATOR_DROPOFF
                    else:
                        rospy.logwarn('Excavator Failed to Dig Out a Volatile')
                        self.current_state = Task.STATE_EXCAVATOR_DIG
                elif self.current_state == Task.STATE_EXCAVATOR_DROPOFF:
                    if self.launch(self.PROG_EXCAVATOR_DROPOFF) == 0:
                        rospy.loginfo('Excavator Dropped Volatile into Hauler')
                        # @TODO again we have a possible waiting state for the Hauler to arrive
                        self.current_state = Task.PROG_EXCAVATOR_DROPOFF
                    else:
                        rospy.logwarn('Excavator Failed to Drop Volatile into Hauler')
                        # only if something like a TF failed or if Hauler is not present
                        self.current_state = Task.PROG_EXCAVATOR_DROPOFF
                        # @TODO: Might accidentally drop the Volatile on the ground,
                        #  then need to either dig again or move to next Volatile
                else:
                    rospy.logerr_throttle(20, "UNKNOWN Excavator state: " + str(self.current_state))
            else:
                rospy.logerr_throttle(20, "UNKNOWN rover type: " + self.scoot.rover_type)

        except AbortException as e:
            rospy.loginfo('STOP! Entering manual mode.')
            sys.exit(0)

        except Exception as e:
            # FIXME: What do we do with bugs in task code?
            rospy.logerr('Task caught unknown exception:\n' + traceback.format_exc())
            sys.exit(-2)


def main():
    rospy.init_node('task')
    scoot = Scoot(rospy.get_param('rover_name', default='small_scout_1'))
    scoot.start(node_name='scoots_task')
    taskman = Task()
    taskman.scoot = scoot
    while not rospy.is_shutdown():
        taskman.run_next()


if __name__ == '__main__':
    main()

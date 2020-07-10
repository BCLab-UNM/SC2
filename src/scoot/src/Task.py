#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy
import StringIO
import traceback
import threading
from std_msgs.msg import UInt8, String
from scoot import sync
from scoot.src.Scoot import scoot, AbortException

# Behavior programs
import scoot.behaviors.scout.search
import scoot.behaviors.scout.fine_search

task_lock = threading.Lock()

'''Node that coordinates the overall robot task'''


class Task:
    STATE_SCOUT_SEARCH = 0
    STATE_SCOUT_FINE_SEARCH = 1

    PROG_SCOUT_SEARCH = 'scout/search.py'
    PROG_SCOUT_SEARCH = 'scout/fine_search.py'

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
            rval = prog(has_block=self.has_block)
            if rval is None:
                rval = 0
        except SystemExit as e:
            rval = e.code
        return rval

    @sync(task_lock)
    def run_next(self):
        try:
            if self.current_state == Task.STATE_SCOUT_SEARCH:
                if self.launch(scoot.behaviors.search.main) == 0:
                    self.print_state('Search succeeded. Do Fine Search')
                    self.current_state = Task.STATE_SCOUT_FINE_SEARCH
                else:
                    self.print_state('Search failed! Trying again')
                    self.current_state = Task.STATE_SCOUT_SEARCH
                    
            elif self.current_state == Task.STATE_SCOUT_FINE_SEARCH:
                if self.launch(scoot.behaviors.fine_search.main) == 0:
                    self.print_state('Fine Search succeeded. Starting search.')
                    self.current_state = Task.STATE_SCOUT_SEARCH
                else:
                    self.print_state('Fine Search failed!')
                    self.current_state = Task.STATE_SCOUT_SEARCH


        except AbortException as e:
            rospy.loginfo('STOP! Entering manual mode.')
            sys.exit(0)

        except Exception as e:
            # FIXME: What do we do with bugs in task code?
            rospy.logerr('Task caught unknown exception:\n' + traceback.format_exc())
            sys.exit(-2)


def main():
    scoot.start(node_name='task')
    taskman = Task()
    while not rospy.is_shutdown():
        taskman.run_next()


if __name__ == '__main__':
    main()

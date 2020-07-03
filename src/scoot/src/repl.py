#!/usr/bin/env python
from Scoot import *
from behaviors import *
from obstacle.msg import Obstacles
from scoot.msg import MoveResult

if __name__ == "__main__":
    rospy.init_node('ScootNode')
    scoot = Scoot("scout_1")
    scoot.start()
    # Systems will have an unmet dependency run "sudo pip install ipython"
    try:
        from IPython import embed
        embed(user_ns=globals())
    except ImportError as e:
        print("Missing IPython run 'sudo pip install ipython'\n Failing over")
        try:
            while True:
                line = raw_input('>>> ')
                if line is not None and line != '':
                    try:
                        exec (line)
                    except Exception as e:
                        print (e)
        except EOFError as e:
            print ("Goodbye")
    print ("Qapla'!")

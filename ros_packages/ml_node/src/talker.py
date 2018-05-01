#!/usr/bin/env python
# license removed for brevity
import rospy, actionlib
from std_msgs.msg import String
from deliver_puck.msg import DeliverPuckAction, DeliverPuckGoal
import subprocess

from world import InterestPoint

class CretinoMind:
    BRAIN_PATH = "/home/figo/catkin_ws/src/ml_node/src/brain.py"

    def __init__(self):
        self.state = 30 * [0]

        self.world = []

        self.brain = subprocess.Popen([CretinoMind.BRAIN_PATH],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE)

        print("Brain is thinking! %s" %(self.brain.stdout.readline()))

        self.encode_initial_state()

    def discpos(self, pos):
        DISCRETE_FACTOR = 20
        return (int((pos[0] * 100)/DISCRETE_FACTOR),
            int((pos[1] * 100)/DISCRETE_FACTOR))

    def encode_initial_state(self):
        dcs = []

        # Holding no pucks
        self.state[0] = 0

        # Robot x, y
        self.state[1] = rospy.get_param("/DockStation/x")
        self.state[2] = rospy.get_param("/DockStation/y")

        # DCs are empty
        self.state[3] = 0
        self.state[4] = 0
        self.state[5] = 0
        self.state[6] = 0
        self.state[7] = 0
        self.state[8] = 0

        # DCs discrete positions
        p = (rospy.get_param("/DC1/x"), rospy.get_param("/DC1/y"))
        p = self.discpos(p)
        self.state[9], self.state[10] = p[0], p[1]
        dcs.append(InterestPoint(InterestPoint.Type.DC, p))

        p = (rospy.get_param("/DC2/x"), rospy.get_param("/DC2/y"))
        p = self.discpos(p)
        self.state[11], self.state[12] = p[0], p[1]
        dcs.append(InterestPoint(InterestPoint.Type.DC, p))

        p = (rospy.get_param("/DC3/x"), rospy.get_param("/DC3/y"))
        p = self.discpos(p)
        self.state[13], self.state[14] = p[0], p[1]
        dcs.append(InterestPoint(InterestPoint.Type.DC, p))

        p = (rospy.get_param("/DC4/x"), rospy.get_param("/DC4/y"))
        p = self.discpos(p)
        self.state[15], self.state[16] = p[0], p[1]
        dcs.append(InterestPoint(InterestPoint.Type.DC, p))

        p = (rospy.get_param("/DC5/x"), rospy.get_param("/DC5/y"))
        p = self.discpos(p)
        self.state[17], self.state[18] = p[0], p[1]
        dcs.append(InterestPoint(InterestPoint.Type.DC, p))

        p = (rospy.get_param("/DC6/x"), rospy.get_param("/DC6/y"))
        p = self.discpos(p)
        self.state[19], self.state[20] = p[0], p[1]
        dcs.append(InterestPoint(InterestPoint.Type.DC, p))

        # Machine colors
        self.state[21] = 1
        self.state[22] = 2
        self.state[23] = 3

        # Machines discrete positions
        p = (rospy.get_param("/Machine1/x"), rospy.get_param("/Machine1/y"))
        p = self.discpos(p)
        self.state[24], self.state[25] = p[0], p[1]
        self.world.append(InterestPoint(InterestPoint.Type.MACHINE, p,
            InterestPoint.ColorEncode.RED))

        p = (rospy.get_param("/Machine2/x"), rospy.get_param("/Machine2/y"))
        p = self.discpos(p)
        self.state[26], self.state[27] = p[0], p[1]
        self.world.append(InterestPoint(InterestPoint.Type.MACHINE, p,
            InterestPoint.ColorEncode.YELLOW))

        p = (rospy.get_param("/Machine3/x"), rospy.get_param("/Machine3/y"))
        p = self.discpos(p)
        self.state[28], self.state[29] = p[0], p[1]
        self.world.append(InterestPoint(InterestPoint.Type.MACHINE, p,
            InterestPoint.ColorEncode.BLUE))

        self.world += dcs

    def print_state(self):
        print("State:")
        print("- Puck: %s" %(self.state[0]))
        print("- Robot position: %s" %([self.state[1], self.state[2]]))
        print("- DC 1 puck: %s" %(self.state[3]))
        print("- DC 2 puck: %s" %(self.state[4]))
        print("- DC 3 puck: %s" %(self.state[5]))
        print("- DC 4 puck: %s" %(self.state[6]))
        print("- DC 5 puck: %s" %(self.state[7]))
        print("- DC 6 puck: %s" %(self.state[8]))
        print("- DC 1 position: %s" %([self.state[9], self.state[10]]))
        print("- DC 2 position: %s" %([self.state[11], self.state[12]]))
        print("- DC 3 position: %s" %([self.state[13], self.state[14]]))
        print("- DC 4 position: %s" %([self.state[15], self.state[16]]))
        print("- DC 5 position: %s" %([self.state[17], self.state[18]]))
        print("- DC 6 position: %s" %([self.state[19], self.state[20]]))
        print("- Machine 1 color: %s" %(self.state[21]))
        print("- Machine 2 color: %s" %(self.state[22]))
        print("- Machine 3 color: %s" %(self.state[23]))
        print("- Machine 1 position: %s" %([self.state[24], self.state[25]]))
        print("- Machine 2 position: %s" %([self.state[26], self.state[27]]))
        print("- Machine 3 position: %s" %([self.state[28], self.state[29]]))


    def act(self):
        state = ""

        for s in self.state:
            state += str(s) + " "

        self.brain.stdin.write("%s\n" %(state))

        action = int(self.brain.stdout.readline())

        if action <= 9:
            # Go dest
            # p = self.world[action].position
            # self.state[1] = p[0]
            # self.state[2] = p[1]
            pass

        elif action == 10:
            # Grab puck
            pass

        elif action == 11:
            pass
            # deliver puck


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # cretino = CretinoMind()

    # cretino.act()

    client = actionlib.SimpleActionClient('deliver_puck', DeliverPuckAction)
    goal = DeliverPuckGoal()
    goal.action_id = 10
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
from wattius_bms import WattiusBMS


def main():

    rospy.init_node("wattius_bms_node")

    rc_node = WattiusBMS()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
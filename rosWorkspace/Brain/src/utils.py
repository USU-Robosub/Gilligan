import rospy

from Robosub.msg import HighLevelControl


def move(direction, motion_type, value):
    move.msg.Direction = direction
    move.msg.MotionType = motion_type
    move.msg.Value = value
    move.pub.publish(move.msg)
move.pub = rospy.Publisher('/High_Level_Motion', HighLevelControl)
move.msg = HighLevelControl()

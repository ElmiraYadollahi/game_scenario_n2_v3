import rospy
from std_msgs.msg import String, Bool

def callback1(data):
    rospy.loginfo("Callback1 heard %s", data.data)

def callback2(data):
    rospy.loginfo("Callback2 heard %s", data.data) 

def listener():
    rospy.init_node('sample_subscriber')
    rospy.Subscriber("pushedR", Bool, callback1)
    rospy.Subscriber("pushedG", Bool, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
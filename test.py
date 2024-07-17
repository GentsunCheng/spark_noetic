import rospy
from swiftpro.msg import *


pos = position()
pos.x = 100
pos.y = 0
pos.z = 100
pos.speed = 100
rospy.init_node('test', anonymous=True)
pub = rospy.Publisher("/position_write_topic", position, queue_size=10)
pub.publish(pos)
rospy.sleep(1.5)
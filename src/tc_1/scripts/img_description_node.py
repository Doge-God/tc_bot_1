#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String

def img_description():
    rospy.init_node("img_description")
    rate = rospy.Rate(10)

    def on_img_description(data):
        rospy.loginfo(f"> VISUAL: {str(data.data)}")

    rospy.Subscriber("img_description", String, on_img_description)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        img_description()
    except rospy.ROSInterruptException:
         pass
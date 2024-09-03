#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String

def onboard_vis():
    rospy.init_node("onboard_vis")
    rate = rospy.Rate(10)

    def on_llm_response(data):
        rospy.loginfo(f"> RESPONSE: {str(data.data)}")

    def on_user_speech(data):
        rospy.loginfo(f"> USER: {str(data.data)}")

    rospy.Subscriber("llm_sentence", String, on_llm_response)
    rospy.Subscriber("stt_sentence", String, on_user_speech)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        onboard_vis()
    except rospy.ROSInterruptException:
         pass
#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String

def onboard_vis():
    rospy.init_node("onboard_vis")
    rate = rospy.Rate(10)

    def on_llm_response(data):
        rospy.loginfo(f"> RESPONSE: {str(data.data).strip()} \n")

    def on_user_speech(data):
        rospy.loginfo(f"> USER: {str(data.data).strip()} \n")

    def on_img_description(data):
        rospy.loginfo(f"\n >> NEW VISUAL CONTEXT: {str(data.data).strip()} \n")

    rospy.Subscriber("llm_sentence", String, on_llm_response)
    rospy.Subscriber("stt_sentence", String, on_user_speech)
    rospy.Subscriber("img_description", String, on_img_description)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        onboard_vis()
    except rospy.ROSInterruptException:
         pass
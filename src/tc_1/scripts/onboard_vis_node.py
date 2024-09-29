#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String
import sys

def onboard_vis():
    rospy.init_node("onboard_vis")
    rate = rospy.Rate(10)

    response_tag = rospy.get_param('~response_tag', "RESPONSE")
    user_tag = rospy.get_param('~user_tag', "USER")

    def on_llm_response(data):
        print(f"> {response_tag}: {str(data.data).strip()} \n")

    def on_user_speech(data):
        print(f"> {user_tag}: {str(data.data).strip()} \n")

    def on_img_description(data):
        print(f"\n >> NEW VISUAL CONTEXT: {str(data.data).strip()} \n")

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
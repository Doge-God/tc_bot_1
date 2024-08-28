#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String
from tc_1.srv import SttControl
import subprocess

set_stt_active = rospy.ServiceProxy("stt_control", SttControl)

def on_llm_response(data):
    global set_stt_active
    try:
        set_stt_active(False)
    except rospy.ServiceException:
        rospy.logwarn("TTS: failed to deactivate STT.")
        pass

    rospy.loginfo(str(data.data))

    subprocess.run(["espeak", str(data.data)])

    try:
        set_stt_active(True)
    except rospy.ServiceException:
        rospy.logwarn("TTS: failed to activate STT.")
        pass
    


def tts():
    rospy.init_node("tts")
    rate = rospy.Rate(10)
    rospy.loginfo(".. Initializing TTS Node")
    rospy.Subscriber("llm_sentence", String, on_llm_response)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        tts()
    except rospy.ROSInterruptException:
         pass
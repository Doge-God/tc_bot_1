#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String
from tc_1.srv import ToggleShowVisCtx, ToggleShowVisCtxResponse
import sys

def onboard_vis():
    rospy.init_node("vis")
    rate = rospy.Rate(10)

    is_showing_vis_ctx = True

    response_tag = rospy.get_param('~response_tag', "RESPONSE")
    user_tag = rospy.get_param('~user_tag', "USER")

    def on_llm_response(data):
        print(f"> {response_tag}: {str(data.data).strip()} \n")

    def on_user_speech(data):
        print(f"> {user_tag}: {str(data.data).strip()} \n")

    def on_img_description(data):
        if is_showing_vis_ctx:
            print(f"\n >> NEW VISUAL CONTEXT: {str(data.data).strip()} \n")

        #### HANDLE MANUAL TTS STOP
    def handle_toggle_show_vis_ctx(req):
        is_showing_vis_ctx = req.isShowingVisCtx
        return ToggleShowVisCtxResponse(f"Showing visual context: {is_showing_vis_ctx}")
    rospy.Service("toggle_show_vis_ctx", ToggleShowVisCtx, handle_toggle_show_vis_ctx)

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
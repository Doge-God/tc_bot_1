#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String
from tc_1.srv import ToggleShowVisCtx, ToggleShowVisCtxResponse
import os

class Vis():
    def __init__(self) -> None:
        self.is_showing_vis_ctx = True

    def run(self):
        rospy.init_node("vis")
        rate = rospy.Rate(10)

        is_showing_vis_ctx = True

        response_tag = rospy.get_param('~response_tag', "RESPONSE")
        user_tag = rospy.get_param('~user_tag', "USER")

        os.system("clear")

        def on_llm_response(data):
            print(f"> {response_tag}: {str(data.data).strip()} \n")
            # rospy.loginfo(f"> {response_tag}: {str(data.data).strip()} \n")

        def on_user_speech(data):
            print(f"> {user_tag}: {str(data.data).strip()} \n")
            # rospy.loginfo(f"> {user_tag}: {str(data.data).strip()} \n")

        def on_img_description(data):
            if self.is_showing_vis_ctx:
                print(f"\n >> NEW VISUAL CONTEXT: {str(data.data).strip()} \n")

        #### HANDLE MANUAL TTS STOP
        def handle_toggle_show_vis_ctx(req):
            self.is_showing_vis_ctx = req.isShowingVisCtx
            return ToggleShowVisCtxResponse(f"Showing visual context: {self.is_showing_vis_ctx}")
        rospy.Service("toggle_show_vis_ctx", ToggleShowVisCtx, handle_toggle_show_vis_ctx)

        rospy.Subscriber("/llm_sentence", String, on_llm_response)
        rospy.Subscriber("/stt_sentence", String, on_user_speech)
        rospy.Subscriber("/img_description", String, on_img_description)

        while not rospy.is_shutdown():
            rate.sleep()
    


if __name__ == '__main__':
    try:
        vis = Vis()
        vis.run()
    except rospy.ROSInterruptException:
         pass
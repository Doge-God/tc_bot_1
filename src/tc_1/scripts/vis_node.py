#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String, Bool
from tc_1.srv import ToggleShowVisCtx, ToggleShowVisCtxResponse
import os

class Vis():
    def __init__(self) -> None:
        self.is_showing_vis_ctx = True
        self.is_active = True
        self.current_visual_context = "No visual context."

    def print_header(self):
        print(f'''
              
              ===========================================================================================
               ████████ ███████ ███    ██ ████████  █████   ██████ ██    ██ ██       █████  ██████  
                  ██    ██      ████   ██    ██    ██   ██ ██      ██    ██ ██      ██   ██ ██   ██ 
                  ██    █████   ██ ██  ██    ██    ███████ ██      ██    ██ ██      ███████ ██████  
                  ██    ██      ██  ██ ██    ██    ██   ██ ██      ██    ██ ██      ██   ██ ██   ██ 
                  ██    ███████ ██   ████    ██    ██   ██  ██████  ██████  ███████ ██   ██ ██   ██ 
              ==========================================================================================
        ''')

    def run(self):
        rospy.init_node("vis")
        rate = rospy.Rate(10)

        self.is_showing_vis_ctx = True

        response_tag = rospy.get_param('~response_tag', "RESPONSE")
        user_tag = rospy.get_param('~user_tag', "USER")
        # is_using_header = rospy.get_param('~is_using_header')
        is_limited_to_one_exchange = True if int(rospy.get_param('~is_one_exchange_only', 0)) == 1 else False

        PRINT_VISUAL_CONTEXT_ASAP = False

        os.system("clear")

        def on_llm_response(data):
            if self.is_active:
                print(f"> {response_tag}: {str(data.data).strip()} \n")
        rospy.Subscriber("/llm_sentence", String, on_llm_response)


        def on_user_speech(data):
            if self.is_active:
                if is_limited_to_one_exchange:
                    os.system("clear")
                    if self.is_showing_vis_ctx and (not PRINT_VISUAL_CONTEXT_ASAP):
                        print(f"\n >> NEW VISUAL CONTEXT: {self.current_visual_context} \n")
                    

                print(f"> {user_tag}: {str(data.data).strip()} \n")
        rospy.Subscriber("/stt_sentence", String, on_user_speech)
        

        def on_img_description(data):
            if self.is_showing_vis_ctx and self.is_active:
                self.current_visual_context = str(data.data).strip()

                if PRINT_VISUAL_CONTEXT_ASAP:
                    print(f"\n >> NEW VISUAL CONTEXT: {str(data.data).strip()} \n")

        rospy.Subscriber("/img_description", String, on_img_description)


        def on_toggle_is_showing_visual_context(data):
            self.is_showing_vis_ctx = data.data
            # rospy.loginfo(f"Showing visual context: {self.is_showing_vis_ctx}")
        rospy.Subscriber("/show_vis_ctx",Bool,on_toggle_is_showing_visual_context)

        def on_toggle_is_active(data):
            self.is_active = data.data
        rospy.Subscriber("/visualize", Bool, on_toggle_is_active)

        def on_clear(data):
            os.system("clear")
        rospy.Subscriber("/clear_visualization",Bool, on_clear)



        #### HANDLE MANUAL TTS STOP
        # NOTE: service 
        def handle_toggle_show_vis_ctx(req):
            self.is_showing_vis_ctx = req.isShowingVisCtx
            return ToggleShowVisCtxResponse(f"Showing visual context: {self.is_showing_vis_ctx}")
        rospy.Service("toggle_show_vis_ctx", ToggleShowVisCtx, handle_toggle_show_vis_ctx)
        

        while not rospy.is_shutdown():
            rate.sleep()
    


if __name__ == '__main__':
    try:
        vis = Vis()
        vis.run()
    except rospy.ROSInterruptException:
         pass
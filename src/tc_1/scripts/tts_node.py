#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String 
from tc_1.srv import SttControl
from tc_1.msg import TTSParams
import subprocess
from dynamic_reconfigure.parameter_generator_catkin import *

class TTS():
    def __init__(self) -> None:
        self.volume = 100 # -a 0 - 200
        self.pitch = 50 # -p 0 - 99
        self.speed = 145 # -s 80 - 500 default 150
        self.capital_behaviour = 1 # -k 1,2, ... 20
        self.voice = "en+f4" # -v 
        self.should_manage_stt = False

    

    def run(self):
        # Reinitialize the node with the retrieved namespace
        rospy.init_node('tts')
        rate = rospy.Rate(10)
        rospy.loginfo(".. Initializing TTS Node")
        self.should_manage_stt = rospy.get_param('~should_manage_stt')
        # self.volume = rospy.get_param('~volume', default="100")
        # self.pitch = rospy.get_param('~pitch', default="50")
        # self.speed = rospy.get_param('~speed', default="150")
        # self.capital_behaviour = rospy.get_param('~capital_behaviour', default="1")
        # self.voice = rospy.get_param('~voice', default="en+m1")

        def on_change_params(data):
            self.volume = str(data.volume)
            self.pitch = str(data.pitch)
            self.speed = str(data.speed)
            self.voice = str(data.voice)
            self.capital_behaviour = str(data.capital_behaviour)
        rospy.Subscriber("tts_params", TTSParams, on_change_params)


        # def on_change_volume(data):
        #     self.volume = str(data.data)
        # rospy.Subscriber("volume", String, on_change_volume)

        # def on_change_pitch(data):
        #     self.pitch = str(data.data)
        # rospy.Subscriber("pitch", String, on_change_pitch)

        # def on_change_speed(data):
        #     self.speed = str(data.data)
        # rospy.Subscriber("speed", String, on_change_speed)

        # def on_change_voice(data):
        #     self.voice = str(data.data)
        # rospy.Subscriber("voice", String, on_change_voice)

        # def on_change_capital_behaviour(data):
        #     self.capital_behaviour=str(data.data)
        # rospy.Subscriber("capital_behaviour", String, on_change_capital_behaviour)

        def stt_control(is_on:bool):
            try:
                set_stt_active = rospy.ServiceProxy("/stt_control", SttControl)
                set_stt_active(is_on)
            except rospy.ServiceException:
                rospy.logwarn("TTS: failed to deactivate STT.")
                pass


        def on_llm_response(data):
            
            if self.should_manage_stt:
                stt_control(is_on=False)

            rospy.loginfo(str(data.data))
            

            subprocess.run(["espeak", 
                    "-a", str(self.volume), 
                    "-p", str(self.pitch), 
                    "-s", str(self.speed), 
                    "-k", str(self.capital_behaviour), 
                    "-v", str(self.voice), 
                    str(data.data)])

            if self.should_manage_stt:
                stt_control(is_on=True)

        rospy.Subscriber("/llm_sentence", String, on_llm_response)


        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        tts = TTS()
        tts.run()
    except rospy.ROSInterruptException:
         pass
#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String 
from tc_1.srv import SttControl
from tc_1.srv import StopTts, StopTtsResponse
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

        self.tts_process = None


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

        #### HANDLE PARAM CHANGE
        def on_change_params(data):
            self.volume = str(data.volume)
            self.pitch = str(data.pitch)
            self.speed = str(data.speed)
            self.voice = str(data.voice)
            self.capital_behaviour = str(data.capital_behaviour)
        rospy.Subscriber("tts_params", TTSParams, on_change_params)


        def stt_control(is_on:bool):
            try:
                set_stt_active = rospy.ServiceProxy("/stt_control", SttControl)
                set_stt_active(is_on)
            except rospy.ServiceException:
                rospy.logwarn("TTS: failed to deactivate STT.")
                pass

        #### HANDLE MANUAL TTS STOP
        def handle_stop_tts(_):
            # make sure there is a processed that is running
            if self.tts_process != None and self.tts_process.returncode != None:
                self.tts_process.terminate()
                self.tts_process.wait()
                stt_control(is_on=True)
            return StopTtsResponse("Success")
        rospy.Service("stop_tts", StopTts, handle_stop_tts)


        def on_llm_response(data):

            if self.should_manage_stt:
                stt_control(is_on=False)

            rospy.loginfo(str(data.data))

            # make sure there is no process OR one is already done
            if self.tts_process == None or self.tts_process.returncode != None:
                self.tts_process = subprocess.Popen(["espeak", 
                        "-a", str(self.volume), 
                        "-p", str(self.pitch), 
                        "-s", str(self.speed), 
                        "-k", str(self.capital_behaviour), 
                        "-v", str(self.voice), 
                        str(data.data)])
                self.tts_process.wait()

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
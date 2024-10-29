#! /usr/local/bin/python3.10

import rospy
from std_msgs.msg import String 
from tc_1.srv import SttControl, TtsControl
from tc_1.srv import StopTts, StopTtsResponse, TtsControlResponse
from tc_1.msg import TTSParams
import subprocess
from dynamic_reconfigure.parameter_generator_catkin import *
import os
from piper import PiperVoice
from typing import Literal
from dynamic_reconfigure.server import Server
from tc_1.cfg import TTSConfig

class TTS():
    def __init__(self) -> None:

        ###### Flags
        self.is_active = True

        ###### General Parameters
        self.tts_method = 0  # TTS method (string)

        ###### Espeak Parameters
        self.volume = 100 # -a 0 - 200
        self.pitch = 50 # -p 0 - 99
        self.speed = 145 # -s 80 - 500 default 150
        self.capital_behaviour = 1 # -k 1,2, ... 20
        self.voice = "en+f4" # -v 

        ###### Piper Parameters
        self.piper_voice_key = 0  # Choices of Piper voices
        
        ###### STT Control 
        self.should_manage_stt = False
        self.tts_process = None

        # Piper setup
        model_path = os.path.expanduser("~/piper/models/en_US-amy-low.onnx")
        self.piper_voice = PiperVoice.load(model_path)

        self.tts_methods = ["piper", "espeak"]

        self.piper_voices = [os.path.expanduser("~/piper/models/en_US-amy-medium.onnx"), 
                             os.path.expanduser("~/piper/models/en_US-danny-low.onnx"),
                             os.path.expanduser("~/piper/models/en_GB-cori-high.onnx"),
                             os.path.expanduser("~/piper/models/en_GB-alan-medium.onnx"),
                             os.path.expanduser("~/piper/models/en_GB-alba-medium.onnx"),
                             os.path.expanduser("~/piper/models/en_GB-jenny_dioco-medium.onnx"),
                             os.path.expanduser("~/piper/models/it_IT-paola-medium.onnx"),
                             os.path.expanduser("~/piper/models/de_DE-thorsten-high.onnx"),
                             os.path.expanduser("~/piper/models/en_US-ryan-high.onnx"),
                             ]
         

    def dynamic_reconfig_callback(self, config, level):
        """
        This callback will be called whenever parameters are updated via dynamic reconfigure.
        """
        
        self.volume = config["volume"]
        self.pitch = config["pitch"]
        self.speed = config["speed"]
        self.capital_behavior = config["capital_behaviour"]
        self.voice = config["voice"]
        self.tts_method = config["tts_method"]
        self.piper_voice = PiperVoice.load(self.piper_voices[config["piper_voice"]])
        self.piper_bitrate = config["piper_bitrate"]

        rospy.loginfo("Reconfigured TTS.")

        return config

    def run(self):
        # Reinitialize the node with the retrieved namespace
        rospy.init_node('tts')
        rate = rospy.Rate(10)
        rospy.loginfo(".. Initializing TTS Node")
        self.should_manage_stt = rospy.get_param('~should_manage_stt')
        

        ###### General Parameters
        self.tts_method = rospy.get_param("~tts_method", 0)  # TTS method (string)

        ###### Espeak Parameters
        self.volume = rospy.get_param("~volume", 100.0)  # -a 0 - 200
        self.pitch = rospy.get_param("~pitch", 50.0)  # -p 0 - 99
        self.speed = rospy.get_param("~speed", 145.0)  # -s 80 - 500 default 150
        self.capital_behavior = rospy.get_param("~capital_behaviour", 1)  # -k 1,2, ... 20
        self.voice = rospy.get_param("~voice", "en+f4")  # -v 

        ###### Piper Parameters
        self.piper_voice_key = rospy.get_param("~piper_voice", 0)  # Choices of Piper voices
        self.piper_bitrate = rospy.get_param("~piper_bitrate", 22050)

        ###### STT Control 
        self.tts_process = None

        # Dynamic reconfigure server to handle live updates of parameters
        self.server = Server(TTSConfig, self.dynamic_reconfig_callback)


        #### HANDLE PARAM CHANGE
        # def on_change_params(data):
        #     self.volume = str(data.volume)
        #     self.pitch = str(data.pitch)
        #     self.speed = str(data.speed)
        #     self.voice = str(data.voice)
        #     self.capital_behaviour = str(data.capital_behaviour)
        # rospy.Subscriber("tts_params", TTSParams, on_change_params)


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
            if self.tts_process != None and self.tts_process.returncode == None:
                self.tts_process.terminate()
                self.tts_process.wait()
                stt_control(is_on=True)
            return StopTtsResponse("Success")
        rospy.Service("stop_tts", StopTts, handle_stop_tts)

        #### HANDLE TTS CONTROL:
        def handle_tts_control(req):
            self.is_active = req.isTtsActive
            rospy.loginfo(f"TTS control recieved, is_active = {self.is_active}")
            
            # Stop current speaking process, signal stt control for finishing speaking
            if self.is_active == False:
                if self.tts_process != None and self.tts_process.returncode == None:
                    self.tts_process.terminate()
                    self.tts_process.wait()
            
            
            stt_control(is_on=True)

            return TtsControlResponse(self.is_active)
        rospy.Service('tts_control', TtsControl, handle_tts_control)

        def run_tts_espeak(content:str):
            # make sure there is no process OR one is already done
            
            if self.tts_process == None or self.tts_process.returncode != None:
                if self.should_manage_stt:
                    stt_control(is_on=False)
                    
                rospy.loginfo(f"ESPEAK: {content}")
                self.tts_process = subprocess.Popen(["espeak", 
                        "-a", str(self.volume), 
                        "-p", str(self.pitch), 
                        "-s", str(self.speed), 
                        "-k", str(self.capital_behaviour), 
                        "-v", str(self.voice), 
                        content])

                self.tts_process.wait()
                print(f"espeak process done: {self.tts_process.returncode}")

                if self.should_manage_stt:
                    stt_control(is_on=True)

        def run_tts_piper(content:str):
            
            # make sure there is no process OR one is already done
            if self.tts_process == None or self.tts_process.returncode != None:
                if self.should_manage_stt:
                    stt_control(is_on=False)

                rospy.loginfo(f"PIPER: {content}")
                self.tts_process = subprocess.Popen([
                        "aplay", "-r", str(self.piper_bitrate), "-f", "S16_LE", "-t", "raw" 
                    ], stdin=subprocess.PIPE)
                
                synthesized_buffer = self.piper_voice.synthesize_stream_raw(content)
                
                try: 
                    for data in synthesized_buffer:
                        self.tts_process.stdin.write(data)

                    self.tts_process.stdin.flush() 
                    self.tts_process.stdin.close()
                    self.tts_process.wait()

                    print(f"piper process done: {self.tts_process.returncode}")
                
                except:
                    print("Error sending bytes to tts process.")

                # self.tts_process.wait()

                if self.should_manage_stt:
                    stt_control(is_on=True)


        def on_llm_response(data):
            if not self.is_active:
                rospy.loginfo("TTS recieved message but is not active.")
                stt_control(is_on=True)
                return
            if self.tts_method == 0:
                run_tts_piper(str(data.data))
            elif self.tts_method == 1:
                run_tts_espeak(str(data.data))
            else:
                raise NotImplementedError("Unknown TTS Method.")

            
        rospy.Subscriber("/llm_sentence", String, on_llm_response)


        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        tts = TTS()
        tts.run()
    except rospy.ROSInterruptException:
         pass
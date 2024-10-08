#! /usr/local/bin/python3.10

#

# /usr/bin/python3

import rospy
from std_msgs.msg import String
import os
from dotenv import load_dotenv
import pyaudio

from deepgram import DeepgramClient, LiveTranscriptionEvents, LiveOptions, AsyncLiveClient

from tc_1.srv import SttControl, SttControlResponse, SttManualControl, SttManualControlResponse

import rospy
from std_msgs.msg import String
import time

load_dotenv()

API_KEY = os.getenv("DEEPGRAM_KEY")
#################################

class AudioHandler():
    def __init__(self, format=pyaudio.paInt16, channels=1, rate=16000, chunk=8000) -> None:

        # Internal data structures
        self.audio_queue = []

        # Settings
        self.FORMAT = format
        self.CHANNELS=channels
        self.RATE=rate
        self.CHUNK = chunk
        self.stream = None

    def mic_callback(self,input_data, frame_count, time_info, status_flag):
        self.audio_queue.append(input_data)
        return (input_data, pyaudio.paContinue)
        
        
    def start_audio_stream(self):
        if self.stream != None:
            return
        # create new audio stream if one does not exist yet
        audio = pyaudio.PyAudio()
        self.stream = audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK,
            stream_callback=self.mic_callback,
        )
        self.stream.start_stream()

    def pause_stream(self):
        self.stream.stop_stream()
    
    def unpause_stream(self):
        self.stream.start_stream()

    def stop_audio_stream(self):
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()

    def get_audio_data(self):
        try:
            return self.audio_queue.pop(0)
        except:
            return None
    
    def clear_audio_queue(self):
        self.audio_queue = []

class STTDeepgram():
    def __init__(self) -> None:

        # internal data structures & flags
        self.current_sentence = ""
        self.is_pausing = False # controlled by tts and such
        self.is_exiting = False
        self.is_manual_stopped = False
        self.should_restart_stt = True

        self.stt_sentence_pub = None

        # initialize audio handler placeholder
        self.audio_handler = None
        #init dg connection placeholder
        self.dg_connection = None

        # settings
        self.dg_options = LiveOptions(
            model="nova-2", 
            language="en-US", 
            smart_format=True,
            encoding="linear16",
            sample_rate=16000,
            # VAD event settings
            interim_results=True,
            vad_events=True,
            utterance_end_ms="1000",
            # Time in milliseconds of silence to wait for before finalizing speech
            endpointing=30
        )
    
    def get_on_message(self):
        def on_message(sdk_self, result, **kwargs):
            if result.speech_final:
                sentence = result.channel.alternatives[0].transcript

                if len(sentence) == 0:
                    return
                self.current_sentence += f" {sentence}"
        return on_message

    def get_on_utterance_end(self):
        def on_utterance_end(sdk_self, utterance_end,**kwargs):
            self.stt_sentence_pub.publish(self.current_sentence)
            rospy.loginfo(f"Utterance end: {self.current_sentence}")
            self.current_sentence = ""
        return on_utterance_end
        
    def get_on_metadata(self):
        def on_metadata(sdk_self, metadata, **kwargs):
            print(f"\n\n{metadata}\n\n")
        return on_metadata
    
    def get_on_error(self): 
        def on_error(sdk_self, error, **kwargs):
            rospy.logerr(f"\n\n{error}\n\n")
            # set flags for exiting audio sending loop 
            self.should_restart_stt = True
            self.is_exiting = True
            # stop audio stream
            self.audio_handler.stop_audio_stream()
            self.audio_handler = None  
        return on_error
    

    def run(self):
        #################################################
        ## Node setup ###################################
        #################################################
        rospy.init_node("stt")
        rate = rospy.Rate(1)
        rospy.loginfo(".. Initializing STT Node")
        self.stt_sentence_pub = rospy.Publisher('stt_sentence', String, queue_size=5)

        # Set up stt_control service
        def handle_toggle_recording(req):
            self.is_pausing = not req.isSttActive
            rospy.loginfo(f"stt control recieved, pausing = {self.is_pausing}")
            return SttControlResponse(not self.is_pausing)
        rospy.Service('stt_control', SttControl, handle_toggle_recording)

        # Setup stt manual control service
        def handle_toggle_manual_control(req):
            self.is_manual_stopped = not req.isSttActive
            rospy.loginfo(f"stt MANUAL control recieved, stt stopped = {self.is_manual_stopped}")
            return SttManualControlResponse(not self.is_manual_stopped)
        rospy.Service('stt_manual_control', SttManualControl,handle_toggle_manual_control)

         ##########################################
        ## Node Shutdown Behaviour ###############
        ##########################################
        def on_node_shutdown():
            self.is_exiting = True
        rospy.on_shutdown(on_node_shutdown)
             

        #################################################
        ## STT setup ####################################
        #################################################
        while not rospy.is_shutdown():
            # try to re-establish stt service
            rospy.loginfo("## STARTING NEW DG CLIENT")
            self.is_exiting = False
            

            try:
                # Create a Deepgram client and connection
                deepgram = DeepgramClient(API_KEY)
                dg_connection = deepgram.listen.websocket.v("1")     

                # DG client event callbacks

                # Register callbacks
                dg_connection.on(LiveTranscriptionEvents.Transcript, self.get_on_message())
                dg_connection.on(LiveTranscriptionEvents.Metadata, self.get_on_metadata())
                dg_connection.on(LiveTranscriptionEvents.Error, self.get_on_error())
                dg_connection.on(LiveTranscriptionEvents.UtteranceEnd, self.get_on_utterance_end())
                
                # Start the connection
                dg_connection.start(self.dg_options)
                
                # initialize audio handler
                self.audio_handler = AudioHandler()
                self.audio_handler.start_audio_stream()
                
                ###########################################################
                #### AUDIO SENDING LOOP ###################################
                while True:
                    # get audio data from queue
                    data = self.audio_handler.get_audio_data()
                    if data == None:
                        continue
                    # lock exit flag and check for exit
                    if self.is_exiting:
                        break
                    # If paused or manually stopped, dont send anything and continue

                    if self.is_manual_stopped or self.is_pausing:
                        self.audio_handler.pause_stream()
                        self.audio_handler.clear_audio_queue()
                        while self.is_manual_stopped or self.is_pausing:
                            # self.audio_handler.clear_audio_queue()
                            # self.audio_handler.stream.stop_stream
                            dg_connection.keep_alive()

                        self.audio_handler.unpause_stream()
                        
                        
                        
                    # send data over stream
                    dg_connection.send(data)

                # close connection
                dg_connection.finish()

            except Exception as e:
                rospy.logerr(f"STT Node Error: {str(e)}")
                # stop audio stream
                self.audio_handler.stop_audio_stream()
                self.audio_handler = None
                continue

            rate.sleep()
        
       

# Execute if main
if __name__ == '__main__':
    try:
        stt = STTDeepgram()
        stt.run()
    except rospy.ROSInterruptException:
        pass
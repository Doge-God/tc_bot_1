#! /home/usr-1/.pyenv/versions/3.11.9/bin/python

import rospy
from std_msgs.msg import String
import os
from dotenv import load_dotenv
import threading
import pyaudio

from deepgram import DeepgramClient, LiveTranscriptionEvents, LiveOptions


load_dotenv()

API_KEY = os.getenv("DEEPGRAM_KEY")

################################
## DATA & CONFIG ###############
################################
all_mic_data = []
all_transcripts = []
audio_queue = []
current_sentence = ""

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 8000

# flow control flags & lock for thread sync
lock_exit = threading.Lock()
lock_pause = threading.Lock()
is_exiting = False
is_pausing = False

# Configure Deepgram options for live transcription
dg_options = LiveOptions(
    model="nova-2", 
    language="en-US", 
    smart_format=True,
    encoding="linear16",
    sample_rate=RATE,
    # VAD event settings
    interim_results=True,
    vad_events=True,
    utterance_end_ms="1000",
    # Time in milliseconds of silence to wait for before finalizing speech
    endpointing=30
)
#################################

def mic_callback(input_data, frame_count, time_info, status_flag):
    audio_queue.append(input_data)
    return (input_data, pyaudio.paContinue)



def stt():
    global current_sentence
    global dg_options
    global is_exiting
    global is_pausing

    #################################################
    ## Node setup ###################################
    #################################################
    rospy.init_node("stt")
    rate = rospy.Rate(10)
    rospy.loginfo(".. Initializing STT Node")
    stt_sentence_pub = rospy.Publisher('stt_sentence', String, queue_size=10)

    #################################################
    ## STT setup ####################################
    #################################################
    try:
        # open microphone stream
        audio = pyaudio.PyAudio()
        stream = audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK,
            stream_callback=mic_callback,
        )

        stream.start_stream()

        # Create a Deepgram client and connection
        deepgram = DeepgramClient(API_KEY)
        dg_connection = deepgram.listen.websocket.v("1")


        ## EVENT CALLBACKS #########################################################
        def on_message(self, result, **kwargs):
            global current_sentence
            if result.speech_final:
                sentence = result.channel.alternatives[0].transcript

                if len(sentence) == 0:
                    return
                current_sentence += f" {sentence}"

        def on_utterance_end(self, utterance_end,**kwargs):
            global current_sentence
            stt_sentence_pub.publish(current_sentence)
            rospy.loginfo(f"Utterance end: {current_sentence}")
            current_sentence = ""

        def on_metadata(self, metadata, **kwargs):
            print(f"\n\n{metadata}\n\n")

        def on_error(self, error, **kwargs):
            print(f"\n\n{error}\n\n")

        # Register callbacks
        dg_connection.on(LiveTranscriptionEvents.Transcript, on_message)
        dg_connection.on(LiveTranscriptionEvents.Metadata, on_metadata)
        dg_connection.on(LiveTranscriptionEvents.Error, on_error)
        dg_connection.on(LiveTranscriptionEvents.UtteranceEnd, on_utterance_end)
        ###########################################################################
        
        
        # Start the connection
        dg_connection.start(dg_options)

        # Audio loop to send audio to deepgram
        def audio_send_loop():
            while True:
                # get data from audio queue
                if len(audio_queue) == 0:
                    continue
                data = audio_queue.pop(0)
                # lock exit flag and check for exit
                lock_exit.acquire()
                if is_exiting:
                    break
                lock_exit.release()
                
                # lock pause flag and check for pause
                lock_pause.acquire()
                if is_pausing:
                    continue
                lock_pause.release()

                # send data over stream
                dg_connection.send(data)
        
        ## START AUDIO THREAD ##
        audio_thread = threading.Thread(target=audio_send_loop)
        audio_thread.start()

        ##########################################
        ## Node Shutdown Behaviour ###############
        ##########################################
        def on_node_shutdown():
            global lock_exit
            global is_exiting
            # Set the exit flag to True to stop the thread
            lock_exit.acquire()
            is_exiting = True
            lock_exit.release()

            # Wait for the thread to finish
            audio_thread.join()

            # Close the connection to Deepgram
            dg_connection.finish()

            # close audio stream
            stream.stop_stream()
            stream.close()  

        rospy.on_shutdown(on_node_shutdown)

        while not rospy.is_shutdown():
            rate.sleep()

    except Exception as e:
        rospy.logwarn(f"STT Node Error: {str(e)}")
        return


# Execute if main
if __name__ == '__main__':
    try:
        stt()
    except rospy.ROSInterruptException:
        pass
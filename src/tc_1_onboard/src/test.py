import os
from dotenv import load_dotenv
import threading
import pyaudio

from deepgram import (
    DeepgramClient,
    LiveTranscriptionEvents,
    LiveOptions,
)

load_dotenv()

API_KEY = os.getenv("DEEPGRAM_KEY")

################################
## DATA #######################
##################################
all_mic_data = []
all_transcripts = []
audio_queue = []
current_sentence = ""

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 8000
####################################################

def mic_callback(input_data, frame_count, time_info, status_flag):
    audio_queue.append(input_data)
    return (input_data, pyaudio.paContinue)



def main():
    global current_sentence
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

        # Create a Deepgram client using the API key
        deepgram = DeepgramClient(API_KEY)

        # Create a websocket connection to Deepgram
        dg_connection = deepgram.listen.websocket.v("1")

        # Define the event handlers for the connection
        def on_message(self, result, **kwargs):
            global current_sentence
            if result.speech_final:
                sentence = result.channel.alternatives[0].transcript

                if len(sentence) == 0:
                    return
                current_sentence += f" {sentence}"

        def on_utterance_end(self, utterance_end,**kwargs):
            global current_sentence
            print(current_sentence)
            current_sentence = ""

        def on_metadata(self, metadata, **kwargs):
            print(f"\n\n{metadata}\n\n")

        def on_error(self, error, **kwargs):
            print(f"\n\n{error}\n\n")

        # Register the event handlers
        dg_connection.on(LiveTranscriptionEvents.Transcript, on_message)
        dg_connection.on(LiveTranscriptionEvents.Metadata, on_metadata)
        dg_connection.on(LiveTranscriptionEvents.Error, on_error)
        dg_connection.on(LiveTranscriptionEvents.UtteranceEnd, on_utterance_end)

        # STEP 5: Configure Deepgram options for live transcription
        options = LiveOptions(
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
        
        # STEP 6: Start the connection
        dg_connection.start(options)

        # STEP 7: Create a lock and a flag for thread synchronization
        lock_exit = threading.Lock()
        lock_pause = threading.Lock()
        exit = False
        pause = False

        # STEP 8: Define a thread that streams the audio and sends it to Deepgram
        def myThread():
            while True:
                # get data from audio queue
                if len(audio_queue) == 0:
                    continue
                data = audio_queue.pop(0)
                # lock exit flag and check for exit
                lock_exit.acquire()
                if exit:
                    break
                lock_exit.release()
                
                # lock pause flag and check for pause
                lock_pause.acquire()
                if pause:
                    continue
                lock_pause.release()

                # send data over stream
                dg_connection.send(data)


        # STEP 9: Start the thread
        audio_thread = threading.Thread(target=myThread)
        audio_thread.start()

        # STEP 10: Wait for user input to stop recording
        input("Press Enter to stop recording...\n\n")

        # STEP 11: Set the exit flag to True to stop the thread
        lock_exit.acquire()
        exit = True
        lock_exit.release()

        # STEP 12: Wait for the thread to finish
        audio_thread.join()

        # STEP 13: Close the connection to Deepgram
        dg_connection.finish()

        # close audio stream
        stream.stop_stream()
        stream.close()  

        print("Finished")

    except Exception as e:
        print(f"Could not open socket: {e}")
        return


if __name__ == "__main__":
    main()

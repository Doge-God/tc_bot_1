import os
import httpx
from dotenv import load_dotenv
import threading
import pyaudio
import asyncio

from deepgram import (
    DeepgramClient,
    LiveTranscriptionEvents,
    LiveOptions,
)

load_dotenv()

# URL for the realtime streaming audio you would like to transcribe
URL = "http://stream.live.vc.bbcmedia.co.uk/bbc_world_service"
API_KEY = os.getenv("DEEPGRAM_KEY")

################################
## DATA #######################
##################################
all_mic_data = []
all_transcripts = []
audio_queue = []

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 8000
####################################################

def mic_callback(input_data, frame_count, time_info, status_flag):
    audio_queue.append(input_data)
    return (input_data, pyaudio.paContinue)

#  # Set up microphone if streaming from mic
# async def microphone():
#     audio = pyaudio.PyAudio()
#     stream = audio.open(
#         format=FORMAT,
#         channels=CHANNELS,
#         rate=RATE,
#         input=True,
#         frames_per_buffer=CHUNK,
#         stream_callback=mic_callback,
#     )

#     stream.start_stream()

#     global SAMPLE_SIZE
#     SAMPLE_SIZE = audio.get_sample_size(FORMAT)

#     while stream.is_active():
#         await asyncio.sleep(0.1)

#     stream.stop_stream()
#     stream.close()


def main():
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

        # STEP 1: Create a Deepgram client using the API key
        deepgram = DeepgramClient(API_KEY)

        # STEP 2: Create a websocket connection to Deepgram
        dg_connection = deepgram.listen.websocket.v("1")

        # STEP 3: Define the event handlers for the connection
        def on_message(self, result, **kwargs):

            if result.speech_final:
                sentence = result.channel.alternatives[0].transcript

                if len(sentence) == 0:
                    return
                print(f"speaker: {sentence}")

        def on_metadata(self, metadata, **kwargs):
            print(f"\n\n{metadata}\n\n")

        def on_error(self, error, **kwargs):
            print(f"\n\n{error}\n\n")

        # STEP 4: Register the event handlers
        dg_connection.on(LiveTranscriptionEvents.Transcript, on_message)
        dg_connection.on(LiveTranscriptionEvents.Metadata, on_metadata)
        dg_connection.on(LiveTranscriptionEvents.Error, on_error)

        # STEP 5: Configure Deepgram options for live transcription
        options = LiveOptions(
            model="nova-2", 
            language="en-US", 
            smart_format=True,
            encoding="linear16",
            sample_rate=RATE
            )
        
        # STEP 6: Start the connection
        dg_connection.start(options)

        # STEP 7: Create a lock and a flag for thread synchronization
        lock_exit = threading.Lock()
        exit = False

        # STEP 8: Define a thread that streams the audio and sends it to Deepgram
        def myThread():
            # with httpx.stream("GET", URL) as r:
            #     for data in r.iter_bytes():
            #         lock_exit.acquire()
            #         if exit:
            #             break
            #         lock_exit.release()

            #         dg_connection.send(data)
            while not len(audio_queue) == 0:
                data = audio_queue.pop(0)
                lock_exit.acquire()
                if exit:
                    break
                lock_exit.release()

                dg_connection.send(data)


        # STEP 9: Start the thread
        # audio_thread = threading.Thread(target=myThread)
        # audio_thread.start()

        while True:
            if len(audio_queue) == 0:
                continue
            data = audio_queue.pop(0)
            lock_exit.acquire()
            if exit:
                break
            lock_exit.release()

            dg_connection.send(data)

        # STEP 10: Wait for user input to stop recording
        input("Press Enter to stop recording...\n\n")

        # STEP 11: Set the exit flag to True to stop the thread
        lock_exit.acquire()
        exit = True
        lock_exit.release()

        # STEP 12: Wait for the thread to finish
        # audio_thread.join()

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

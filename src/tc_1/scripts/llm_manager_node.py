#! /usr/local/bin/python3.10

# 

# /usr/bin/python3

import rospy
from std_msgs.msg import String
import os
from dotenv import load_dotenv
from openai import OpenAI
import subprocess
from tc_1.srv import SttControl
from collections import deque

load_dotenv()

OPEN_AI_KEY = os.getenv("OPEN_AI_KEY")
system_prompt = {
    "role": "system",
    "content": "You are a robot. You give very brief responses. Omit any formatting like **."
}
message_log = deque([], maxlen=30)
open_ai_client = OpenAI(api_key=OPEN_AI_KEY)

def create_user_msg(msg:str):
    return {"role":"user","content":msg}

def create_assist_msg(msg:str):
    return {"role":"assistant","content":msg}

def create_system_prompt(msg:str):
    return {"role":"system", "content":msg}


def llm_manager():
    rospy.init_node("llm_manager")
    rate = rospy.Rate(10)
    rospy.loginfo(".. Initializing LLM Manager Node")
    rospy.Subscriber("stt_sentence", String, on_user_speech)
    llm_sentence_pub = rospy.Publisher("llm_sentence",String, queue_size=10)


    def get_gpt_response(messages):
            global message_log, system_prompt
            completion = open_ai_client.chat.completions.create(model="gpt-4o-mini", messages=messages)
            gpt_msg = completion.choices[0].message.content.strip()

            message_log.append(create_assist_msg(gpt_msg))
            
            # if (len(message_log) >= 25):
            #     message_log.pop(0)
            #     message_log.pop(0)
            #     message_log.insert(0,system_prompt)

            return gpt_msg

    def on_user_speech(data):

        set_stt_active = rospy.ServiceProxy("stt_control", SttControl)
        set_stt_active(False)

        # ignore null or extrememly short cut-off words
        if len(str(data.data)) <= 4:
            return

        global message_log, system_prompt
        message_log.append(create_user_msg(str(data.data)))
        response = get_gpt_response([system_prompt].extend(message_log))

        llm_sentence_pub.publish(response)


    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        llm_manager()
    except rospy.ROSInterruptException:
         pass
    

    
    
    
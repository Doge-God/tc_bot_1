#! /usr/local/bin/pip3.12

# /usr/bin/env python



import rospy
from std_msgs.msg import String
import os
from dotenv import load_dotenv
from openai import OpenAI

load_dotenv()

OPEN_AI_KEY = os.getenv("OPEN_AI_KEY")
SYSTEM_PROMPT = {
    "role": "system",
    "content": "You are a helpful robot."
}
message_log = []
open_ai_client = OpenAI(api_key=OPEN_AI_KEY)

def create_user_msg(msg:str):
    return {"role":"user","content":msg}

def create_assist_msg(msg:str):
    return {"role":"assistant","content":msg}

def get_gpt_response(messages):
        global message_log
        completion = open_ai_client.chat.completions.create(model="gpt-4o-mini", messages=messages)
        gpt_msg = completion.choices[0].message.content.strip()

        # if "IGNORE" in gpt_msg:
        #     messages_log.pop()
        #     return None

        message_log.append(create_assist_msg(gpt_msg))
        
        if (len(message_log) >= 25):
            message_log.pop(0)
            message_log.pop(0)
            message_log.insert(0,SYSTEM_PROMPT)

        return gpt_msg

def on_user_speech(data):
        global message_log
        message_log.append(create_user_msg(str(data.data)))
        response = get_gpt_response(message_log)
        rospy.loginfo(response)

def llm_manager():
    rospy.init_node("llm_manager")
    rate = rospy.Rate(10)
    rospy.loginfo(".. Initializing LLM Manager Node")
    rospy.Subscriber("stt_sentence", String, on_user_speech)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        llm_manager()
    except rospy.ROSInterruptException:
         pass
    

    
    
    
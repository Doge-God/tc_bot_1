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
from tc_1.srv import UpdateVisualPrompt, UpdateVisualPromptResponse
from tc_1.srv import InjectUserInput, InjectUserInputResponse
from tc_1.srv import InjectLLMResponse, InjectLLMResponseResponse
from collections import deque

load_dotenv()

OPEN_AI_KEY = os.getenv("OPEN_AI_KEY")
INTERACTION_BASE_SYSTEM_PROMPT = "You are a robot. You talk like Hal9000. You give very brief responses. Omit any formatting in your response."

VISUAL_BASE_SYSTEM_PROMPT ="Describe what you see. Respond in second person as if you are prompting a LLM."
visual_additional_system_prompt = ""
visual_system_promp = {
    "role":"system",
    "content": f"{VISUAL_BASE_SYSTEM_PROMPT} {visual_additional_system_prompt}"
}

# logical_context_strings = []
visual_context_string = ""
interaction_system_prompt = {
    "role": "system",
    "content": f"{INTERACTION_BASE_SYSTEM_PROMPT} {visual_context_string}"
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
    rospy.loginfo(".. Initializing LLM Manager Node")
    rate = rospy.Rate(10)
    llm_sentence_pub = rospy.Publisher("llm_sentence",String, queue_size=5)
    stt_sentence_pub = rospy.Publisher('stt_sentence', String, queue_size=5)

    def handle_update_visual_prompt(req):
        global visual_additional_system_prompt

    # setup inject_user_input service
    def handle_inject_user_input(req):
        fake_user_input = str(req.fakeUserInput)
        stt_sentence_pub.publish(fake_user_input)
        return InjectUserInputResponse("success")
    rospy.Service("inject_user_input", InjectUserInput, handle_inject_user_input)

    # setup inject_llm_response service
    def handle_inject_llm_response(req):
        global message_log
        fake_llm_response = str(req.fakeLLMResponse)
        message_log.append(create_assist_msg(fake_llm_response))
        try:
            set_stt_active = rospy.ServiceProxy("stt_control", SttControl)
            set_stt_active(False)
        except:
            rospy.logwarn("LLM Manager: STT control failed.")
        llm_sentence_pub.publish(fake_llm_response)
        return InjectLLMResponseResponse("success")
    rospy.Service("inject_llm_response",InjectLLMResponse, handle_inject_llm_response)


    def get_gpt_response(messages):
        global message_log, interaction_system_prompt
        completion = open_ai_client.chat.completions.create(model="gpt-4o-mini", messages=messages)
        gpt_msg = completion.choices[0].message.content.strip()

        message_log.append(create_assist_msg(gpt_msg))
        
        # if (len(message_log) >= 25):
        #     message_log.pop(0)
        #     message_log.pop(0)
        #     message_log.insert(0,system_prompt)

        return gpt_msg

    def on_user_speech(data):

        try:
            set_stt_active = rospy.ServiceProxy("stt_control", SttControl)
            set_stt_active(False)
        except:
            rospy.logwarn("LLM Manager: STT control failed.")

        # ignore null or extrememly short cut-off words
        if len(str(data.data)) <= 4:
            try:
                set_stt_active = rospy.ServiceProxy("stt_control", SttControl)
                set_stt_active(True)
            except:
                rospy.logwarn("LLM Manager: STT control failed.")
            return

        global message_log, interaction_system_prompt
        message_log.append(create_user_msg(str(data.data)))
        message_log_w_sys_prompt = [interaction_system_prompt]
        message_log_w_sys_prompt.extend(list(message_log))

        try:
            response = get_gpt_response(message_log_w_sys_prompt)
            llm_sentence_pub.publish(response)
        except:
            rospy.logerr("LLM manager: OpenAI API call failed. Consider manual override.")


        

    rospy.Subscriber("stt_sentence", String, on_user_speech)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        llm_manager()
    except rospy.ROSInterruptException:
         pass
    

    
    
    
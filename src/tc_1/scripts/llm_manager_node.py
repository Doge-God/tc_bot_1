#! /usr/local/bin/python3.10

# 

# /usr/bin/python3

import rospy
from std_msgs.msg import String
import os
from dotenv import load_dotenv
from openai import OpenAI
from cv_bridge import CvBridge
import base64
import cv2
from sensor_msgs.msg import CompressedImage, Image

from tc_1.srv import SttControl
from tc_1.srv import InjectUserInput, InjectUserInputResponse
from tc_1.srv import InjectLLMResponse, InjectLLMResponseResponse
from tc_1.srv import UpdateVisualContext, UpdateVisualContextResponse
from collections import deque

load_dotenv()

OPEN_AI_KEY = os.getenv("OPEN_AI_KEY")
INTERACTION_BASE_SYSTEM_PROMPT = "You are a robot, you can hear, see, and move around. You talk like Hal9000. You give very brief responses. Omit any formatting in your response."

VISUAL_BASE_SYSTEM_PROMPT ="Describe what you see. Do not use \"The image shows\"."
visual_additional_system_prompt = ""
visual_system_promp = {
    "role":"system",
    "content": f"{VISUAL_BASE_SYSTEM_PROMPT} {visual_additional_system_prompt}"
}

visual_context_string = ""
interaction_system_prompt = {
    "role": "system",
    "content": f"{INTERACTION_BASE_SYSTEM_PROMPT} This is what you see: {visual_context_string}"
}
message_log = deque([], maxlen=30)
open_ai_client = OpenAI(api_key=OPEN_AI_KEY)
bridge = CvBridge()

def create_user_msg(msg:str):
    return {"role":"user","content":msg}

def create_assist_msg(msg:str):
    return {"role":"assistant","content":msg}

def create_image_msg(base64_img):
    return {"role":"user","content":[
        {
            "type":"image_url",
            "image_url": {
                "url":f"data:image/jpeg;base64,{base64_img}"
            }
        }
    ]}

def create_system_prompt(msg:str):
    return {"role":"system", "content":msg}

def get_gpt_response(messages):
    global message_log, interaction_system_prompt
    completion = open_ai_client.chat.completions.create(model="gpt-4o-mini", messages=messages)
    gpt_msg = completion.choices[0].message.content.strip()

    message_log.append(create_assist_msg(gpt_msg))

    return gpt_msg
    

def llm_manager():
    rospy.init_node("llm_manager")
    rospy.loginfo(".. Initializing LLM Manager Node")
    rate = rospy.Rate(10)
    llm_sentence_pub = rospy.Publisher("llm_sentence",String, queue_size=5)
    stt_sentence_pub = rospy.Publisher('stt_sentence', String, queue_size=5)

    #========================================================================================
    #=============  SERVICE HANDLERS ========================================================
    #========================================================================================

    #### UPDATE VISUAL CONTEXT
    def handle_update_visual_context(req):
        global visual_context_string

        base_prompt = "Describe what you see. Do not use \"The image shows\"."
        extra_prompt = str(req.visualPrompt)
        ros_img:CompressedImage = rospy.wait_for_message("/camera/rgb/image_raw/compressed",CompressedImage)
        cv_img = bridge.compressed_imgmsg_to_cv2(ros_img, desired_encoding='passthrough')
        _, buffer = cv2.imencode('.jpg', cv_img)
        base64_img = base64.b64encode(buffer).decode('utf-8')

        system_prompt = create_system_prompt(f"{base_prompt} {extra_prompt}")
        image_message = create_image_msg(base64_img)

        new_visual_context = get_gpt_response([system_prompt, image_message])
        visual_context_string = new_visual_context

        return UpdateVisualContextResponse(new_visual_context)
    rospy.Service("update_visual_context", UpdateVisualContext, handle_update_visual_context)


    #### INJECT UPSER INPUT
    def handle_inject_user_input(req):
        fake_user_input = str(req.fakeUserInput)
        stt_sentence_pub.publish(fake_user_input)
        return InjectUserInputResponse("success")
    rospy.Service("inject_user_input", InjectUserInput, handle_inject_user_input)

    #### INJECT LLM RESPONSE
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
    #========================================================================================

    


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
    

    
    
    
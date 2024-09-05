#! /usr/local/bin/python3.10

# 

# /usr/bin/python3

import rospy
from std_msgs.msg import String
import os
from dotenv import load_dotenv
from openai import OpenAI
from cv_bridge import CvBridge
import datetime
import base64
import cv2
from sensor_msgs.msg import CompressedImage, Image

from tc_1.srv import SttControl
from tc_1.srv import InjectUserInput, InjectUserInputResponse
from tc_1.srv import InjectLLMResponse, InjectLLMResponseResponse
from tc_1.srv import UpdateVisualContext, UpdateVisualContextResponse
from tc_1.srv import UpdateSystemPrompt, UpdateSystemPromptResponse
from tc_1.srv import ClearChatLog, ClearChatLogResponse
from collections import deque

load_dotenv()

class LLMManager():
    def __init__(self) -> None:
        self.OPEN_AI_KEY = os.getenv("OPEN_AI_KEY")
        self.LOG_FILE_PATH = "/home/adam/Desktop/conversation_logs.txt"
        self.add_log_entry(" ======== LLM MANAGER RESTARTED: NEW SESSION ========")

        # self.INTERACTION_BASE_SYSTEM_PROMPT = "You are a robot, you can hear, see, and move around. You give very brief responses. Omit any formatting in your response."

        self.additional_system_prompt = "You talk like HAL9000"
        self.visual_context_string = ""
        self.interaction_system_prompt = {
            "role": "system",
            "content": f"{self.additional_system_prompt} This is what you see: {self.visual_context_string}"
        }
        self.message_log = deque([], maxlen=30)
        self.open_ai_client = OpenAI(api_key=self.OPEN_AI_KEY)
        self.bridge = CvBridge()

    def add_log_entry(self,input_string:String):
        # Get the current timestamp
        timestamp = datetime.datetime.now().strftime("[%Y-%m-%d %H:%M:%S]")
        
        # Format the string to include the timestamp
        formatted_string = f"{timestamp} {str.strip(input_string)}\n"
        
        # Open the file in append mode and write the formatted string
        with open(self.LOG_FILE_PATH, 'a+') as file:
            file.write(formatted_string)


    def refresh_system_prompt(self):
        self.interaction_system_prompt = {
            "role": "system",
            "content": f"{self.additional_system_prompt} This is what you see: {self.visual_context_string}"
        }

    def create_user_msg(self,msg:str):
        return {"role":"user","content":msg}

    def create_assist_msg(self,msg:str):
        return {"role":"assistant","content":msg}

    def create_image_msg(self,base64_img):
        return {"role":"user","content":[
            {
                "type":"image_url",
                "image_url": {
                    "url":f"data:image/jpeg;base64,{base64_img}"
                }
            }
        ]}

    def create_system_prompt(self,msg:str):
        return {"role":"system", "content":msg}

    def get_gpt_response(self,messages):
        completion = self.open_ai_client.chat.completions.create(model="gpt-4o-mini", messages=messages)
        gpt_msg = completion.choices[0].message.content.strip()

        self.message_log.append(self.create_assist_msg(gpt_msg))

        return gpt_msg
        

    def run(self):
        rospy.init_node("llm_manager")
        rospy.loginfo(".. Initializing LLM Manager Node")
        rate = rospy.Rate(10)
        llm_sentence_pub = rospy.Publisher("llm_sentence",String, queue_size=5)
        stt_sentence_pub = rospy.Publisher('stt_sentence', String, queue_size=5)
        img_description_pub = rospy.Publisher('img_description', String, queue_size=5)

        #========================================================================================
        #============= UTIL FUNCTIONS ===========================================================
        #========================================================================================

        def get_base64_image():
            ros_img:CompressedImage = rospy.wait_for_message("/camera/rgb/image_raw/compressed",CompressedImage)
            cv_img = self.bridge.compressed_imgmsg_to_cv2(ros_img, desired_encoding='passthrough')
            _, buffer = cv2.imencode('.jpg', cv_img)
            base64_img = base64.b64encode(buffer).decode('utf-8')
            return base64_img

        #========================================================================================
        #=============  SERVICE HANDLERS ========================================================
        #========================================================================================

        #### CLEAR CHAT LOG
        def handle_clear_chat_log(_):
            self.message_log = []
            self.add_log_entry("==== Warning: chat log cleared. ====")
            return ClearChatLogResponse("Success")
        rospy.Service("clear_chat_log", ClearChatLog, handle_clear_chat_log)

        #### UPDATE VISUAL CONTEXT
        def handle_update_visual_context(req):

            base_prompt = "Do not use \"The image shows\"."
            extra_prompt = str(req.visualPrompt)
            
            base64_img = get_base64_image()

            system_prompt = self.create_system_prompt(f"{base_prompt} {extra_prompt}")
            image_message = self.create_image_msg(base64_img)

            new_visual_context = self.get_gpt_response([system_prompt, image_message])
            self.visual_context_string = new_visual_context
            img_description_pub.publish(new_visual_context)
            self.add_log_entry(f" == Visual: {str.strip(new_visual_context)}")
            self.refresh_system_prompt()

            return UpdateVisualContextResponse(new_visual_context)
        rospy.Service("update_visual_context", UpdateVisualContext, handle_update_visual_context)

        #### UPDATE SYSTEMP PROMPT
        def handle_update_system_prompt(req):

            self.additional_system_prompt = str(req.systemPrompt)
            self.refresh_system_prompt()

            return UpdateSystemPromptResponse(self.additional_system_prompt)
        rospy.Service("update_system_prompt", UpdateSystemPrompt, handle_update_system_prompt)

        #### INJECT UPSER INPUT
        def handle_inject_user_input(req):
            fake_user_input = str(req.fakeUserInput)
            stt_sentence_pub.publish(fake_user_input)
            return InjectUserInputResponse("success")
        rospy.Service("inject_user_input", InjectUserInput, handle_inject_user_input)

        #### INJECT LLM RESPONSE
        def handle_inject_llm_response(req):
            fake_llm_response = str(req.fakeLLMResponse)
            self.message_log.append(self.create_assist_msg(fake_llm_response))
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

            self.message_log.append(self.create_user_msg(str(data.data)))
            message_log_w_sys_prompt = [self.interaction_system_prompt]
            message_log_w_sys_prompt.extend(list(self.message_log))

            try:
                response = self.get_gpt_response(message_log_w_sys_prompt)
                llm_sentence_pub.publish(response)
            except:
                rospy.logerr("LLM manager: OpenAI API call failed. Consider manual override.")
            
            # Log entries
            if self.LOG_FILE_PATH:
                self.add_log_entry(f"User: {str(data.data)}")
                self.add_log_entry(f"Response: {response}")

        rospy.Subscriber("stt_sentence", String, on_user_speech)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    manager = LLMManager()
    try:
        manager.run()
    except rospy.ROSInterruptException:
         pass
    

    
    
    
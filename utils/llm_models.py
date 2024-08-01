from multiprocessing import Value
from openai import OpenAI
import vertexai
import os
from vertexai.generative_models import (
    GenerationConfig,
    GenerativeModel,
    Image,
    Part,
)
import base64
import asyncio
import utils.utils as utils
import tenacity
from tenacity import retry
import json
import xml.etree.ElementTree as ET
import tiktoken

def error_callback(response,retry_reason,required_keys):
    if (retry_reason == 'json'):
        return [
            {
                "role":"assistant",
                "content":[
                    {
                        "type":"text",
                        "text": response
                    }
                ]
            },
            {
                "role":"user",
                "content":[
                    {
                        "type":"text",
                        "text": "Your output is not parseable with python's json.loads function. Please try again."
                    }
                ]
            }
        ]
    elif(retry_reason == 'keys'):
        return [
            {
                "role":"assistant",
                "content":[
                    {
                        "type":"text",
                        "text": response
                    }
                ]
            },
            {
                "role":"user",
                "content":[
                    {
                        "type":"text",
                        "text": f"Your output is does not have the correct output keys. Your output should be parseable by the json.loads python function and have the following keys:{required_keys}"
                    }
                ]
            }
        ]
    elif(retry_reason == 'xml'):
        return [
            {
                "role":"assistant",
                "content":[
                    {
                        "type":"text",
                        "text": response
                    }
                ]
            },
            {
                "role":"user",
                "content":[
                    {
                        "type":"text",
                        "text": "Your output is not a valid XML tree. It is not parseable with python's xml.etree.ElementTree.fromstring function. Please try again."
                    }
                ]
            }
        ]
    else:
        return []

class GPTModel:
    def __init__(self,model_name,project_id,tools = None) -> None:
        self.model_name = model_name
        #self.openai_temperature = config.OPENAI_TEMPERATURE
        self.client=OpenAI(project=project_id)
        self.messages = []
        self.tools = tools      
    
    def get_response(self,messages):
        response = self.client.chat.completions.create(
                                model = self.model_name,
                                response_format={"type":'json_object'},
                                messages=messages
                                )
        return response.choices[0].message.content    

    def is_response_valid(self,response,expected_keys=None):
        try:
            output = json.loads(response)
        except json.JSONDecodeError as e:
            print("JSONDecodeError")
            return False      
        
        if (float(response.count('\t'))/len(response))>0.5 or (float(response.count('\n'))/len(response))>0.5:
            return False
        
        if expected_keys!=None:
            output = {k.lower().replace('_','').replace(' ',''): v for k,v in output.items()}
            if sorted(output.keys()) != sorted(expected_keys):
                print(f"Incorrect keys:{output.keys()}")
                return False
        
        return True

models = {
    'gpt':GPTModel
}
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
import requests
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
    def __init__(self,model_name,tools = None,debug = False) -> None:
        self.model_name = model_name
        #self.openai_temperature = config.OPENAI_TEMPERATURE
        self.client=OpenAI()
        self.messages = []
        self.tools = tools      
        self.debug = debug
    
    def get_response(self,messages,response_format):
        return self.client.beta.chat.completions.parse(
            model = self.model_name,
            messages = messages,
            response_format = response_format #use structured format
        )

    def extract_response(self,response):
        output = response.choices[0].message
        if response.choices[0].finish_reason != 'stop' or output.refusal:
            return False
        return output.parsed

models = {
    'gpt':GPTModel
}
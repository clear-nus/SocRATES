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
import utils.utils as utils
class GPTModel:
    def __init__(self,config) -> None:
        self.model_name = config['MODEL_NAME']
        #self.openai_temperature = config.OPENAI_TEMPERATURE
        self.client=OpenAI()
    
    def get_response(self,**kwargs):
        response = self.client.chat.completions.create(
                    model = self.model_name,
                    response_format={"type":kwargs['format']},
                    messages=kwargs['messages']
                    )
        return response.choices[0].message.content
    
    def get_token_count(self,payload):
        token_count = 0
        for message in payload:
            token_count += utils.num_tokens_from_messages(message,self.model_name)
        return token_count
    
    def get_img_from_path(self,img_path):
        with open(img_path, "rb") as image_file:
            encoded_string = base64.b64encode(image_file.read())
        return encoded_string.decode("utf-8")
    
    def get_payload(self,**kwargs):
        payload = []
        content = kwargs['content']
        if 'system' in content.keys():
            payload.append({"role":"system",
                            "content": content['system']})
        user_content ={"role": "user","content":[]}
        
        for msg in content['user']:
            if msg['type'] == 'text':
                user_content['content'].append({"type": "text",
                                              "text": msg['content']})
            elif msg['type'] == 'image':
                encoded_img = utils.encode_image(msg['content'])
                user_content['content'].append({"type": "image_url",
                                              "image_url": {"url":f"data:image/jpeg;base64,{encoded_img}",
                                                            "detail":msg['detail']}})
        payload.append(user_content)
        return payload 
        
        
class GeminiModel:
    def __init__(self,config) -> None:
        self.vertexai_project = config['VERTEXAI_PROJECT']
        self.vertexai_region = config['VERTEXAI_REGION']
        self.model_name = config['MODEL_NAME']
        vertexai.init(project = self.vertexai_project, location = self.vertexai_region)
        self.model = GenerativeModel(
            model_name=self.model_name,
            )
        
    def get_response(self,**kwargs):
        response = self.model.generate_content(kwargs['messages'],kwargs['stream'])
        return response.text    
    
    def get_token_count(self,payload):
        return self.model.count_tokens(payload) 

    def get_payload(self,**kwargs):
        payload = []
        content = kwargs['content']      
        for msg in content['user']:
            payload.append({"type": "text",
                                            "text": msg['content']})
        return payload 
    
class Model:
    def __init__(self,config) -> None:
        if 'gemini' in config['MODEL_NAME']:
            self.model_type = 'gemini'
            self.model = GeminiModel(config)
        # elif 'llama' in config['MODEL_NAME']:
        #     self.model_type = 'llama'
        elif 'gpt' in config['MODEL_NAME']:
            self.model_type = 'gpt'
            self.model = GPTModel(config)
        else:
            print("Unrecognized Model")
            self.model_type = None
            self.model = None
        print(f"Using {self.model_type} model")
        
    def get_response(self,**kwargs):
        return self.model.get_response(**kwargs)

    def get_token_count(self,**kwargs):
        return self.model.get_token_count(**kwargs)

    def get_payload(self,**kwargs):
        return self.model.get_payload(**kwargs)
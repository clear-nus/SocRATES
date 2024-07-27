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
    def __init__(self,config,tools = None) -> None:
        self.model_name = config['MODEL_NAME']
        #self.openai_temperature = config.OPENAI_TEMPERATURE
        self.client=OpenAI()
        self.messages = []
        self.tools = tools
    
    def create_file(self,file_location,purpose):
        return self.client.files.create(
            file = open(file_location,"rb"), purpose = purpose
        )        
        
    def get_response(self,**kwargs):
        encoding = tiktoken.encoding_for_model('gpt-4-turbo')
        while True:
            messages = kwargs['messages']
            replies = []
            for attempt in tenacity.Retrying(stop=tenacity.stop_after_attempt(5),
                                             wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
                try:
                    for r in replies:
                        messages.append(r)
                        
                    response = self.client.chat.completions.create(
                                model = self.model_name,
                                response_format={"type":kwargs['format']},
                                messages=messages
                                )
                    output = response.choices[0].message.content
                    token_count = len(encoding.encode(output))
                    # check for bad \t output
                    if (float(output.count('\t'))/len(output))>0.5 or (float(output.count('\n'))/len(output))>0.5: 
                        replies = []
                        raise ValueError
                    #print(output)
                    if 'expected_keys' in kwargs:
                        #check if the output is json parseable otherwise retry
                        try:
                            output = json.loads(output)
                        except json.JSONDecodeError as e:
                            print("JSONDecodeError")
                            #replies = error_callback(response.choices[0].message.content,'json',[])
                            raise e
                        output = {k.lower().replace('_','').replace(' ',''): v for k,v in output.items()}
                        #if len(list(set(list(output.keys())).intersection(set(kwargs['expected_keys']))))!=len(list(output.keys())):
                        if sorted(output.keys()) != sorted(kwargs['expected_keys']):
                            print(f"Incorrect keys:{output.keys()}")
                            #replies = error_callback(response.choices[0].message.content,'keys',kwargs['expected_keys'])
                            raise ValueError
                        #check if the values o
                        if 'tree' in kwargs['expected_keys']:
                            try:
                                test_xml = ET.fromstring(output['tree'])  
                                print("Recieved Valid XML")
                            except:
                                print("ParseError")
                                replies = error_callback(response.choices[0].message.content,'xml',[])
                                raise ValueError
                            if utils.validate_bt(test_xml,kwargs["node_library"]):
                                print("Recieved Valid BT")
                                return output
                            else:
                                raise ValueError("Recieved JSON Parseable output but it is not a valid XML tree")
                        
                        if len(replies) == 0:
                            return output
                    else:
                        return output
                except (ValueError,json.JSONDecodeError) as e:
                    pass
    
    def get_response_ablation(self,**kwargs):
        encoding = tiktoken.encoding_for_model('gpt-4-turbo')
        while True:
            messages = kwargs['messages']
            replies = []
            for attempt in tenacity.Retrying(stop=tenacity.stop_after_attempt(5),
                                             wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
                try:
                    for r in replies:
                        messages.append(r)
                        
                    response = self.client.chat.completions.create(
                                model = self.model_name,
                                response_format={"type":kwargs['format']},
                                messages=messages
                                )
                    output = response.choices[0].message.content
                    token_count = len(encoding.encode(output))
                    # check for bad \t output
                    if (float(output.count('\t'))/len(output))>0.5 or (float(output.count('\n'))/len(output))>0.5: 
                        replies = []
                        raise ValueError
                    #print(output)
                    if 'expected_keys' in kwargs:
                        #check if the output is json parseable otherwise retry
                        try:
                            output = json.loads(output)
                        except json.JSONDecodeError as e:
                            print("JSONDecodeError")
                            #replies = error_callback(response.choices[0].message.content,'json',[])
                            raise e
                        output = {k.lower().replace('_','').replace(' ',''): v for k,v in output.items()}
                        #if len(list(set(list(output.keys())).intersection(set(kwargs['expected_keys']))))!=len(list(output.keys())):
                        if sorted(output.keys()) != sorted(kwargs['expected_keys']):
                            print(f"Incorrect keys:{output.keys()}")
                            #replies = error_callback(response.choices[0].message.content,'keys',kwargs['expected_keys'])
                            raise ValueError
                        
                        if 'tree' in kwargs['expected_keys']:
                            try:
                                for human, tree in output['tree'].items():
                                    test_xml = ET.fromstring(tree)
                            except:
                                print("ParseError")
                                replies = error_callback(response.choices[0].message.content,'xml',[])
                                raise ValueError
                            
                        print("Recieved JSON Parseable output")
                        return output
                    else:
                        return output
                except (ValueError,json.JSONDecodeError) as e:
                    pass
    
        
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
        
        for msg in content:
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
    
    @retry(wait=tenacity.wait_random_exponential(multiplier=1, max=40), stop=tenacity.stop_after_attempt(3))
    def chat_completion_request(self,messages, tools=None, tool_choice=None):
        try:
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
                tools=tools, #provide tools (funcitons/APIs) the model can use. 
                tool_choice=tool_choice, #force model to use this tool for this request
            )
            return response
        except Exception as e:
            print("Unable to generate ChatCompletion response")
            print(f"Exception: {e}")
            return e
    
    def clear_chat_history(self):
        self.messages = []
     
    def converse(self,new_message,tools=None):
        self.messages.append({"role":"user","content":new_message})
        if tools!=False:
            chat_response = self.chat_completion_request(self.messages,self.tools)
        else:
            chat_response = self.chat_completion_request(self.messages,tools)
            
        assistant_message = chat_response.choices[0].message
        self.messages.append(assistant_message)
        return assistant_message
        
    def clear_assistant_files(self):
        for file in list(self.client.files.list()):
            self.client.files.delete(file.id)
        
    def clear_assistants(self):
        for asst in list(self.client.beta.assistants.list()):
            try:
                self.client.beta.assistants.delete(asst.id)
            except NotFoundError:
                pass
        print(f"Running assistants: {len(list(self.client.beta.assistants.list()))}")
    
    def get_assistant_by_name(self,name):
        for asst in list(self.client.beta.assistants.list()):
            if asst.name == name:
                return asst
        print(f"Assistant not found, create again")
        return None
    
    def get_files_by_name(self,name):
        for fil in list(self.client.files.list()):
            if fil.filename == name:
                return fil
        print("File not found, upload again")
        return None
    
    def create_and_run_thread(self,content,attachments,assistant_id):
        #create empty thread
        thread = self.client.beta.threads.create()
        #add messages to thread
        thread_message = self.client.beta.threads.messages.create(
            role = "user",
            thread_id = thread.id,
            content = content,
            attachments= attachments
            
        )
        run = self.client.beta.threads.runs.create(
                thread_id=thread.id,
                assistant_id=assistant_id
            )
        return thread.id, run.id
        
    
    def poll_run_result(self,thread_id,run_id):
        #https://community.openai.com/t/assistant-calls-and-responses/600746/8
        completed = False
        respmsg = None
        while not completed:
            try:
                response = self.client.beta.threads.runs.retrieve(
                    thread_id=thread_id,
                    run_id=run_id
                )

                if response.status == 'completed':
                    print('Run completed!')
                    respmsg = self.client.beta.threads.messages.list(thread_id)
                    print(respmsg.data[0].content[0].text.value)

                    outgoingMessages = respmsg.data[0].content[0].text.value
                    completed = True
                    # bstack.emit('chat.message', {'message': outgoingMessages})
                    break

                if response.status == 'expired':
                    print('Task expired!')
                    respmsg = self.client.beta.threads.messages.list(thread_id)
                    print(respmsg.data[0].content[0].text.value)

                    outgoingMessages = respmsg.data[0].content[0].text.value
                    completed = True
                    # bstack.emit('chat.message', {'message': outgoingMessages})
                    break

                if response.status == 'requires_action':
                    print('Run Action Required!')

                    # tool_outputs = []
                    # for ra in response.required_action.submit_tool_outputs.tool_calls:
                    #     if ra.type == 'function' and ra.function.name in functions:
                    #         print(f'Running {ra.function.name} with arguments {ra.function.arguments}')
                    #         ret = functions[ra.function.name](ra.function.arguments)
                    #         print('Returned', ret)
                    #         tool_outputs.append({
                    #             'tool_call_id': ra.id,
                    #             'output': json.dumps(ret)
                    #         })
                    #     else:
                    #         tool_outputs.append({
                    #             'tool_call_id': ra.id,
                    #             'output': 'There is a problem, function not found!'
                    #         })

                    # run = model.client.beta.threads.runs.submitToolOutputs(
                    #     thread.id,
                    #     runId,
                    #     {'tool_outputs': tool_outputs}
                    # )
                    completed = True
                    break

                if response.status in ('failed', 'cancelled', 'expired', 'cancelling'):
                    print(f'Status: {response.status}')
                    completed = True
                    break

                if response.status in ('in_progress', 'queued'):
                    print(f'Run {response.status}')
                    asyncio.sleep(2)  # Poll every 2 seconds

            except Exception as e:
                print('Error in polling (retrying):', str(e))
                break
                # Handle the exception or retry logic as necessary   
        print(response.status)
        return response
        
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
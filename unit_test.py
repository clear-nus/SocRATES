import os
import json
import pprint
from utils.models import *
from tqdm import tqdm
MODEL="gpt-4o"
LOCATION = 'Warehouse_fixed'
SCENARIO = 'narrow_passageway'

model = GPTModel(config = dict(
    MODEL_NAME = MODEL
))
with open(os.path.join('unit_test',SCENARIO,'prompt.txt'),'r') as f:
    prompt = f.read()

for i in tqdm(range(10)):

    payload = model.get_payload(
        content={
            'system':[],
            'user':[
                {
                    'type':'text',
                    'content':prompt
                        
                },
                {
                    'type':'image',
                    'content': os.path.join('locations',LOCATION,'scene_graph.png'),
                    'detail':'high'
                }            
            ]
        }
    )

    response = model.get_response(
        messages = payload,
        format = 'text'
    )
    
    with open(os.path.join('unit_test',SCENARIO,'api_responses.txt'),'a') as f:
        f.writelines('\n'+'==========================='+'\n'+response)
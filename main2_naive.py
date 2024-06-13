import os
import json
import pprint
import re
from PIL import Image as PILImage
from Prompts import *
import pickle as pkl
import yaml
from Prompts import abalation_prompt
from utils.config import *
from utils.models import *
import copy
import subprocess
import pathlib
from pathlib import Path
from tenacity import RetryError
N = 5
scenarios = ['group']
model = GPTModel(config = dict(
    MODEL_NAME = MODEL
))

with open(os.path.join('locations',LOCATION,'scene_graph.json'),'r') as f:
    scene_graph = json.load(f)

scgraph = utils.SceneGraph(scene_graph)
encoded_img = utils.encode_image(os.path.join('locations',LOCATION,'scene_graph.png'))
node_types = []
for node in scgraph.get_parent_nodes():
    if scgraph.graph.nodes[node]['type'] not in node_types:
        node_types.append(scgraph.graph.nodes[node]['type'])

for scenario in scenarios:
    for iter in range(N):
        RESPONSES_DIR = os.path.join(f"responses/ablation/naive/{scenario}_{iter}")
        os.makedirs(RESPONSES_DIR,exist_ok=True)
        if QUERY_SC:    
            abQ = AbalationQuery()
            abQ_full_prompt = abQ.get_full_prompt(
                context=CONTEXT,
                task=TASK,
                rough_scenario=ROUGH_SCENARIOS[scenario],
                location = LOCATION_DESC
            )
            scq_response_json = model.get_response(messages = abQ_full_prompt,format = "json_object",expected_keys=abQ.required_output_keys)
            if SAVE_SC_RESPONSE:
                print("Saving scenario response")
                with open(os.path.join(RESPONSES_DIR,'reponse_sc.json'),'w') as f:
                    json.dump(scq_response_json,f)
                    
            # print("================PROPOSED SCENARIO:===============")
            # print(scq_response_json)
            # user_input = input("Continue with proposed scenario? ('no' to requery) (yes/no): ")
            # if user_input.lower() == "yes" or user_input.lower() == "y":
            #     print("Continuing...")
            #     break
            
            print('USING GENERATED SCENARIO')
            scenario_desc = scq_response_json['scenariodescription']
            num_humans = scq_response_json['numberofhumans']
            behav_desc = scq_response_json['humanbehavior']

        else:
            if USE_HANDCRAFTED_SCENARIO:
                print('USING HANDCRAFTED SCENARIO')
                print('Scenario Description:')
                scenario_desc = scenario_desc_hc
                num_humans = num_humans_hc
                behav_desc = behav_desc_hc
            else:
                print("Loading prior scenario response")
                with open(os.path.join(RESPONSES_DIR,'reponse_sc.json'),'r') as f:
                    scq_response_json = json.load(f)
                
                scq_response_json = {k.lower().replace('_','').replace(' ',''): v for k,v in scq_response_json.items()}
                scenario_desc = scq_response_json['scenariodescription']
                behav_desc =  {k.lower().replace('_','').replace(' ',''): v for k,v in scq_response_json['humanbehavior'].items()}
                num_humans =  scq_response_json['numberofhumans']
                    
        print(scenario_desc)
        print(f'Number of Humans:{num_humans}')
        print(f'Behaviors:{behav_desc}')

        flq = FLocationQuery()

        flq_full_prompt = flq.get_full_prompt(
            scene_graph = str(scene_graph),
            node_types = ','.join(node_types),
            encoded_img = encoded_img,
            sc_desc = scenario_desc
        )
        if QUERY_TRAJ:
            print("Querying LLM for Trajectories")
            valid_trajectories = 0
            all_trajectories_valid = False
            payload = flq_full_prompt.copy()
            while not all_trajectories_valid:
                retries = -1
                payload = flq_full_prompt.copy()
                while retries <=3: #retry until trajectories are valid (max 3)
                    all_trajectories_valid = True
                    flq_response_json = model.get_response(messages = payload,format = 'json_object',expected_keys=flq.required_output_keys)
                    if not isinstance(flq_response_json,dict):
                        retries+=1
                        continue
                    trajectories = flq_response_json['trajectories']
                    groupids = flq_response_json['groupids']
                    traj_valid = True
                    #test connectivity
                    for k,v in trajectories.items():
                        traj_valid,errors = scgraph.isvalidtrajectory(v)
                        if not traj_valid: #requery LLM with error message
                            print("Disconnected Trajectory Output, Retrying")
                            valid_trajectories=0
                            all_trajectories_valid = False
                            reply = f"""
                                    You made the following mistakes:
                                    <ERRORS>
                                    Retry and Return the answer in the same JSON format.
                                    """
                            error_string = ""
                            for err in errors:
                                error_string+=f"There is no edge connecting {err[0]} and {err[1]}!"
                                
                            reply = reply.replace('<ERRORS>',error_string)
                            payload.append(
                                {
                                    "role":"assistant",
                                    "content":[
                                        {
                                            "type":"text",
                                            "text": str(flq_response_json)
                                        }
                                    ]
                                }    
                                )
                            payload.append({
                                    "role": "user", 
                                    "content": [
                                        {
                                            "type":"text",
                                            "text": reply
                                        }
                                            ]
                                        }
                                        )
                            
                            retries+=1
                            break
                        else:
                            valid_trajectories+=1
                
                    if all_trajectories_valid:
                        break
            groupids =  {k.lower().replace('_','').replace(' ',''): v for k,v in flq_response_json['groupids'].items()}
            trajectories =  {k.lower().replace('_','').replace(' ',''): v for k,v in flq_response_json['trajectories'].items()}

            if SAVE_TRAJ_RESPONSE:
                print("Saving trajectory response")
                with open(os.path.join(RESPONSES_DIR,'reponse_traj.json'),'w') as f:
                    json.dump(flq_response_json,f)
                    
        else:
            assert LOAD_TRAJ_RESPONSE == True
            print("Loading prior fine location response")
            with open(os.path.join(RESPONSES_DIR,'reponse_traj.json'),'r') as f:
                flq_response_json = json.load(f)
            
            flq_response_json = {k.lower().replace('_','').replace(' ',''): v for k,v in flq_response_json.items()}
            groupids =  {k.lower().replace('_','').replace(' ',''): v for k,v in flq_response_json['groupids'].items()}
            trajectories =  {k.lower().replace('_','').replace(' ',''): v for k,v in flq_response_json['trajectories'].items()}


        world_trajectories = {}        
        for k,v in trajectories.items():
            world_trajectories[k] = []
            for l in v:
                world_trajectories[k].append(utils.pix2world(scgraph.graph.nodes[l]['pos']))

        print(f'OUTPUT TRAJECTORIES:{trajectories}')
        print(f'GROUP:{flq_response_json["groupids"]}')
        
        #writing to HuNavSim files
        print("ADDING TRAJECTORIES TO SIM YAML FILES")
        agents_yaml = {'hunav_loader': {'ros__parameters': {'map': LOCATION,
        'publish_people': True,
        'agents': []}}}
        blank_human = {'id': None,
            'skin': 0,
            'behavior': 0,
            'group_id': -1,
            'max_vel': 1.5,
            'radius': 0.4,
            'init_pose': {'x': None, 'y': None, 'z': 1.25, 'h': 0.0},
            'goal_radius': 0.3,
            'cyclic_goals': False,
            'goals': [],
            }
        agents = {}


        for i in range(len(trajectories.keys())-1):
            agents_yaml['hunav_loader']['ros__parameters']['agents'].append(f'agent{i}')
            agents[f'agent{i}'] = copy.deepcopy(blank_human)
            agents[f'agent{i}']['id'] = i
            agents[f'agent{i}']['behavior'] = 7+i
            agents[f'agent{i}']['group_id'] = groupids[f'human{i+1}']
            for j,g in enumerate(world_trajectories[f'human{i+1}']):
                if j == 0:
                    agents[f'agent{i}']['init_pose'] = {
                        'x':g[0],
                        'y':g[1],
                        'z':1.25,
                        'h':0.0,
                    }
                    if len(world_trajectories[f'human{i+1}']) <= 1:
                        agents[f'agent{i}']['goals'].append(f'g{1}')
                        agents[f'agent{i}'][f'g{1}'] = {
                            'x':g[0],
                            'y':g[1],
                            'h':1.25
                        }
                else:
                    agents[f'agent{i}']['goals'].append(f'g{j}')
                    agents[f'agent{i}'][f'g{j}'] = {
                        'x':g[0],
                        'y':g[1],
                        'h':1.25
                    }
        agents_yaml['hunav_loader']['ros__parameters'].update(agents)
        
        with open(os.path.join(RESPONSES_DIR,'custom_agents.yaml'),'w') as f:
            yaml.dump(agents_yaml,f)

        with open(os.path.join(RESPONSES_DIR,'robot.yaml'),'w') as f:
            yaml.dump({
            'x_pose': world_trajectories['robot'][0][0],
            'y_pose': world_trajectories['robot'][0][1],
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 3.14
        },f)
            
        behav_desc =  {k.lower().replace('_','').replace(' ',''): v for k,v in scq_response_json['humanbehavior'].items()}
        print(behav_desc)
        
        for i in range(len(list(behav_desc.keys()))):
            print(i)
            btq = BTQuery()
            if QUERY_BT:
                print(f'Querying for Human {i+1}')
                #print(behav_desc[f'human{i+1}']['Behavior Towards Robot'])
                btq_full_prompt = btq.get_full_prompt(behavior = behav_desc[f'human{i+1}'])
                btq_response_json = model.get_response(messages = btq_full_prompt,format = 'json_object',expected_keys=btq.required_output_keys)
                if SAVE_BT_RESPONSE:
                    print(f"Saving BT {i+1} response")
                    with open(os.path.join(RESPONSES_DIR,f'reponse_bt_{i+1}.json'),'w') as f:
                        json.dump(btq_response_json,f)                
                
            else:
                assert LOAD_BT_RESPONSE == True
                print(f'Loading prior BT {i+1} response')
                with open(os.path.join(RESPONSES_DIR,f'reponse_bt_{i+1}.json'),'r') as f:
                    btq_response_json = json.load(f)
            
            print('================Proposed Behavior:================') 
            print(json.dumps(btq_response_json,indent=4))
            print("Continuing...")
            bt_xml = btq_response_json['tree']
            for k,v in btq_response_json.items():
                if 'custom' in str.lower(k):
                    custom_node_requests.append(v)
            print(os.path.join(HUNAV_SIM_BT_FOLDER,f'LLMBT_{i}.xml'))
            with open(os.path.join(RESPONSES_DIR,f'LLMBT_{i}.xml'),'w') as f:
                f.write(bt_xml)
            print(f"Wrote BT to LLMBT_{i}.xml")

    


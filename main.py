# %%
import os
import json
import pprint
import re
from Prompts import *
import pickle as pkl
import yaml
from utils.config import *
from utils.models import *
import copy
import subprocess

# %% [markdown]
# ### Initialize Model

# %%
model = Model(config = dict(
    MODEL_NAME = MODEL
))

# %% [markdown]
# ### Query for Scenario

# %%
with open(os.path.join('locations',LOCATION,'scene_graph.json'),'r') as f:
    scene_graph = json.load(f)
scQ = ScenarioQuery()
scQ_full_prompt = scQ.get_full_prompt(context=CONTEXT,
    task=TASK,
    graph=scene_graph
)
print(f'Scenario Context: {CONTEXT}')
print(f'Scenario Task: {TASK}')
# print(scQ_txt_payload[1]['content'][0]['text'])
# print(f"""Num tokens from main message: {utils.num_tokens_from_messages(scQ_txt_payload[1],MODEL)}""")

# %% [markdown]
# ### Get Scenario Proposal from LLM

# %%
model.get_payload(content = scQ_full_prompt)

while True:# %%
    if QUERY_SC:
        print("Querying LLM for scenario")
        payload = model.get_payload(content = scQ_full_prompt)
        response = model.get_response(messages = payload,format = "json_object")
        scq_response_json = json.loads(response)
        if SAVE_SC_RESPONSE:
            print("Saving scenario response")
            with open('responses/reponse_sc.json','w') as f:
                json.dump(scq_response_json,f)
    else:
        assert LOAD_SC_RESPONSE == True
        print("Loading prior scenario response")
        with open('responses/reponse_sc.json','r') as f:
            scq_response_json = json.load(f)
    
    print("================PROPOSED SCENARIO:===============")
    print(json.dumps(scq_response_json,indent=4))
    user_input = input("Continue ? (yes/no): ")
    if user_input.lower() == "yes" or user_input.lower() == "y":
        
        print("Continuing...")
        break
        



# %%
if USE_HANDCRAFTED_SCENARIO:
    print('USING HANDCRAFTED SCENARIO')
    print('Scenario Description:')
    scenario_desc = scenario_desc_hc
    num_humans = num_humans_hc
    traj_desc = traj_desc_hc
    behav_desc = behav_desc_hc
    
    print(scenario_desc)
    print(f'Number of Humans:{num_humans}')
    print(f'Trajectory Description:{traj_desc}')
    print(f'Behaviors:{behav_desc}')
    
else:
    print('USING GENERATED SCENARIO')
    scenario_desc = scq_response_json['Scenario Description']
    num_humans = scq_response_json['Number of Humans']
    traj_desc = {}
    for k,v in scq_response_json['Trajectories'].items():
        traj_desc[k] = v.split(' -> ')
    behav_desc = scq_response_json['Behaviors']

# Assume the output is formatted correctly
expected_robot_behav_desc = scq_response_json['Expected Robot Behavior'] 
pranking_desc = scq_response_json['Principle Ranking'] 
reasoning_desc = scq_response_json['Reasoning'] 

# %%
coarse_locations = []
for k,v in traj_desc.items():
    for loc in v:
        match = re.search(r"\((\d+)\)", loc)
        if match:
            l = int(match.group(1))
            if l not in coarse_locations:
                coarse_locations.append(l)
print(f'================QUERYING LOCATIONS:================')
print(f'Coarse_locations:{coarse_locations}')

# %%
coarse_loc_imgs = []
[coarse_loc_imgs.append(os.path.join('locations',LOCATION,f'{img}.png')) for img in coarse_locations]
#for loc in coarse_locations:
#    coarse_loc_imgs[loc] = utils.encode_image(os.path.join('locations',location,f'{loc}.png'))
#coarse_loc_imgs['overhead'] = utils.encode_image(os.path.join('locations',location,'overhead_annotated.png'))
#coarse_loc_imgs = 
coarse_loc_imgs.append(os.path.join('locations',LOCATION,'overhead_annotated.png'))

# %% [markdown]
# ### Fine Locations Query

# %%
flocationQ = FLocationQuery(
    loc_imgs = coarse_loc_imgs
)

flq_full_prompt = flocationQ.get_full_prompt(
    scene_graph = str(scene_graph),
    num_humans = num_humans,
    traj_desc = traj_desc,
    sc_desc = scenario_desc
)

# %%
pprint.pprint(flq_full_prompt)

# %% [markdown]
# ### Get Fine Locations from LLM

# %%
if QUERY_LOC:
    print("Querying LLM for fine locations")
    payload = model.get_payload(content = flq_full_prompt)
    response = model.get_response(messages = payload,format = 'json_object')
    flq_response_json = json.loads(response)
    if SAVE_LOC_RESPONSE:
        print("Saving fine location response")
        with open('responses/reponse_loc.json','w') as f:
            json.dump(flq_response_json,f)
else:
    assert LOAD_LOC_RESPONSE == True
    print("Loading prior fine location response")
    with open('responses/reponse_loc.json','r') as f:
        flq_response_json = json.load(f)

print(json.dumps(flq_response_json,indent=4))
# %%
trajectories = {}
for k,v in flq_response_json['Trajectory'].items():
    trajectories[k] = []
    for loc in v.split(' -> '):
        trajectories[k].append(loc.split('-')[1])

print(f'OUTPUT TRAJECTORIES:{trajectories}')
print(f'GROUP:{flq_response_json["Group"]}')

# %%
### Get world location of the trajectories
with open(os.path.join('locations',LOCATION,'annotated_world_coords.pkl'),'rb') as f:
    flwc = pkl.load(f)
# with open(os.path.join('locations',LOCATION,'fine_loc_world_coords.pkl'),'rb') as f:
#     flwc = pkl.load(f)

# %% [markdown]
# annotated_positions = {}
# starti = 1
# for k,v in flwc.items():
#     startc = 65    
#     for l in v:
#         annotated_positions[str(starti)+chr(startc)] = (l[0],-l[1])
#         startc+=1
#     starti+=1
# print(annotated_positions)
# with open(os.path.join('locations',LOCATION,'annotated_world_coords.pkl'),'wb') as f:
#     pkl.dump(annotated_positions,f)

# %%
trajectories_world_coords = {}
for k,v in trajectories.items():
    trajectories_world_coords[k] = []
    for loc in v:
        trajectories_world_coords[k].append(flwc[loc])
print(trajectories_world_coords)

# %% [markdown]
# #### Add fine locations to sim yaml files

# %%
print("ADDING FINE LOCATIONS TO SIM YAML FILES")
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

# %%
for i in range(num_humans):
    agents_yaml['hunav_loader']['ros__parameters']['agents'].append(f'agent{i}')
    agents[f'agent{i}'] = copy.deepcopy(blank_human)
    agents[f'agent{i}']['id'] = i
    agents[f'agent{i}']['behavior'] = 7+i
    agents[f'agent{i}']['group_id'] = flq_response_json['Group'][f'Human {i+1}']
    for j,g in enumerate(trajectories_world_coords[f'Human {i+1}']):
        if j == 0:
            agents[f'agent{i}']['init_pose'] = {
                'x':g[0],
                'y':g[1],
                'z':1.25,
                'h':0.0,
            } 
        else:
            agents[f'agent{i}']['goals'].append(f'g{j}')
            agents[f'agent{i}'][f'g{j}'] = {
                'x':g[0],
                'y':g[1],
                'h':1.25
            }
agents_yaml['hunav_loader']['ros__parameters'].update(agents)

# %%
with open(HUNAV_SIM_AGENTS_FILE,'w') as f:
    yaml.dump(agents_yaml,f)

with open(os.path.join(HUNAV_GAZEBO_WRAPPER_DIR,'config','robot.yaml'),'w') as f:
    yaml.dump({
    'x_pose': trajectories_world_coords['Robot'][0][0],
    'y_pose': trajectories_world_coords['Robot'][0][1]
},f)

# %% [markdown]
# ### Query LLM for BT for humans

# %%
print("===================QUERYING for BTs:=================")
custom_node_requests = []

for i in range(num_humans):
    btq = BTQuery()
    while True:
        if QUERY_BT:
            print(f'Querying for Human {i+1}')
            print(behav_desc[f'Human {i+1}'])
            btq_full_prompt = btq.get_full_prompt(behavior = behav_desc[f'Human {i+1}'])
            payload = model.get_payload(content = btq_full_prompt)
            response = model.get_response(messages = payload,format = 'json_object')
            btq_response_json = json.loads(response)
            if SAVE_BT_RESPONSE:
                print(f"Saving BT {i+1} response")
                with open(f'responses/reponse_bt_{i+1}.json','w') as f:
                    json.dump(btq_response_json,f)
        else:
            assert LOAD_BT_RESPONSE == True
            print(f'Loading prior BT {i+1} response')
            with open(f'responses/reponse_bt_{i+1}.json','r') as f:
                btq_response_json = json.load(f)
        
        print('================Proposed Behavior:================')    #print(btq_response_json.keys())
        print(json.dumps(btq_response_json,indent=4))
        

        user_input = input("Continue ? (yes/no): ")
        if user_input.lower() == "yes" or user_input.lower() == "y":
            print("Continuing...")
            bt_xml = btq_response_json['Tree']
            for k,v in btq_response_json.items():
                if 'custom' in str.lower(k):
                    custom_node_requests.append(v)
            with open(os.path.join(HUNAV_SIM_BT_FOLDER,f'LLMBT_{i}.xml'),'w') as f:
                f.write(bt_xml)
            print(f"Wrote BT to LLMBT_{i}.xml")
            break

# %% [markdown]
# ### Query LLM for custom Nodes and Auxillary functions

# %%
print("==================QUERYING for Custom Nodes:=================")
print(custom_node_requests)

# %%
for i in range(len(custom_node_requests)):
    while True:
        if QUERY_AUX:
            print(f'Querying for custom node {i}')
            ctnq = NodeQuery()
            ctnq_full_prompt = ctnq.get_full_prompt(description = custom_node_requests[i])
            payload = model.get_payload(content = ctnq_full_prompt)
            response = model.get_response(messages = payload,format = 'json_object')
            ctnq_response_json = json.loads(response)
            if SAVE_AUX_RESPONSE:
                print(f"Saving AUX {i} response")
                with open('responses/reponse_aux.json','w') as f:
                    json.dump(ctnq_response_json,f)
        else:
            assert LOAD_AUX_RESPONSE == True
            print(f'Loading prior AUX {i} response')
            with open('responses/reponse_aux.json','r') as f:
                ctnq_response_json = json.load(f)
        print(json.dumps(ctnq_response_json,indent=4))
        if user_input.lower() == "yes" or user_input.lower() == "y":
            print("Continuing...")
            break
            
    with open(os.path.join('templates','extended_bt_functions.cpp'),'r') as f:
        btf_cpp = f.read()

    with open(os.path.join('templates','extended_bt_functions.hpp'),'r') as f:
        btf_hpp = f.read()

    #register node in bt_node.cpp
    with open(os.path.join('templates','extended_agent_manager.cpp'),'r') as f:
        agm_cpp = f.read()

    with open(os.path.join('templates','extended_agent_manager.hpp'),'r') as f:
        agm_hpp = f.read()

    with open(os.path.join('templates','extended_bt_node.cpp'),'r') as f:
        btn_cpp = f.read()

    with open(os.path.join('templates','extended_bt_node.hpp'),'r') as f:
        btn_hpp = f.read()
    
    #Write functions to extended_bt_functions.cpp file
    btf = ctnq_response_json['NODE_DEFINITION']
    btf = btf.replace('BT::NodeStatus BTfunctions::','BT::NodeStatus BTfunctionsExt::')
    btf_name = ctnq_response_json['NODE_NAME']
    btf_type = ctnq_response_json['NODE_TYPE'].lower().capitalize()
    btfn_name = btf_name[0].lower() + btf_name[1:]
    btf_header = ctnq_response_json['NODE_HEADER']

    btf_cpp = btf_cpp.replace('//<NEW FUNCTION>','//<NEW FUNCTION> \n' + btf)
    btf_hpp = btf_hpp.replace('//<NEW PUBLIC FUNCTION>','//<NEW PUBLIC FUNCTION> \n' + btf_header)
    
    #register BT nodes in extended_bt_node.cpp
    #   3 ports are available for each BT node:
    #       simple_port: agent_id
    #       visibleports: agent_id + distance
    #       portsNav: agent_id + timestep
    
    node_register = f"""factory_.registerSimple{btf_type}("{ctnq_response_json['NODE_NAME']}",std::bind(&BTfunctionsExt::{btfn_name},&btfunc_, _1),PORT);"""
    if ctnq_response_json['PORTS_USED'] == ['agent_id','distance']:
        node_register = node_register.replace('PORT','visibleports')
    elif ctnq_response_json['PORTS_USED'] == ['agent_id','time_step']:
        node_register = node_register.replace('PORT','portsNav')
    elif ctnq_response_json['PORTS_USED'] == ['agent_id']:
        node_register = node_register.replace('PORT','simple_port')
    elif ctnq_response_json['PORTS_USED'] == ['agent_id','message']:
        node_register = node_register.replace('PORT','portsMsg')
    else:
        node_register = node_register.replace(',PORT','')

    btn_cpp = btn_cpp.replace('//<NEW NODE REGISTER>','//<NEW NODE REGISTER> \n' + node_register)   

    #add aux functions
    for j,agmf in enumerate(ctnq_response_json['AUX_FUNCTIONS']):
        agmf = agmf.replace('void AgentManager::','void AgentManagerExt::')
        agm_cpp = agm_cpp.replace('//<NEW FUNCTION>','//<NEW FUNCTION> \n' + agmf)
        agm_hpp = agm_hpp.replace('//<NEW PUBLIC FUNCTION>','//<NEW PUBLIC FUNCTION> \n' + ctnq_response_json['AUX_FUNCTION_HEADERS'][j])

print('Writing to extended files')
with open(os.path.join(HUNAV_SIM_CPP_FOLDER,'extended_bt_functions.cpp'),'w') as f:
    f.writelines(btf_cpp)

with open(os.path.join(HUNAV_SIM_HPP_FOLDER,'extended_bt_functions.hpp'),'w') as f:
    f.writelines(btf_hpp)

with open(os.path.join(HUNAV_SIM_CPP_FOLDER,'extended_bt_node.cpp'),'w') as f:
    f.writelines(btn_cpp)

with open(os.path.join(HUNAV_SIM_HPP_FOLDER,'extended_bt_node.hpp'),'w') as f:
    f.writelines(btn_hpp)

with open(os.path.join(HUNAV_SIM_CPP_FOLDER,'extended_agent_manager.cpp'),'w') as f:
    f.writelines(agm_cpp)

with open(os.path.join(HUNAV_SIM_HPP_FOLDER,'extended_agent_manager.hpp'),'w') as f:
    f.writelines(agm_hpp)

# %% [markdown]
# ### Build Project

# %%
print('BUILDING PROJECT:')
s = subprocess.getstatusoutput(f' cd ~/catkin_ws && colcon build')
if s[0] == 0:
    print('Build Successful')
else:
    print('Build Failed')
    print(s[1]) 


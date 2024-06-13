import os

### Write the human trajectories to the agents.yaml file in hunavsim
HUNAV_SIM_DIR = '/home/shashank/catkin_ws/src/hunav_sim/hunav_agent_manager'
HUNAV_SIM_AGENTS_FILE = os.path.join(HUNAV_SIM_DIR,'config','custom_agents.yaml')
HUNAV_SIM_BT_FOLDER = os.path.join(HUNAV_SIM_DIR,'behavior_trees')
HUNAV_SIM_CPP_FOLDER = os.path.join(HUNAV_SIM_DIR,'src')
HUNAV_SIM_HPP_FOLDER = os.path.join(HUNAV_SIM_DIR,'include','hunav_agent_manager')
HUNAV_GAZEBO_WRAPPER_DIR = '/home/shashank/catkin_ws/src/hunav_gazebo_wrapper'

SAVE_DIR = 'responses/ablation/scene/scene3'

## GPT 
MODEL="gpt-4o"# "gemini-1.0-pro-vision"


## GEMINI
VERTEXAI_PROJECT_ID = None
VERTEXAI_REGION = None
VERTEXAI_API_KEY = None

LOCATION = 'Warehouse_fixed'
LOCATION_DESC = 'The Location is a Small Warehouse consists of various racks and open packaging areas connected by Passageways. The Passageways also create Intersections.'
# CONTEXT = """Robot is a delivery bot in a small warehouse. There are employees performing daily duties walking around the warehouse."""
# TASK = """Robot has to transport boxes in the warehouse"""
# CONTEXT = """Robot is an emergency response robot inside a warehouse in a disaster situation."""
# TASK = """The task of the robot is to guide humans to safety."""
CONTEXT = """Robot is an maintenance robot inside a warehouse."""
TASK = """The task of the robot is to clean the warehouse."""
#ROUGH_SCENARIO = """Robot start inside a narrow aisle(NODE 79). There is human standing at the opposite end of the aisle, the human stand its position until the robot isn not blocking path, once the robot is not blocking the human, the human starts moving towards its goal in the aisle"""


USE_HANDCRAFTED_SCENARIO = False
# Handcrafted Scenario

scenario_desc_hc = """The robot is at Aisle7 (7) and has to go to Aisle3 (3). At the same time, a human is moving from Aisle6 (6) to Aisle3 (3) through Hall (5) and another human is moving from Aisle6 (6) to Aisle2 (2) through Hall (5)."""
num_humans_hc =  2
traj_desc_hc = {
    'Robot':['Aisle7 (7)','Aisle3 (3)'],
    'Human 1':['Aisle6 (6)','Hall (5)','Aisle3 (3)'],
    'Human 2':['Aisle6 (6)','Hall (5)','Aisle2 (2)']
}
behav_desc_hc = {
    'Human 1':"""The human is partially visually impaired and can only see the robot when it is less than 1m close to him. He thus walks slowly. When he sees the robot he is startled and he stops for 3 seconds before continuing on.""",
    'Human 2':"""The human is curious about the robot and walks slowly towards the robot when he sees it, then gets bored after 5 seconds and continues on."""
}


QUERY_SC = True
QUERY_TRAJ = True
QUERY_BT = True
QUERY_AUX = True

SAVE_SC_RESPONSE = True
SAVE_TRAJ_RESPONSE = True
SAVE_BT_RESPONSE = True
SAVE_AUX_RESPONSE = True

LOAD_SC_RESPONSE = True
LOAD_TRAJ_RESPONSE = True
LOAD_BT_RESPONSE = True
LOAD_AUX_RESPONSE = True

import os
### Write the human trajectories to the agents.yaml file in hunavsim
HUNAV_SIM_DIR = '/home/shashank/catkin_ws/src/hunav_sim/hunav_agent_manager'
HUNAV_SIM_AGENTS_FILE = os.path.join(HUNAV_SIM_DIR,'config','custom_agents.yaml')
HUNAV_SIM_BT_FOLDER = os.path.join(HUNAV_SIM_DIR,'behavior_trees')
HUNAV_SIM_CPP_FOLDER = os.path.join(HUNAV_SIM_DIR,'src')
HUNAV_SIM_HPP_FOLDER = os.path.join(HUNAV_SIM_DIR,'include','hunav_agent_manager')
HUNAV_GAZEBO_WRAPPER_DIR = '/home/shashank/catkin_ws/src/hunav_gazebo_wrapper'

## GPT 
MODEL="gpt-4-1106-preview"# "gemini-1.0-pro-vision"


## GEMINI
VERTEXAI_PROJECT_ID = None
VERTEXAI_REGION = None
VERTEXAI_API_KEY = None

LOCATION = 'Warehouse'
CONTEXT = """Robot is a delivery bot in a small warehouse. There are employees performing daily duties walking around the warehouse."""
TASK = """Guide a human to a rack for inventory check."""

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


QUERY_SC = False
QUERY_LOC = False
QUERY_BT = True
QUERY_AUX = True

SAVE_SC_RESPONSE = True
SAVE_LOC_RESPONSE = True
SAVE_BT_RESPONSE = True
SAVE_AUX_RESPONSE = True

LOAD_SC_RESPONSE = True
LOAD_LOC_RESPONSE = True
LOAD_BT_RESPONSE = True
LOAD_AUX_RESPONSE = True
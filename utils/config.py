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
NUM_SCENARIO_PROPOSALS = 5

LOCATION = 'Warehouse_fixed'
LOCATION_DESC = 'Small Warehouse that consists of various racks and packaging areas. The warehouse consists of passageways (narrow and wide), intersections, narrow aisles, corners, open areas.'
#LOCATION_DESC = 'The location is the 4th floor of an office building with various research labs and meeting rooms connected by doorways and passageways. The passageways create intersections and blind corners.'
CONTEXT = """Robot is a delivery bot in a small warehouse in MUMBAI,INDIA. There are employees performing daily duties walking around the warehouse. The warehouse follows a policy of silence."""
TASK = """Robot has to transport boxes in the warehouse"""
# CONTEXT = """Robot is an emergency response robot inside a warehouse in a disaster situation."""
# TASK = """The task of the robot is to guide humans to safety."""
#CONTEXT = """The robot operates in a research institute office building. The office building generally has employees and service workers in the building walking around. The floor is generally quiet. People generally gather in the coffee room to meet informally."""
#TASK = """Deliver coffee to a location and return to the start point"""
#ROUGH_SCENARIO = """Robot start inside a narrow aisle(NODE 79). There is human standing at the opposite end of the aisle, the human stand its position until the robot isn not blocking path, once the robot is not blocking the human, the human starts moving towards its goal in the aisle"""
#ROUGH_SCENARIO = "The robot encounters the same pedestrian first walking in a direction opposite to the robot in a narrow passageway, then at an intersection and then at a blind corner. The pedestrian is walking in a closed circuit path."
ROUGH_SCENARIO = None
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

node_library = {
 'root':['main_tree_to_execute'],
 'BehaviorTree':['ID'],
 'include':['path'],
 'SetBlackboard':['output_key','value'],
 'Sequence':['name'],
 'Fallback':['name'],
 'SubTree':['ID'],
 'Inverter':[],
 'IsRobotVisible':['agent_id','distance'],
 'IsRobotNearby':['agent_id','distance'],
 'IsGoalReached':['agent_id'],
 'TimeExpiredCondition':['seconds','ts','only_once'],
 'RobotSays':['agent_id','message'],
 'RobotMoved':['agent_id'],
 'IsRobotBlocking':['agent_id','distance'],
 'UpdateGoal':['agent_id'],
 'MakeGesture':['agent_id','message'],
 'RegularNav':['agent_id','time_step'],
 'LookAtRobot':['agent_id'],
 'FollowRobot':['agent_id','time_step'],
 'AvoidRobot':['agent_id','time_step'],
 'GiveWaytoRobot':['agent_id','time_step'],
 'BlockRobot  ':['agent_id','time_step'],
}

QUERY_SC = True
QUERY_TRAJ = True
QUERY_BT = True
QUERY_AUX = True

SAVE_SC_RESPONSE = True
SAVE_TRAJ_RESPONSE = True
SAVE_BT_RESPONSE = True
SAVE_AUX_RESPONSE = True

LOAD_SC = not QUERY_SC
LOAD_TRAJ = not QUERY_TRAJ
LOAD_BT = not QUERY_BT
LOAD_AUX = not QUERY_AUX

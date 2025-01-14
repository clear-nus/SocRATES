from .BasePrompt import BasePrompt
class AbalationQuery(BasePrompt): 
   def __init__(self) -> None:
        super().__init__()
        self.required_output_keys = ['scenariodescription','numberofhumans','humanbehavior','expectedrobotbehavior','tree','reasoning','trajectories','groupids']
        self.payload = [
            {"role": "system", 
     "content": [{
         "type":"text",
         "text":"""
        You are an expert scenario planner. You ALWAYS provide output that in JSON which is fully parseable with json.loads in python.
         """}]
     },
            
            {"role": "user", 
            "content": [{
                "type":"text",
                "text":"""Social Navigation is a complex skill for a robot to accomplish and the appropriateness of the behavior of a robot is highly dependent on the task and the social context. Thus a robot’s social navigation capabilities must be thoroughly tested, and this is done by evaluating the robot’s behavior in a number of scenarios in a variety of contexts. 
You are a scenario designer. Your task is to generate scenarios, then come up with trajectories in the given location to ochestrate the generated scenarios and finally come up with a behaviour tree according to the human behavior description generated.
A Social Navigation [Scenario] is defined by:
    1. Scenario Description: very detailed description of the scenario. WHAT happens in the scenario and WHERE the scenario takes place. WHERE the robot and humans are located.
    2. Human Behavior:  how human interacts with the robot when it is visible, for e.g. Human 1 is scared of the robot and asks it to stop, Human 2 doesn't notice the robot at all etc.

The humans are only capable of performing the actions mentioned above.
User will provide a [Social context], a [Task] that the robot needs to do, a description of the location and optionally a [Rough Scenario]. 
Your generated scenario will be programmatically simulated through a pipeline into a scenario in the Gazebo physics simulator.
Rules:
- The humans can say "WAIT", "PROCEED", "EXCUSE ME" to the robot  to aid in navigation. The robot can say "WAIT", "PROCEED", "EXCUSE ME", "ACKNOWLEDGED" to the humans to aid in navigation. 
- When the user provides a Rough Scenario, ensure your final scenario is strictly aligned to the rough scenario
- The humans in the simulator are SIMPLIFIED OBJECTS that only can move in 2D, send and receive simple phrases, detect and simulate looking at the robot, group together with other humans, navigate to a predefined goal and change their trajectory conditioned on the robot’s position and velocity.
- When using groups in the scenario, add all group members to the humans in the scenario. Having only 1 human with 'INTERACTING WITH GROUP' task is incorrect.

You have also been provided an image and a scene graph. The image shows a location which is represented by a scene graph. A scene graph is a graph with nodes (numbered red circles) representing locations and edges (blue lines) connecting them. 
A person/ can only move from one node to another if the two nodes are connected by an edge. 
The scene graph in json file format is also given below, where each node and edge has a unique id, a type, the pixel position of the node in the image (pos). 
<SCENE GRAPH>
The type of a node is one among <NODE TYPES>. 
The graph is bidirectional and each edge is also represented with the edge list 'links' in the json given below.  
Remember that a node can only be reached from another node if they have an edge between them in the scene graph json.

You must select the trajectories of the humans and the robot to orchestrate the generated scenario. You must also assign integer group ids to the HUMANS ONLY involved in the scenario according to the 
following rule:
1. All the members of the same group must have the same group id.
2. If a human is not involved in a group, they will have a group id of -1.
3. If the scenario does not specifically mention a group, then assign -1 to all humans
4. All humans involved in the scenario must be assigned a group id.

You also need to design a Behaviour tree using the BehaviorTree.CPP library in XML according to the human behavior previously generated by you, using only the existing actions and conditions. 
The tutorial below will explain how to design behavior trees in XML:
    - The first tag of the tree is <root> with the attribute 'main_tree_to_execute'. It should contain 1 or more tags <BehaviorTree> and the The tag <BehaviorTree> should have the attribute [ID]. E.g:
        <root main_tree_to_execute = 'SaySomething'>
        ....
        <BehaviorTree ID="SaySomething>
        ...
        </BehaviorTree> 
        </root>
    - The <BehaviorTree> tag can contain only 1 child node (including Fallback, Sequence, Action and Condition Nodes)
    - Sequence and Fallback nodes contain 1 to N children and dictate control flow in the behavior tree.
        - Sequence nodes execute their children in order and return Success only if ALL CHILD NODES returned SUCCESS. (like an AND gate).
            - Each sequence node typically has multiple child nodes
        - Fallback nodes execute their children in order and return Success if ANY CHILD NODE SUCCEEDS(like an OR gate). 

    - Each Sequence and Fallback TreeNodes are represented by a single tag with an associated name. E.g.:
        <Fallback name="SaySomething 1">
            <Sequence name = "SaySomething 2">
                ....
            </Sequence>
        </Fallback>
    JSONe">
            <Sequence name="RegNav">
                <SetBlackboard output_key="agentid" value="{id}" /> <!--Main tree creates a blackboard entry 'agentid' with the value retrieved from the blackboard entry "id" -->
                <SetBlackboard output_key="timestep" value="{dt}" /> <!--Main tree creates a blackboard entry 'timestep' with the value retrieved from the blackboard entry "id" -->
                <SubTree ID="RegularNavTree" id="agentid" dt="timestep" /> <!-- The "id" and "dt" ports of the subtree are mapped to the agentid and timestep ports of the main tree blackboard --> 
            </Sequence>             
        </BehaviorTree>

The following Action Nodes, Condition nodes and Decorators are available to use and can be composed into behavior trees to achieve the user's request. 

    - BT Conditions
        - IsRobotVisible(agent_id ,distance) : returns Success when the robot is within the input distance of the agent and visible(in line of sight) to the agent. Distance is in meters. 
        - IsRobotNearby(agent_id, distance) : returns Success when the robot is within the input distance of the agent. Distance is in meters.
        - IsGoalReached(agent_id) : returns Success if the agent has reached their current goal
        - TimeExpiredCondition(seconds, ts, only_once): Creates a timer and returns SUCCESS if the input duration in seconds has expired since ticking this node the first time (and FAILURE otherwise). If only_once = False, then the timer repeats (use this for periodically occuring behaviors).
        - RobotSays(agent_id, message) : Returns Success if the robot is currently performing a gesture corresponding to the message(int) passed to the functions. Messages correspond to gestures as: [0 (No gesture), 1("WAIT"), 2("PROCEED"),3("ACKNOWLEDGED"),4("EXCUSE ME")].
        - RobotMoved(agent_id): Returns success if the robot has non zero velocity.
        - IsRobotBlocking(agent_id, distance): Returns Success if the robot is in direct line of sight of the agent and within the input distance. Distance is in meters.
    - Note: In the simulator, for distance, 0.5 is considered very close, 1.0 is considered very closed, 2.0 is considered moderate distance and 5.0 is considered far
    - BT Actions:
        - UpdateGoal(agent_id) : Updates the goal of the agent to the next goal in the agent’s goal queue
        - MakeGesture(agent_id,message): Makes the agent perform a gesture. Choices are: [0 (No gesture), 1("WAIT"), 2("PROCEED"), 3("EXCUSE ME")]. Initial value is 0 and once this node is ticked, the agent will keep making the gesture until it is set back to 0. 
        - RegularNav(agent_id,time_step) : Makes the agent perform standard social-force-model based motion planning, where the robot is treated as a normal obstacle.
        - LookAtRobot(agent_id) : Makes the agent look in the direction of the robot
        - FollowRobot(agent_id,time_step): Makes the agent follow the robot
        - AvoidRobot(agent_id,time_step): Makes the agent overly avoid the robot.
        - GiveWaytoRobot(agent_id,time_step): Makes the agent give way to the robot.
        - BlockRobot(agent_id,time_step): Makes the agent move in front of the robot and block it
    
    - BT Decorators:
        - Inverter: An inverter block inverts the output of its children (If children output failure, inverter outputs success and vice-versa).

The following behavior tree is available for including as a subtree, which implements a simple obstacle-avoiding human:
    - BTRegularNav.xml:         
    <root main_tree_to_execute = "RegularNavTree">
        <BehaviorTree ID="RegularNavTree">
            <Fallback name="RegularNavFallback">
                <Sequence name="RegularNavigation">
                    <Inverter>
                        <IsGoalReached agent_id="{id}" />
                    </Inverter>
                    <RegularNav agent_id="{id}" time_step="{dt}" />
                </Sequence>
                <UpdateGoal agent_id="{id}" />
            </Fallback>
        </BehaviorTree>
    </root>

Design a scenario relevant to the following specifications:

[LOCATION_DESCRIPTION]: "The location consists of passageways, intersections, and open areas, organized to facilitate efficient navigation and connectivity. Passageways form the primary paths that connect various parts of the facility, ensuring straightforward routes between specific nodes like corners and intersections. Intersections act as critical nodes where multiple passageways converge, enabling choices in directional movement. Open areas provide larger, unobstructed spaces that could serve as gathering places or functional zones. The entire layout is linked by direct connections between these nodes, forming a grid-like structure that aids in understanding the spatial organization and potential routes for movement within the environment."
[CONTEXT]: "Robot is an emergency response robot inside a warehouse in a disaster situation."
[TASK]: "The task of the robot is to guide humans to the safest exit"
[ROUGH_SCENARIO]: "The robot is guiding employees to a safe exit due to an emergency. The robot starts from the east Passageway heading towards an emergency exit located in the west Passageway. Two employees are present in the warehouse. The robot needs to guide both employees to the exit while avoiding obstacles "
        
YOU ADHERE TO THE FOLLOWING JSON FORMAT STRICTLY. 

'Scenario Description': <very detailed description of the scenario >,
'Number of Humans': <Number of humans that are involved in the scenario>,
‘Human behavior': 
    ‘Human 1’: <Describe the behavior of Human 1>,
    ‘Human 2’: <Describe the behavior of Human 2>,
'Expected Robot Behavior': <Describe the behavior expected from the robot>
'TRAJECTORIES': 
    'ROBOT': <...>
    'HUMAN 1': <...>,
    'HUMAN 2': <...>,
    'HUMAN 3': <...>,
'GROUP IDS':
    'HUMAN 1': ..,
    'HUMAN 2': ...,
    ...
'TREE': <XML Behavior Tree ONLY>
    'HUMAN 1':  ..,
    'HUMAN 2':  ...,
'REASONING': <Reasoning behind answer for the question>
                    """},
                    {         
                        "type":"image_url",
                        "image_url":{
                            "url":"data:image/jpeg;base64,{<ENCODED IMAGE>}",
                            "detail":"high"
                        }
                    }]},
            {"role": "assistant", 
                    "content": [
                        {
                            "type":"text",
                            "text":"""
                            'Scenario Description':"The robot is guiding employees to a safe exit due to an emergency. The scene starts in the east section of the Small Warehouse near the open packaging area. The robot starts from the east Passageway heading towards an emergency exit located in the west Passageway. Two employees are present, one near the north intersection of the Passageways and the other by a rack in the center of the warehouse. The robot needs to guide both employees to the exit while avoiding obstacles.",
                            'Number of Humans': 2,
                            'Human Behavior': 
                                "Human 1": "Human 1 sees the robot approaching and follows it immediately upon hearing 'PROCEED'."}, 
                                "Human 2": "Human 2 initially checks the visibility of the robot, then continues normal activities. When the robot approaches and says 'PROCEED', Human 2 follows the robot."}}, "expectedrobotbehavior": "The robot navigates through the Passageways, avoiding obstacles while saying 'PROCEED' to each human it encounters. The robot ensures that both humans are following by occasionally checking back before moving towards the exit. It waits for all humans to be in a safe zone before proceeding further."},
                            'Expected Robot Behavior':"The robot navigates through the Passageways, avoiding obstacles while saying 'PROCEED' to each human it encounters. The robot ensures that both humans are following by occasionally checking back before moving towards the exit. It waits for all humans to be in a safe zone before proceeding further.",
                            'Trajectories': 
                                "ROBOT": ["79", "99", "cf", "ba", "cb", "0e"], 
                                "HUMAN 1": ["99", "cf", "ba", "cb", "0e"], 
                                "HUMAN 2": ["cf", "ba", "cb", "0e"]}, 
                            "GROUP ID": 
                            "HUMAN 1": 1, 
                            "HUMAN 2": 1,
                            'TREE':
                            "HUMAN 1": "<root main_tree_to_execute='HumanFollowRobotTree'>\n    <BehaviorTree ID='HumanFollowRobotTree'>\n        <Sequence name='FollowRobotSequence'>\n            <IsRobotVisible agent_id='{id}' distance='4.0'/>\n            <RobotSays agent_id='{id}' message='2'/>\n            <FollowRobot agent_id='{id}' time_step='{dt}'/>\n        </Sequence>\n    </BehaviorTree>\n</root>"
                            "HUMAN 2": "<root main_tree_to_execute='Human2BehaviorTree'>\n    <include path='BTRegularNav.xml'/>\n    <BehaviorTree ID='Human2BehaviorTree'>\n        <Fallback name='MainFallback'>\n            <Sequence name='RobotVisibleAndProceed'>\n                <IsRobotVisible agent_id='{id}' distance='4.0'/>\n                <RobotSays agent_id='{id}' message='2'/>\n                <FollowRobot agent_id='{id}' time_step='{dt}'/>\n            </Sequence>\n            <Sequence name='NormalActivities'>\n                <SetBlackboard output_key='agentid' value='{id}'/>\n                <SetBlackboard output_key='timestep' value='{dt}'/>\n                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>\n            </Sequence>\n        </Fallback>\n    </BehaviorTree>\n</root>"
                            'REASONING':"To orchestrate the scenario where the robot is guiding employees to a safe exit due to an emergency, we need to plan the robot's trajectory from the east passageway towards the west passageway, while collecting the employees along the way. The robot will start in the east section of the warehouse near the open packaging area and travel towards the west passageway where the emergency exit is located. The employees are positioned as follows: Human 1 is near the north intersection of the passageways, and Human 2 is by a rack in the center of the warehouse. All involved humans are part of the same group as they are being guided together by the robot. 
                            For Human 1 The behavior requires to start following the robot upon hearing 'PROCEED' when the robot is visible. We will use the IsRobotVisible condition node to check if the robot is visible and the RobotSays condition node to check if the robot says 'PROCEED'. If both conditions are met, the human will execute the FollowRobot action. If not, the tree will fallback to doing nothing. To implement the described behavior for Human 2, we need a behavior tree where Human 2 first checks the visibility of the robot and then proceeds to its normal activities. When the robot approaches and says 'PROCEED', Human 2 will start following the robot. The logic can be broken down into the following steps: check if the robot is visible, if the robot is visible and says 'PROCEED', follow the robot. Otherwise, continue with regular navigation activities."
                            """
                        }
                    ]
                },
              {"role": "user",
                "content": [
                    {
                    "type":"text",
                    "text":"""
                    Now design a scenario for the following case:
                    [LOCATION_DESCRIPTION]: "The location consists of passageways, intersections, and open areas, organized to facilitate efficient navigation and connectivity. Passageways form the primary paths that connect various parts of the facility, ensuring straightforward routes between specific nodes like corners and intersections. Intersections act as critical nodes where multiple passageways converge, enabling choices in directional movement. Open areas provide larger, unobstructed spaces that could serve as gathering places or functional zones. The entire layout is linked by direct connections between these nodes, forming a grid-like structure that aids in understanding the spatial organization and potential routes for movement within the environment."
                    [CONTEXT]: "Robot is a delivery bot in a small warehouse. There are employees performing daily duties walking around the warehouse."
                    [TASK]: "The robot has to transport a box from one point to another in the warehouse."
                    [ROUGH_SCENARIO]: "Robot start inside a narrow aisle(NODE 79). There is human standing at the opposite end of the aisle, the human stand its position until the robot isn not blocking path, once the robot is not blocking the human, the human starts moving towards its goal in the aisle"
                    """
                    
                }]}
                ]
   
   def get_full_prompt(self,**kwargs):
    full_prompt = self.payload
    #add json and image to first user prompt
    full_prompt[1]["content"][0]["text"] = full_prompt[1]["content"][0]["text"].replace('<SCENE GRAPH>',kwargs['scene_graph'])
    full_prompt[1]["content"][0]["text"] = full_prompt[1]["content"][0]["text"].replace('<NODE TYPES>',kwargs['node_types'])
    full_prompt[1]["content"][1]["image_url"]["url"] = full_prompt[1]["content"][1]["image_url"]["url"].replace('<ENCODED IMAGE>',kwargs['encoded_img'])
    return full_prompt
from pydantic import BaseModel
class StructuredBTResponse(BaseModel):
    reasoning: str
    tree: str
    
from .BasePrompt import BasePrompt
class BTQuery(BasePrompt):
    def __init__(self) -> None:
        super().__init__()
        self.payload = [
        {"role": "system", "content": [{
         "type":"text",
         "text":f"""
        Act as an expert Behavior Tree Designer for a social navigation robotics simulator and an expert software engineer. You ALWAYS provide output that in JSON which is fully parseable with json.loads python.
        You are diligent and tireless!
        You NEVER leave comments describing code without implementing it!
        You always COMPLETELY IMPLEMENT the needed code and do not leave placeholders or assume anything!
         """}]
        },         
        {"role": "user", 
     "content": [{
         "type":"text",
         "text":"""
Your job is to design a Behavior Tree using the BehaviorTree.CPP library in XML according to the [BEHAVIOR] described by the user, using only the existing actions and conditions. 
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
        - IsRobotNearby(agent_id, distance) : returns Success when the robot is near the agent within the input distance. Distance is in meters.
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
Now answer the following question:

If i want a human to gesture "WAIT" what should be the corresponding xml code

Return output in the following json format:
{   
    'REASONING': <Reasoning behind answer>
    'ANSWER': <XML CODE>
    
}"""}]},
        {
            "role": "assistant", 
            "content": [
                {
                    "type":"text",
                    "text":"""
                    {
    'REASONING': "To create a behavior tree where a human gestures "WAIT," you need to use the `MakeGesture` action node with the message corresponding to the "WAIT" gesture. The message code for "WAIT" is 1. 
In this behavior tree:
- `<root main_tree_to_execute="WaitGestureTree">` specifies the main tree to execute.
- `<BehaviorTree ID="WaitGestureTree">` defines a behavior tree with the ID "WaitGestureTree."
- `<Sequence name="WaitGestureSequence">` creates a sequence node named "WaitGestureSequence."
- `<MakeGesture agent_id="{id}" message="1"/>` makes the agent perform the "WAIT" gesture (message code 1).
"
    'ANSWER': "<root main_tree_to_execute="WaitGestureTree">
    <BehaviorTree ID="WaitGestureTree">
        <Sequence name="WaitGestureSequence">
            <MakeGesture agent_id="{id}" message="1"/>
        </Sequence>
    </BehaviorTree>
</root>
"
}"""
                }
            ]
        },
        {
            "role":"user",
            "content":[
                {
                    "type":"text",
                    "text":"""If I want the human to wait for 50 secs and then check if the the robot is visible what should be the corresponding xml code?
Return output in the following json format:
{   
    'REASONING': <Reasoning behind answer>
    'ANSWER': <XML CODE>
    
}"""
                }
            ]
        },
        {
            "role":"assistant",
            "content":[
                {
                    "type":"text",
                    "text":"""
                    {
                    'REASONING':"To create a behavior tree where a human waits for 50 seconds and then checks if the robot is visible, you can use the `TimeExpiredCondition` to create the waiting period and the `IsRobotVisible` condition to check the robot's visibility. 
In this behavior tree:
- `<root main_tree_to_execute="WaitAndCheckVisibilityTree">` specifies the main tree to execute.
- `<BehaviorTree ID="WaitAndCheckVisibilityTree">` defines a behavior tree with the ID "WaitAndCheckVisibilityTree."
- `<Sequence name="WaitAndCheckSequence">` creates a sequence node named "WaitAndCheckSequence."
- `<TimeExpiredCondition seconds="50" ts="{dt}" only_once="true" />` creates a condition to wait for 50 seconds.
- `<IsRobotVisible agent_id="{id}" distance="some_distance_value"/>` checks if the robot is visible within a specified distance. Replace `some_distance_value` with the appropriate distance value based on your requirements.
This sequence ensures that the agent waits for 50 seconds before checking if the robot is visible within the specified distance.
"
                    'ANSWER':"
                    <root main_tree_to_execute="WaitAndCheckVisibilityTree">
    <BehaviorTree ID="WaitAndCheckVisibilityTree">
        <Sequence name="WaitAndCheckSequence">
            <TimeExpiredCondition seconds="50" ts="{dt}" only_once="true" />
            <IsRobotVisible agent_id="{id}" distance="some_distance_value"/>
        </Sequence>
    </BehaviorTree>
</root>"}"""
                }
            ]
        },
        {
            "role":"user",
            "content":[
                {
                    "type":"text",
                    "text":"""if i want to the human to check if the robot is saying "PROCEED" what should be the corresponding xml code?
Return output in the following json format:
{   
    'REASONING': <Reasoning behind answer>
    'ANSWER': <XML CODE>
    
}"""
                }
            ]
        },
        {
            "role":"assistant",
            "content":[
                {
                    "type":"text",
                    "text":"""{
                        
                        'REASONING':To create a behavior tree where a human checks if the robot is saying "PROCEED," you need to use the `RobotSays` condition node with the message corresponding to the "PROCEED" gesture. 
                        The message code for "PROCEED" is 2.
In th behavior tree:
- `<root main_tree_to_execute="CheckProceedGestureTree">` specifies the main tree to execute.
- `<BehaviorTree ID="CheckProceedGestureTree">` defines a behavior tree with the ID "CheckProceedGestureTree."
- `<Sequence name="CheckProceedGestureSequence">` creates a sequence node named "CheckProceedGestureSequence."
- `<RobotSays agent_id="{id}" message="2"/>` checks if the robot is saying "PROCEED" (message code 2).

                        'ANSWER':
"<root main_tree_to_execute="CheckProceedGestureTree">
    <BehaviorTree ID="CheckProceedGestureTree">
        <Sequence name="CheckProceedGestureSequence">
            <RobotSays agent_id="{id}" message="2"/>
        </Sequence>
    </BehaviorTree>
</root>"           }"""   
                }
            ]
        },
        {
            "role":"user",
            "content":[
                {
                    "type":"text",
                    "text":"""Given below is Behavior Tree for a human:
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='WaitForProceedTree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='WaitForProceedTree'>
        <Fallback name='VisibilityFallback'>
            <Sequence name='WaitForProceed'>
                <IsRobotVisible agent_id='{id}' distance='4.0'/>
                <RobotSays agent_id='{id}' message='2'/>
                <MakeGesture agent_id='{id}' message='0'/>
            </Sequence>
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>
The corresponing behavior for the above behavior tree is:
When the robot is visible, the human will wait for the robot to say 'PROCEED' and only then continue towards their goal normally, treating the robot as a normal obstacle afterward. If the robot is not visible, the human will continue navigating normally.
There are some errors in the above XML code to model the above behavior. List the errors and correct them.
Return output in the following json format:

{
        'REASONING': <Reasoning behind answer>
        'ANSWER': <LIST OF CORRECTIONS>
}"""
                }
            ]
        },
        {
            "role":"assistant",
            "content":[
                {
                    "type":"text",
                    "text":"""{
        'REASONING': "The provided behavior tree describes a human agent's behavior in a simulated environment. Here's a detailed explanation of the behavior corresponding to the tree:
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='WaitForProceedTree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='WaitForProceedTree'>
        <Fallback name='VisibilityFallback'>
            <Sequence name='WaitForProceed'>
                <IsRobotVisible agent_id='{id}' distance='4.0'/>
                <RobotSays agent_id='{id}' message='2'/>
                <MakeGesture agent_id='{id}' message='0'/>
            </Sequence>
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>

### Explanation
#### Main Tree Execution
The main tree to execute is specified as `WaitForProceedTree`.
#### Included File
The tree includes another behavior tree defined in `BTRegularNav.xml`. This file likely contains the behavior for regular navigation using the `RegularNavTree` ID.
#### Fallback Node (`VisibilityFallback`)
The root node of the `WaitForProceedTree` is a fallback node named `VisibilityFallback`. A fallback node executes its children from left to right and returns `SUCCESS` if any child node returns `SUCCESS`. If all children return `FAILURE`, then the fallback node returns `FAILURE`.
#### First Sequence Node (`WaitForProceed`)
The first child of the fallback node is a sequence node named `WaitForProceed`. A sequence node executes its children from left to right and returns `FAILURE` if any child node returns `FAILURE`. If all children return `SUCCESS`, then the sequence node returns `SUCCESS`.
1. **`IsRobotVisible` Condition**
    - Checks if the robot is visible within a distance of 4.0 units from the human agent.
    - If the robot is not visible, this condition returns `FAILURE`, causing the sequence to fail and the fallback to proceed to the next child.
2. **`RobotSays` Condition**
    - Checks if the robot is saying "PROCEED" (message code 2).
    - If the robot is not saying "PROCEED", this condition returns `FAILURE`, causing the sequence to fail and the fallback to proceed to the next child.
3. **`MakeGesture` Action**
    - Makes the human agent stop any gesture (message code 0).
    - If this action is successful, the sequence returns `SUCCESS`, causing the fallback node to return `SUCCESS` as well.
#### Second Sequence Node (`RegNav`)
If the first sequence (`WaitForProceed`) fails, the fallback node proceeds to execute the second sequence node named `RegNav`.
1. **`SetBlackboard` Actions**
    - Sets the blackboard key `agentid` to the value of `{id}`.
    - Sets the blackboard key `timestep` to the value of `{dt}`.
2. **Subtree (`RegularNavTree`)**
    - Executes the `RegularNavTree` subtree using the `agentid` and `timestep` values set in the blackboard.
    - This subtree contains the regular navigation behavior for the human agent, likely involving obstacle avoidance and standard motion planning.
### Summary
The human agent's behavior can be summarized as follows:
1. The agent first checks if the robot is visible within a 4.0 unit distance.
2. If the robot is visible, the agent then checks if the robot is saying "PROCEED."
3. If the robot is saying "PROCEED," the agent stops any gestures.
4. If either the robot is not visible or the robot is not saying "PROCEED," the agent proceeds with its regular navigation behavior defined in `BTRegularNav.xml`."
        'ANSWER': "### Errors in the Behavior Tree corresponding to the Given Behavior
1. The above behavior tree doesn't do RegularNav if the RobotSays "PROCEED" condition is satisfied and instead just calls the MakeGesture action.
### Corrected Tree
Below is the corrected behavior tree to implement the given behavior exactly.
<?xml version='1.0' encoding='utf-8'?>
<root main_tree_to_execute='WaitForProceedTree'>
    <include path='BTRegularNav.xml'/>
    <BehaviorTree ID='WaitForProceedTree'>
        <Fallback name='VisibilityFallback'>
            <Sequence name='WaitForProceed'>
                <IsRobotVisible agent_id='{id}' distance='4.0'/>
                <RobotSays agent_id='{id}' message='2'/>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
            <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}' />
                <SetBlackboard output_key='timestep' value='{dt}' />
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>"
}"""
                }
            ]
        }
        
        ]
        
        bt_example_1 = [{
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""Now, create a behavior tree for the following behavior:

Human rushes towards the robot if the robot is visible and continuously blocks the robot for 40s and asks it to WAIT.

YOU MUST NOT USE ANY CUSTOM ACTION/CONDITION NODES AND ONLY USE THE NODES AVAILABLE.
Return an answer in JSON format shown below:"""+"""
{   
    'REASONING': <reasoning behind tree design>,
    'TREE': <XML Behavior Tree ONLY>,
}
"""
            }]
        },{
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{
    'REASONING': "The behavior can be achieved by running the BlockRobot node for 40s if the robot is visible with MakeGesture and falling back to regularNav when the robot is not visible.",
    'TREE':""
    <root main_tree_to_execute = "BullyHumanTree">
        <include path="BTRegularNav.xml"/>        
        <BehaviorTree ID="BullyHumanTree">
            <Fallback name="VisibilityFallback">
                <Sequence name="RobotVisibleSequence">
                    <IsRobotVisible agent_id="{id}" distance="4.0" />
                    <MakeGesture agent_id="{id}" message="1"/>
                    <Inverter>
                        <TimeExpiredCondition seconds="40.0" ts="{dt}" only_once="True" />
                    </Inverter>
                    <BlockRobot agent_id="{id}" time_step="{dt}" />
                </Sequence>
                <Sequence name="RegNav">
                    <MakeGesture agent_id="{id}" message="0"/>
                    <SetBlackboard output_key="agentid" value="{id}" />
                    <SetBlackboard output_key="timestep" value="{dt}" />
                    <SubTree ID="RegularNavTree" id="agentid" dt="timestep" /> <!-- Using Subtree -->
                </Sequence>
            </Fallback>
        </BehaviorTree>
    </root>"",
} """
            }]
        }]
        
        bt_example_2 = [{
            "role":"user",
            "content":[{
                "type":"text",
                "text":""" Now, create a behavior tree for the following behavior:
If the robot is visible at the intersection and its moving, the human will wait for the robot to say 'PROCEED' and then continue towards their goal normally, treating the robot as a normal obstacle afterward.

YOU MUST NOT USE ANY CUSTOM ACTION/CONDITION NODES AND ONLY USE THE NODES AVAILABLE.
Return an answer in JSON format shown below:"""+"""
{   
    'REASONING': <reasoning behind tree design>,
    'TREE': <XML Behavior Tree ONLY>,
}
"""
            }]
        },{
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{
                    
                    'REASONING':"he human waits for the robot to say 'PROCEED' (gesture code 2) "
              'when visible at the intersection and when it is moving, and then continues towards '
              'their goal using RegularNav."
                    'TREE':
         "<root main_tree_to_execute = "HumanAtIntersectionTree">
             <include path="BTRegularNav.xml"/>
             <BehaviorTree ID="HumanAtIntersectionTree">
                 <Fallback name="VisibilityAndGestureFallback">
                     <Sequence name="WaitForProceedSequence">
                         <IsRobotVisible agent_id="{id}" distance="4.0" />
                         <RobotMoved agent_id='{id}'/>
                         <RobotSays agent_id="{id}" message="2" />
                         <MakeGesture agent_id="{id}" message="0" />
                         <SetBlackboard output_key="agentid" value="{id}" />
                         <SetBlackboard output_key="timestep" value="{dt}"/>
                         <SubTree ID="RegularNavTree" id="agentid",dt="timestep" /> <!-- Using Subtree -->
                     </Sequence>
                     <Sequence name="RegNav">
                         <MakeGesture agent_id="{id}" message="0"/>
                         <SetBlackboard output_key="agentid" value="{id}"/>
                         <SetBlackboard output_key="timestep" value="{dt}"/>
                         <SubTree ID="RegularNavTree" id="agentid", dt="timestep" /> <!-- Using Subtree -->
                     </Sequence>
                 </Fallback>
             </BehaviorTree>
         </root>"
            } """
            }]
        }]
        
        
        bt_example_3 = [{
            "role":"user",
            "content":[{
                "type":"text",
                "text":""" Now, create a behavior tree for the following behavior:
If the robot is visible at the intersection, the human will wait for the robot to say 'PROCEED' and then continue towards their goal normally, treating the robot as a normal obstacle afterward.
YOU MUST NOT USE ANY CUSTOM ACTION/CONDITION NODES AND ONLY USE THE NODES AVAILABLE.
Return an answer in JSON format shown below:"""+"""
{   
    'REASONING': <reasoning behind tree design>,
    'TREE': <XML Behavior Tree ONLY>,
}
"""
            }]
        },{
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{{
        'REASONING': "The human waits for the robot to say 'PROCEED' (gesture code 2) "
              'when visible at the intersection and then continues towards '
              'their goal using RegularNav.',
        'TREE': '<root main_tree_to_execute = "HumanAtIntersectionTree">
             <include path="BTRegularNav.xml"/>
             <BehaviorTree ID="HumanAtIntersectionTree">
                 <Fallback name="VisibilityAndGestureFallback">
                     <Sequence name="WaitForProceedSequence">
                         <IsRobotVisible agent_id="{id}" distance="4.0" />
                         <RobotSays agent_id="{id}" message="2" />
                         <MakeGesture agent_id="{id}" message="0" />
                         <SetBlackboard output_key="agentid" value="{id}" />
                         <SetBlackboard output_key="timestep" value="{dt}" />
                         <SubTree ID="RegularNavTree" id="agentid" , dt="timestep" /> <!-- Using Subtree -->
                     </Sequence>
                     <Sequence name="RegNav">
                         <MakeGesture agent_id="{id}" message="0"/>
                         <SetBlackboard output_key="agentid" value="{id}"/>
                         <SetBlackboard output_key="timestep" value="{dt}"/>
                         <SubTree ID="RegularNavTree" id="agentid",dt="timestep" /> <!-- Using Subtree -->
                     </Sequence>
                 </Fallback>
             </BehaviorTree>
         </root>'} """
            }]
        }]
        
        
        new_behavior_query = {
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""Now, create a behavior tree for the following behavior:

<BEHAVIOR>

YOU MUST NOT USE ANY CUSTOM ACTION/CONDITION NODES AND ONLY USE THE NODES AVAILABLE.
Return an answer in JSON format shown below:"""+"""
{   
    'REASONING': <reasoning behind tree design>,
    'TREE': <XML Behavior Tree ONLY>,
}"""
        }]
    }   
        
        for msg in bt_example_1:
            self.payload.append(msg)
        for msg in bt_example_2:
            self.payload.append(msg)
        for msg in bt_example_3:
            self.payload.append(msg)
        
        self.payload.append(new_behavior_query)
            
    def get_full_prompt(self,**kwargs):
         full_prompt = self.payload
         full_prompt[-1]["content"][0]['text'] = full_prompt[-1]["content"][0]['text'].replace("<BEHAVIOR>",kwargs['behavior'])
         return full_prompt

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
    
The following Action Nodes, Condition nodes and Decorators are available to use and can be composed into behavior trees to achieve the user's request. 
Note that you can only fill in values for node attributes with "??".

    - BT Conditions
        - <IsRobotVisible agent_id="{id}" distance="??" /> : True if robot is in line of sight within ?? metres. 
        - <IsRobotNearby agent_id="{id}" distance="??" /> : True if robot is within ?? metres of the agent (regardless of line of sight).            
        - <RobotSays agent_id="{id}" message="??"/> :  True if robot is saying ??. The possible inputs and their meanings are: 0 (No gesture), 1("WAIT"), 2("PROCEED"),3("ACKNOWLEDGED"),4("EXCUSE ME"),5("FOLLOW ME"). 
        - <HumanSays agent_id="{id}" target_id="??" message="??"/> :  True if human with the id = target_id is saying ??. The possible inputs and their meanings are: 0 (No gesture), 1("WAIT"), 2("PROCEED"),3("ACKNOWLEDGED"),4("EXCUSE ME"),5("FOLLOW ME"). This is the same as the number of the human - 1. For example, Human_1 has id = 0 and Human_2 has id = 1
        - <RobotMoved agent_id="{id}"/>: True if the robot is currently moving. 
        - <IsRobotBlocking agent_id="{id}" distance="??"/>: True if the robot is blocking the agent's path and within ?? meters.
         - <TimeExpiredCondition seconds="??" ts="{dt}", only_once="??"): (Use with Inverter) Use this to make the agent do something for a fixed amount of time (for example block/follow/lookatrobot):
            Whatever you place after this within a sequence will be done for ?? seconds. If only_once='True', then once the timer is activated, whatever nodes follow this in a sequence will be done for that long and then never again. 
            To keep doing something every ?? seconds constantly you can set only_once='False'
            For example, observe the following sequence:
                <Sequence name='NoticeAndGiveWay'>
                        <Inverter>
                            <TimeExpiredCondition seconds='1.0' ts='{dt}' only_once='True'/>
                        </Inverter>
                        <IsRobotNearby agent_id='{id}' distance='2.0'/>
                        <MakeGesture agent_id='{id}' message='2'/>
                        <GiveWaytoRobot agent_id='{id}' time_step='{dt}'/>
                </Sequence>
                in this sequence, the agent will wait for 1 second and then the timer is up, so the rest of the nodes will never be run. 
            Now observe this sequence:
                <Sequence name='NoticeAndGiveWay'>
                    <IsRobotNearby agent_id='{id}' distance='2.0'/>
                    <Inverter>
                        <TimeExpiredCondition seconds='1.0' ts='{dt}' only_once='False'/>
                    </Inverter>
                    <MakeGesture agent_id='{id}' message='2'/>
                    <GiveWaytoRobot agent_id='{id}' time_step='{dt}'/>
                </Sequence>
                In this case, the agent will constantly check if the robot is nearby, then at every 1 second, it will make a gesture and give way to the robot

    - Note: In the simulator, for distance, 0.5 is considered very close, 1.0 is considered close, 2.0 is considered moderate distance and 5.0 is considered far
    - BT Actions:
        - <MakeGesture agent_id="{id}" message="??"/>: Makes the agent perform a gesture. Choices are: [0 (No gesture), 1("WAIT"), 2("PROCEED"), 3("EXCUSE ME")]. Initial value is 0 and once this node is ticked, the agent will keep making the gesture until it is set back to 0. 
        - <RegularNav agent_id="{id}" time_step="{dt}"/> : Makes the agent perform standard social-force-model based motion planning, where the robot is treated as a normal obstacle.
        - <LookAtRobot agent_id="{id}" /> : Makes the agent look in the direction of the robot
        - <FollowRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent follow the robot
        - <FollowHuman agent_id="{id}" target_id="??" time_step="{dt}"/>: Makes the agent follow the human with a specific target_id (= ??). This is the same as the number of the human - 1. For example, Human_1 has id = 0 and Human_2 has id = 1
        - <RunAwayFromRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent run away from the robot in a scared manner
        - <GiveWaytoRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent give way to the robot.
        - <BlockRobot agent_id="{id}" time_step="{dt}"/>: Makes the agent move in front of the robot and block it
    - Note that all these actions run perpetually unless they are put within an inverted TimeExpiredCondition block or after a condition. 
    - Note that generally, conditions should be followed by actions. It is rarely the case that an action is followed by a condition.
         - Example:
         1. Make the agent follow the robot perpetually:
            <Sequence name='follow_robot_perpetually'>
                <FollowRobot agent_id="{id}" time_step="{dt}"/>
            </Sequence>
        2. Make the agent follow the robot for 10 seconds:
            <Sequence name='follow_robot_perpetually'>
                <Inverter>
                    <TimeExpiredCondition seconds="10.0" ts="{dt}" only_once="True" />
                </Inverter>
                <FollowRobot agent_id="{id}" time_step="{dt}"/>
            </Sequence>
        3. Make the agent follow the robot until its no longer visible:
            <Sequence name='follow_robot_perpetually'>
                <IsRobotVisible agent_id="{id}" distance="2.0"/>
                <FollowRobot agent_id="{id}" time_step="{dt}"/>
            </Sequence>
            
    - BT Decorators:
        - Inverter: An inverter block inverts the output of its children (If children output failure, inverter outputs success and vice-versa).
    - In order to make the human go to their assigned next goal with regular navigation, use the following block as-is within the behavior tree without any modifications:
        <Sequence name="RegNav">
            <SetBlackboard output_key="agentid" value="{id}" /> 
            <SetBlackboard output_key="timestep" value="{dt}" /> 
            <SubTree ID="RegularNavTree" id="agentid" dt="timestep" /> 
        </Sequence> 

    - NOTE that the overall objective of a human is to reach their goal. Thus, ALWAYS ADD THE REGULAR NAVIGATION NODE AS A FALLBACK AS SHOWN BELOW:
    <?xml version='1.0' encoding='utf-8'?>
    <root main_tree_to_execute='WaitForProceedTree'>
        <include path='BTRegularNav.xml'/>
        <BehaviorTree ID='WaitForProceedTree'>
            <Fallback name='VisibilityFallback'>
                <Sequence name = '??'>
                
                <!--- ADD REQUIRED BEHAVIOR HERE     -->
                
                </Sequence>
                <Sequence name='RegNav'>
                    <SetBlackboard output_key='agentid' value='{id}' />
                    <SetBlackboard output_key='timestep' value='{dt}' />
                    <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
                </Sequence>
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
            <Inverter>
                <TimeExpiredCondition seconds="50" ts="{dt}" only_once="true" />
            </Inverter>
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
                <Sequence name='RegNav'>
                    <SetBlackboard output_key='agentid' value='{id}' />
                    <SetBlackboard output_key='timestep' value='{dt}' />
                    <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
                </Sequence>
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
        },
        {
            "role":"user",
            "content":[{
                "type":"text",
                "text":"""Now, create a behavior tree for the following behavior:

Human rushes towards the robot if the robot is visible and continuously blocks the robot for 40s and asks it to WAIT.

YOU MUST NOT USE ANY CUSTOM ACTION/CONDITION NODES AND ONLY USE THE NODES AVAILABLE.
Return an answer in JSON format shown below:"""+"""
{   
    'REASONING': <reasoning behind tree design>,
    'TREE DESCRIPTION': <Direct Translation of Behavior Tree to Simple English>,
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
    'TREE DESCRIPTION': "If the robot is visible,make the gesture ('WAIT') then block the robot for 40s. If the robot is not visible, navigate normally to the goal location"
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
        },     
        {
            "role":"user",
            "content":[{
                "type":"text",
                "text":""" Now, create a behavior tree for the following behavior:
If the robot is visible at the intersection and its moving, the human will wait for the robot to say 'PROCEED' and then continue towards their goal normally, treating the robot as a normal obstacle afterward.

YOU MUST NOT USE ANY CUSTOM ACTION/CONDITION NODES AND ONLY USE THE NODES AVAILABLE.
Return an answer in JSON format shown below:"""+"""
{   
    'REASONING': <reasoning behind tree design>,
    'TREE DESCRIPTION': <Direct Translation of Behavior Tree to Simple English>,
    'TREE': <XML Behavior Tree ONLY>,
}
"""
            }]
        },{
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{
                    
                    'REASONING':"The human waits for the robot to say 'PROCEED' (gesture code 2) "
              'when visible at the intersection and when it is moving, and then continues towards '
              'their goal using RegularNav.",
              'TREE DESCRIPTION':"If the robot is visible and it is moving and it says 'PROCEED' then make gesture (No Gesture) and continue regular navigation to goal. Otherwise, continue continue regular navigation to goal",
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
                         <Sequence name="RegNav">
                            <SetBlackboard output_key="agentid" value="{id}" />
                            <SetBlackboard output_key="timestep" value="{dt}"/>
                            <SubTree ID="RegularNavTree" id="agentid",dt="timestep" /> <!-- Using Subtree -->
                        </Sequence>
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
        },{
            "role":"user",
            "content":[{
                "type":"text",
                "text":""" Now, create a behavior tree for the following behavior:
The human ignores the robot.
YOU MUST NOT USE ANY CUSTOM ACTION/CONDITION NODES AND ONLY USE THE NODES AVAILABLE.
Return an answer in JSON format shown below:"""+"""
{   
    'REASONING': <reasoning behind tree design>,
    'TREE DESCRIPTION': <Direct Translation of Behavior Tree to Simple English>,
    'TREE': <XML Behavior Tree ONLY>,
}
"""
            }]
        },{
            "role":"assistant",
            "content":[{
                "type":"text",
                "text":"""{{
        'REASONING': "To make the human ignore the robot, it is sufficient for them to only continue normal navigation to the goal and not behave in any special way towards the human.",
        'TREE DESCRIPTION': "Regular navigation towards goal.",
        'TREE': '<root main_tree_to_execute = "HumanAtIntersectionTree">
             <include path="BTRegularNav.xml"/>
             <BehaviorTree ID="IgnoreRobotNav">
                     <Sequence name="RegNav">
                         <SetBlackboard output_key="agentid" value="{id}"/>
                         <SetBlackboard output_key="timestep" value="{dt}"/>
                         <SubTree ID="RegularNavTree" id="agentid",dt="timestep" /> <!-- Using Subtree -->
                     </Sequence>
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
    'TREE DESCRIPTION': <Description of the behavior generated by this tree in natural language>,
    'TREE': <XML Behavior Tree ONLY>,
}"""
        }]
    }           
        self.payload.append(new_behavior_query)
            
    def get_full_prompt(self,**kwargs):
         full_prompt = self.payload
         full_prompt[-1]["content"][0]['text'] = full_prompt[-1]["content"][0]['text'].replace("<BEHAVIOR>",kwargs['behavior'])
         return full_prompt

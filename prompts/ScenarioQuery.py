from .BasePrompt import BasePrompt
from pydantic import BaseModel,PositiveInt
    
class ScenarioQuery(BasePrompt):
    def __init__(self) -> None:
        super().__init__()
        self.payload = [
            {
                "role":"system",
                "content":[{
                    "type":"text",
                    "text":"Provide outputs strictly in JSON format"
                    }]
            },
            {"role":"user","content":[{"type":"text","text":"""
                                      
Social Navigation is a complex skill for a robot to accomplish and the appropriateness of the behavior of a robot is highly dependent on the task and the social context. 
Thus a robot’s social navigation capabilities must be thoroughly tested, and this is done by evaluating the robot’s behavior in a number of scenarios that are appropriate for a given social context and task.
A Social Navigation Scenario is defined by:
    1. Scenario Description: Very detailed description of the scenario. WHAT happens in the scenario and WHERE the scenario takes place. WHERE the robot and humans are located. For example, "Both Human 1 and Human 2 encounter the robot at an intersection".  
    2. Human Behavior:  How each human in the scenario interacts with the robot when it is visible, for e.g. "Human 1 is scared of the robot and asks it to stop", "Human 2 doesn't notice the robot at all and keeps walking" etc.

The user will provide a [Social context], a [Task] that the robot needs to do, a description of the [Location] and optionally a [Rough Scenario]. 
You are a scenario designer. Your task is to generate scenarios to test the social navigation capabilities of a robot, that align with the rough scenario, task and social context, and is an important scenario to test the social navigation capabilities of the robot.
Your generated scenario will be programmatically converted into a scenario in the Gazebo physics simulator.
Using your output scenario description, first, the humans and the robot in the scene will be given waypoints for motion planning, then, the behavior of the human agents will be coded with a behavior tree.
Your output description will be later used by an expert Behaviour tree designer to generate a Behavior Tree for each human in the scene. 

Rules:
- Ensure the output scenario is strictly aligned to the rough scenario
- When the scenario involves a GROUP, add all group members to the number of humans in the scenario.
- Specify locations in general terms (for example: 'intersection', 'stairs', 'open area' etc.), rather than specific location names (for example: 'intersection A', 'stairs B').
- In the simulator, the humans are simulated as cylinders with wheels, with sensors and a speaker. So, [HUMAN BEHAVIOR] should ONLY consist of behaviors like moving and interacting with the robot. Do not make the humans manipulate other objects in the scene, like pushing/pulling/carrying/moving objects etc. Do not suggest human behavior that a cylinder with sensors in the simulation cannot do. 
- The behavior of the human cylinders can only depend on the robot's behavior and not on what the other humans are doing.
- The human cylinders in the simulation have sensors that allow them to perform the following functions:
    - Check if the robot is visible
    - Check if the human has reached their goal
    - Check if robot is gesturing towards the human: The robot can say "WAIT", "PROCEED", "EXCUSE ME", "ACKNOWLEDGED".
    - Check if the robot is currently moving
    - Check if the robot is blocking the human's path
    - Perform a gesture: The humans can say "WAIT", "PROCEED" and "EXCUSE ME" to the robot.
    - Look in the direction of the robot
    - Follow the robot
    - Get scared of the robot and avoid it
    - Give way to the robot
    - Block the robot's path
        
    NOTE: AT ANY GIVEN POINT OF TIME, THE HUMAN CAN ONlY PEFORM ANY ONE OF THE ABOVE ACTIONS.

Design a scenario relevant to the following specifications:

[Social context]: Robot is a home assistant in a Singaporean old-age home and performs daily helpful duties for the residents
[Robot Task]: Deliver coffee
[Rough Scenario]: None
[Location]:   The home has a Kitchen, a Bedroom and a Living Room. A doorway connects the Kitchen to the Living Room and a Passageway connects the Living Room and the Doorway.

YOU ADHERE TO THE FOLLOWING JSON FORMAT STRICTLY. 
{
'Scenario Description': <very detailed description of the scenario, including the approximate path that the robot and the humans take and how the humans behave>,
'Number of Humans': <Number of humans that are involved in the scenario>,
‘Human behavior': {
‘Human 1’: <Describe the behavior of Human 1>,
‘Human 2’: <Describe the behavior of Human 2>,
},
'Expected Robot Behavior': <Describe the behavior expected from the robot>,
'Reasoning': <Reasoning behind the scenario and how the human behaviors can be emulated using human cylinders in the simulation>
}
"""}]},             
{
    "role":"assistant",
   "content":[{
        "type":"text",
        "text":"""
            {'Scenario Description': "The robot is trying to deliver coffee from one room to another through a doorway and encounters one of the elderly residents entering the room via the doorway.",
            'Number of Humans': 1,
            'Human Behavior':{
            'Human 1': If the robot is very close-by, Human asks the robot to stop and waits (for a maximum of 5s) for the robot to stop, then continues navigating. Human ignores the robot if the robot says "WAIT".},
            'Expected Robot Behavior': "The robot says "EXCUSE ME" to the resident. It waits for the resident to be well clear of the Doorway before going through the Doorway to the Living Room in a slow pace."
            },
            'Reasoning': A socially compliant robot should make its presence known to the elderly residents when they are unaware of the robot to avoid collisions. When performing a non-critical task like delivering coffee, the robot should not be obstructing the elderly resident's way of life and prioritize the resident's comfort over task efficiency. For Human 1, Navigation can be simulated by moving the human cylinder, speaking to the robot can be simulated with the speaker on the cylinder.
            """
            }]  
        }
]
        
    def get_full_prompt(self,**kwargs):
        self.payload.append(
            {
            "role":"user",
            "content":[{
                "type":"text",
                "text":
                    f"""
Design a scenario relevant to the following specifications:

[Social Context]:{kwargs['context']}
[Task]:{kwargs['task']}
[Location]:{kwargs['location']}
[Rough Scenario]:{kwargs['rough_scenario']}

YOU ADHERE TO THE FOLLOWING JSON FORMAT STRICTLY."""+"""
{
'Scenario Description': <very detailed description of the scenario >,
'Number of Humans': <Number of humans that are involved in the scenario>,
‘Human behavior': {
‘Human 1’: <Describe the behavior of Human 1>,
‘Human 2’: <Describe the behavior of Human 2>,
....
},
'Expected Robot Behavior': <Describe the behavior expected from the robot>
'Reasoning': <Reasoning behind the scenario and how the human behaviors can be emulated using human cylinders in the simulation>
}

"""
                }]
            }
        )
        
        return self.payload
        
        
        
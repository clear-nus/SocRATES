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
                    "text":"Provide outputs strictly in JSON format."
                    }]
            },
            {"role":"user","content":[
                {
                    "type":"text",
                    "text":"""
                                      
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
    - Check if the robot is visible within a certain distance
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
    - Do Nothing
    - Set a timer
    - Navigate to goal 
        
    NOTE: AT ANY GIVEN POINT OF TIME, THE HUMAN CAN ONlY PEFORM ANY ONE OF THE ABOVE ACTIONS.
- Make sure that you DO NOT SPECIFY OVERLY COMPLICATED behaviors for the humans because the simulator won't be able to handle it. However, make sure that the human behaviors are aligned with the description in the rough scenario
- Every single action or condition involved in the human's behavior, should be from the set of capabilities mentioned above. For example, the humans cannot do something depending what another human does or pickup something or jump etc. because these capabilities are not present in the above list.

Now answer the following question:
Is the following a valid behavior for the simulated human: 
    Human 1 is walking towards an aisle and picks up a box when he reaches there. He ignores the robot when the robot says 'EXCUSE ME'
"""}]},
            {
                "role":"assistant",
                "content":[{
                    "type":"text",
                    "text":""" The following behaviors are required for simulating this behavior:
                    - Identify where an aisle is (not possible)
                    - Walk towards a goal (possible: Navigate to goal)
                    - Pick up box (not possible)
                    - Ignore robot (possible: Do Nothing)
                    - Recognize when the robot says 'EXCUSE ME' (possible)
                    Because there are non-simulatable actions/conditions, it is not possible to simulate this behavior. """
                }]
            },
            {
                "role":"user",
                "content":[{
                    "type":"text",
                    "text":"""
Is the following a valid behavior for the simulated human: 
If the robot is close to Human 1, He looks at what Human 2 is doing and then get's scared and goes away from the robot.
                    """
                }]
            },
            {
                "role":"assistant",
                "content":[{
                    "type":"text",
                    "text":"""
                    The following behaviors are required for simulating this behavior: 
                    - check if robot is close (possible: Check if the robot is visible within a certain distance)
                    - look at Human 2 (not possible)
                    - check what Human 2 is doing (not possible)
                    - get scared and go away from robot (possible: get scared of robot and avoid it)
                    Because there are non-simulatable actions/conditions, it is not possible to simulate this behavior. 
                    """
                }]
            },
            {
                "role":"user",
                "content":[{
                    "type":"text",
                    "text":"""
                    Is the following a valid behavior for the simulated human:
If the robot is blocking the human's way, then he says 'EXCUSE ME' and avoids the robot. 
                    """
                }]
            },
            {
                "role":"assistant",
                "content":[{
                    "type":"text",
                    "text":""" 
                    The following behaviors are required for simulating this behavior: 
                    - Check if the robot is blocking their way (possible: Check if the robot is blocking the human's path)
                    - Say 'EXCUSE ME' (possible: Perform a gesture (EXCUSE ME))
                    - Avoid the robot (possible: Get scared and avoid it or Give way to the robot.)
                    """
                }]
            },
    {
    "role":"user",
    "content":[{
        "type":"text",
        "text":"""Now Design a scenario relevant to the following specifications:

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
'Reasoning': <Why is it important to test the robot in this scenario + How to simulate the proposed human behavior>
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
                        'Human 1': If the robot is very close-by, Human asks the robot to stop and waits (for a maximum of 5s) for the robot to stop, then continues navigating.},
                        'Expected Robot Behavior': "The robot says "EXCUSE ME" to the resident. It waits for the resident to be well clear of the Doorway before going through the Doorway to the Living Room in a slow pace."
                        },
                        'Reasoning': 
                        - Scenario importance: ""A socially compliant robot should make its presence known to the elderly residents when they are unaware of the robot to avoid collisions. When performing a non-critical task like delivering coffee, the robot should not be obstructing the elderly resident's way of life and prioritize the resident's comfort over task efficiency.",
                        - Simulating proposed human behavior: "To simulate human 1, the human can perform regular navigation ("Navigate to goal") unless the robot is very closeby ("Check if the robot is visible within a certain distance"), otherwise the human asks the robot to wait ("Perform a gesture: The humans can say "WAIT", "PROCEED" and "EXCUSE ME" to the robot.") and waits ("Do Nothing") for 5s("Set a timer") for the robot to stop ("Check if the robot is currently moving"))
                        "
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
'Reasoning':  <Why is it important to test the robot in this scenario + How to simulate the proposed human behavior>
}

"""
                }]
            }
        )
        
        return self.payload
        
        
        
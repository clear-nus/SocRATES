from .BasePrompt import BasePrompt
class ScenarioQuery(BasePrompt):
    def __init__(self) -> None:
        super().__init__()
        self.required_output_keys = ['scenariodescription','numberofhumans','humanbehavior','expectedrobotbehavior']
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
Thus a robot’s social navigation capabilities must be thoroughly tested, and this is done by evaluating the robot’s behavior in a number of scenarios in a variety of contexts.
You are a scenario designer. Your task is to generate scenarios to test the social navigation capabilities of a robot.
A Social Navigation [Scenario] is defined by:
    1. Scenario Description: very detailed description of the scenario. WHAT happens in the scenario and WHERE the scenario takes place. WHERE the robot and humans are located.
    2. Human Behavior:  how human interacts with the robot when it is visible, for e.g. Human 1 is scared of the robot and asks it to stop, Human 2 doesn't notice the robot at all etc.
Your output description will be later used by an expert Behaviour tree designer to generate a Behavior Tree for each human in the scene. 

The behavior tree designer is not allowed to modify the scenario and can only create behavior that can be generated using the following Actions and Conditions:
 - Conditions
        - Check the visibility of the robot
        - Check if the human has reached their goal
        - Check if robot is saying any particular phrase
        - Check if the robot is currently moving
        - Check if the robot is blocking the human's path
    
    - Actions:
        - Make the human perform a gesture.
        - Make the human perform normal navigation to reach its goal and treat the robot as a normal obstalce. This is regular behavior for humans.
        - Make the human look in the direction of the robot
        - Make the human follow the robot
        - Make the human scared of the robot and avoid it.
        - Make the human give way to the robot
        - Make the human move quickly towards the front of the robot and block the robot.
        
        NOTE: AT ANY GIVEN POINT OF TIME, THE HUMAN CAN ONlY PEFORM ANY ONE OF THE ABOVE ACTIONS.

The humans are only capable of performing the actions mentioned above.
User will provide a [Social context], a [Task] that the robot needs to do, a description of the location and optionally a [Rough Scenario]. 
Your generated scenario will be programmatically simulated through a pipeline into a scenario in the Gazebo physics simulator.
Rules:
- The humans can say "WAIT", "PROCEED", "EXCUSE ME" to the robot  to aid in navigation. The robot can say "WAIT", "PROCEED", "EXCUSE ME", "ACKNOWLEDGED" to the humans to aid in navigation. 
- When the user provides a Rough Scenario, ensure your final scenario is strictly aligned to the rough scenario
- The humans in the simulator are SIMPLIFIED OBJECTS that only can move in 2D, send and receive simple phrases, detect the robot, look at the robot, group together with other humans, navigate to a predefined goal and change their trajectory conditioned on the robot’s position and velocity.
- When using groups in the scenario, add all group members to the humans in the scenario. Having only 1 human with 'INTERACTING WITH GROUP' task is incorrect.

Design a scenario relevant to the following specifications:

[Social context]: Robot is a home assistant in a Singaporean old-age home and performs daily helpful duties for the residents
[Robot Task]: Deliver coffee
[Rough Scenario]: None
[Location]:   The home has a Kitchen, a Bedroom and a Living Room. A doorway connects the Kitchen to the Living Room and a Passageway connects the Living Room and the Doorway.

YOU ADHERE TO THE FOLLOWING JSON FORMAT STRICTLY. 
{
'Scenario Description': <very detailed description of the scenario >,
'Number of Humans': <Number of humans that are involved in the scenario>,
‘Human behavior': {
‘Human 1’: <Describe the behavior of Human 1>,
‘Human 2’: <Describe the behavior of Human 2>,
},
'Expected Robot Behavior': <Describe the behavior expected from the robot>
}
"""}]},             
{
    "role":"assistant",
   "content":[{
        "type":"text",
        "text":"""
            {'Scenario Description': "The robot is trying to deliver coffee from the Kitchen to the Living Room and encounters one of the elderly residents entering the Kitchen from the Living Room through the  Doorway.",
            'Number of Humans': 1,
            'Human Behavior':{
            'Human 1': Human 1 is going from going to kitchen from the living room. If the robot is very close-by, Human asks the robot to stop and waits (for a maximum of 5s) for the robot to stop, then continues navigating. Ignores the robot if it asks the human to wait.
            },
            'Expected Robot Behavior': "The robot says "I AM HERE" to the resident. It waits for the resident to be well clear of the Doorway before going through the Doorway to the Living Room in a slow pace."
            }
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
},
'Expected Robot Behavior': <Describe the behavior expected from the robot>
}

"""
                }]
            }
        )
        
        return self.payload
        
        
        
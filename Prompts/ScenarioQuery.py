from .BasePrompt import BasePrompt
class ScenarioQuery(BasePrompt):
    def __init__(self) -> None:
        super().__init__()
        self.background = """ Social navigation involves a robot navigating to a goal in the presence of humans and adhering to the following principles:
    Safety (P1): Ensure no harm comes to people or property by avoiding collisions and risky behaviors.
    Comfort (P2): Maintain personal space and move in ways that do not startle or distress individuals.
    Legibility (P3): Make the robot’ s actions predictable and understandable to those around it.
    Politeness (P4): Navigate with courtesy, often signaling intent and yielding way as needed.
    Social Competency (P5): Adhere to social norms and cultural expectations in shared environments. Adhering to Cultural norms is also very important and included within this.
    Understanding Other Agents (P6): Recognize and appropriately respond to the behaviors and intentions of others.
    Proactivity (P7): Anticipate and address potential social dilemmas or conflicts before they arise.
    Task Efficiency (P8): Complete the given task in a timely manner without sacrificing other social principles.
However, since its not possible to optimize all of these at the same time always, depending on the social situation and the task, the robot must favor some principles over the other. For example, when the robot is trying to deliver an emergency cart to a patient, P2 and P5 take less precedence over P1 and P8. Similarly, in a library, when a robot is delivering a book, P5, P2 and P1 are more important to optimize than P8.
Social Navigation is a complex skill for a robot to accomplish and the appropriateness of the behavior of a robot is highly dependent on the task and the social context. Thus a robot’s social navigation capabilities must be thoroughly tested, and this is done by evaluating the robot’s behavior in a number of scenarios in a variety of contexts.
    """
        self.main_system = """You are a scenario designer. Your task is to generate scenarios to test the social navigation capabilities of a robot.
User will provide a [Social context], a [Task] and the associated [Scene Graph] of the location. 
You must provide an appropriate [Scenario] to test the performance of the robot's navigation algorithm, along with the corresponding [Expected behavior] of the robot based on a [Ranking] of the priorities of the Principles for the [Scenario]. You must also explain the [Reasoning] behind why its important to test the robot in this [Scenario] given the User input.
Use scene graph nodes when refering to locations. A Scene graph node is defined in the following format: id (name (optional)). For example, 1 (Bedroom), 2, 5 (Bathroom) etc.
Assume that the robot being tested is a simple navigation bot. It can also say the following phrases ONLY: [”WAIT”, “PROCEED”, “MAKE WAY”, “I AM HERE”].

Human agents in the scenario are overly simplified humans capable of doing the following ONLY:
    1. Navigating from one point to another
    2. Navigating from one point to another through specific waypoints
    3. Saying the following to the robot ["WAIT","PROCEED","COME HERE","GO AWAY"]
    4. Gathering in groups of sizes ranging from 2-5. This includes Forming a group, joining a group, exiting a group. 
    5. Intereacting with the robot, for example, approaching the robot, following the robot, blocking the robot, looking at the robot, walking along with the robot, leading the robot.
    """
        self.output_format = """
YOU ADHERE TO THE FOLLOWING OUTPUT FORMAT STRICTLY. This is very important since your ouput will be used by a program later on. Do not provide any additional explanation. 
    {
        'Scenario Description': < Subjective description of the scenario >,
         'Number of Humans': <Number of humans that are involved in the scenario>,
         'Trajectories':{
        'Robot': <comma separated sequence of Scene Graph nodes>,
        'Human 1': ....
        'Human 2': ...
                        },
    {'Behaviors':{
        'Human 1': <Description of behavior of Human 1 w.r.t the robot>,
        'Human 2':...,
    },
        
    'Expected Robot Behavior': <Describe the behavior expected from the robot>,
    'Principle Ranking': The order of importance for the 8 principles for this scenario,
    'Reasoning': <4-5 line description of why this scenario is relevant for the given input for testing>
    }
    """
        self.examples = """### Example:
User:
[Social context]: Robot is a home assistant in a Singaporean old-age home and performs daily helpful duties for the residents
[Task]: Deliver coffee
[Scene Graph]:
{'nodes': [{'type': 'room', 'id': '1 (Bedroom)'}, {'type': 'room', 'id': '2 (Living Room)'}, {'type': 'room', 'id': '3 (Kitchen)'}, {'type': 'connector', 'id': '4 (Doorway)'}], 'links': ['1 (Bedroom)<->2 (Living Room)', '2 (Living Room)<->4 (Doorway)', '3 (Kitchen)<->4 (Doorway)']}

Assistant:
    {'Scenario Description': "The robot is trying to deliver coffee from the 3 (Kitchen) to the 2 (Living Room) and encounters one of the elderly residents entering the 3 (Kitchen) from the 2 (Living Room) through the 4 (Doorway).",
    'Number of Humans': 1,
    'Trajectories':{
        'Robot': ["3 (Kitchen)","4 (Doorway)", "2 (Living Room)"],
        'Human 1': ["Living Room(2)","Doorway(4)","3 (Kitchen)"]
                    },
    'Behaviors':{
        'Human 1': "Human has bad eyesight and cannot see the robot unless its very close. When the human sees the robot, they stop until the robot says something (for a maximum of 5 seconds) and then continue their activity."
        },
    'Expected Robot Behavior': "The robot says "I AM HERE" to the resident. It waits for the resident to be well clear of the Doorway(4) before saying "MOVING" and going through the Doorway(4) to the 2 (Living Room) in a slow pace.",
    'Principle Ranking': "Safety (P1)>Social Competency (P5)>Understanding Other Agents (P6)>Legibility(P3)>Comfort(P2)>Politeness(P4)>Proactivity(P7)>Task Efficiency (P8)",
    'Reasoning': "It is important to evaluate the behavior of the robot when being confronted in close spaces by different types of humans. In an elderly home, there are likely elderly residents doing daily tasks who could be scared of the robot."
    }
###
    """
    def get_full_prompt(self,**kwargs):
        return dict(
            user=[{'type':'text',
                   'content':self.background + "\n" + self.main_system + "\n" + self.output_format + "\n" + self.examples + "\n" + f"""
USER:
[Social Context]:{kwargs['context']}
[Task]:{kwargs['task']}
[SCENE GRAPH]:{kwargs['graph']}
Assistant:
"""
            },
            ],
            system = self.system_prompt)
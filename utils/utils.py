import base64
import json
from os import error
import tiktoken
import math
import io
import re
import PIL.Image as Image
from io import BytesIO
from collections.abc import Mapping
import networkx as nx
from PIL import Image as PILImage
import random
from termcolor import colored
import pprint
import xml.etree.ElementTree as ET
from termcolor import cprint
from pydantic import BaseModel,StrictInt,PositiveInt
class HumanTraj(BaseModel):
    name: str
    groupid: StrictInt
    trajectory: list[str]
    interaction_point: str
    
class Trajectories(BaseModel):
    robot: list[str]
    humans: list[HumanTraj]

class StructuredScenarioReasoning(BaseModel):
    scenario_importance: str
    num_humans: int
    simulating_humans: list[str]
    
class StructuredTrajResponse(BaseModel):
    reasoning: str
    trajectories: Trajectories

class StructuredBTResponse(BaseModel):
    reasoning: str
    tree_description:str
    tree: str
    
class Behavior(BaseModel):
    name: str
    behavior: str
    
class StructuredScenarioResponse(BaseModel):
    scenariodescription: str
    numberofhumans: int
    humanbehavior: list[Behavior]
    reasoning: StructuredScenarioReasoning
    expected_robot_behavior: str

class StructuredResponse(BaseModel):
    scenario: StructuredScenarioResponse
    behavior: list[StructuredBTResponse]
    trajectories: StructuredTrajResponse  

eprint = lambda x:cprint(x,'red') #error
iprint = lambda x:cprint(x,'yellow') #user input
rprint = lambda x:cprint(x,'green') #result
lprint = lambda x:cprint(x,'blue') #log

def parse_questions_answers(file_path):
    questions_answers = []

    # Open the file and read its contents
    with open(file_path, 'r') as file:
        content = file.read()

    # Split the content by the separator "*****"
    qa_pairs = content.split("*****")

    # Iterate over each QA pair
    for qa in qa_pairs:
        if qa.strip():  # Ensure the string is not empty
            lines = qa.strip().splitlines()

            # Initialize variables to hold the question and answer
            question_lines = []
            answer_lines = []
            current_section = None

            # Parse each line to separate question and answer
            for line in lines:
                if line.startswith("Q:"):
                    current_section = "question"
                    question_lines.append(line[2:].strip())
                elif line.startswith("A:"):
                    current_section = "answer"
                    answer_lines.append(line[2:].strip())
                else:
                    # Append to the current section (either question or answer)
                    if current_section == "question":
                        question_lines.append(line.strip())
                    elif current_section == "answer":
                        answer_lines.append(line.strip())

            # Join the lines to form complete question and answer texts
            question = "\n".join(question_lines).strip()
            answer = "\n".join(answer_lines).strip()

            if question and answer:
                questions_answers.append((question, answer))

    return questions_answers

def validate_bt(tree,node_library,debug=False):
    regular_nav_present = False
    if len(tree.find('BehaviorTree'))!=1:
        error_string = "Behavior Tree node can have only a single child"
        if debug:
            eprint("Retrying Behavior Generation..")
            eprint(error_string)
        return False,error_string
    
    set_blackboard_nodes = tree.findall(".//SetBlackboard")
    #there should be a multiple of 2 blackboard nodes
    if len(set_blackboard_nodes)%2!=0:
        eprint("SetBlackBoard Problem")
        error_string = "Theres a problem with the SetBlackboard node. Have you used too many unnecesarily or too few? Remmeber that SetBlackBoard can only be used for regular navigation"
        return False,error_string
    
    for elem in tree.iter():
        if elem.tag not in node_library:
            error_string = f'{elem.tag} not in node_library'
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False,error_string
        
        if elem.tag == 'SubTree':
            if elem.attrib!={'ID': 'RegularNavTree', 'id': 'agentid', 'dt': 'timestep'}:
                error_string = """RegularNav's attributes are wrong, it should be: <Sequence name='RegNav'>
                <SetBlackboard output_key='agentid' value='{id}'/>
                <SetBlackboard output_key='timestep' value='{dt}'/>
                <SubTree ID='RegularNavTree' id='agentid' dt='timestep'/>
            </Sequence>"""
                if debug:
                    eprint("Retrying Behavior Generation..")
                    eprint(error_string)
                return False,error_string
            #check if RegularNav is included for SubTree
            included_files = tree.findall('.//include')
            if len(included_files)!=1:
                error_string = "File for regularnav is not included. Add it after the root element:  <include path='BTRegularNav.xml'/>"
                if debug:
                    eprint("Retrying Behavior Generation..")
                    eprint(error_string)
                return False, error_string
            if included_files[0].attrib != {'path': 'BTRegularNav.xml'}:
                error_string = "Path of the included file for regularnav is wrong. The correct way is: <include path='BTRegularNav.xml'/>"
                if debug:
                    eprint("Retrying Behavior Generation..")
                    eprint(error_string)
                return False,error_string
            regular_nav_present = True
            continue
         
        #check for incorrect nodes
        for k,v in elem.attrib.items():
            if k not in node_library[elem.tag]:
                error_string = f'{k} not in node_library[{elem.tag}]'
                if debug:
                    eprint("Retrying Behavior Generation..")
                    eprint(error_string)
                return False,error_string
            if k == 'agent_id':
                if v!="""{id}""":
                    error_string = """Incorrect agent id. Only use "agent_id" in the nodes, nothing else"""
                    if debug:
                        eprint("Retrying Behavior Generation..")
                        eprint(error_string)
                    return False,error_string
            
        #check if all attributes are correct
        if len(node_library[elem.tag])!=len(elem.attrib.keys()):
            error_string = f"{node_library[elem.tag]} has incorrect attributes. They should be: {node_library[elem.tag]}"
            if debug:
                eprint("Retrying Behavior Generation..")
                eprint(error_string)
            return False,error_string
        
        for v in node_library[elem.tag]:
            if v not in list(elem.attrib.keys()):
                error_string = f"{node_library[elem.tag]} has incorrect attributes. They should be: {node_library[elem.tag]}"
                if debug:
                    eprint("Retrying Behavior Generation..")
                    eprint(error_string)
                return False,error_string
    if not regular_nav_present:
        error_string = "You haven't added regularNav subtree. Remember to add the blackboards and include the required file (for agentid and dt)"
        if debug:
            eprint(error_string)
        return False, error_string
    return True, None

def get_img_from_path(img_path):
    with open(img_path, "rb") as image_file:
        encoded_string = base64.b64encode(image_file.read())
    return encoded_string.decode("utf-8")

def filter_scene_graph(scene_graph, node_type_to_remove):
    #for simple serialized dict graphs
    G = {'nodes':[],'links':[]}
    filtered_nodes = []
    # Add nodes to the graph if they are not of the type to remove
    for node in scene_graph['nodes']:
        if node['type'] != node_type_to_remove:
            G['nodes'].append(node)
            filtered_nodes.append(str(node['id']))
    
    # Add edges to the graph, only if both nodes in an edge are still in the graph
    for edge in scene_graph['links']:
        node1, node2 = edge.split('<->')
        if node1 in filtered_nodes and node2 in filtered_nodes:
           G['links'].append(edge)
    
    # Create the new scene graph structure to return
    return G

def load_imgs_for_prompt(img_path):
    dims = PILImage.open(img_path).size
    img_cost = calculate_image_token_cost(dims)
    print(f'{dims}:{img_cost}')
    with open(img_path, "rb") as image_file:
        encoded_img = base64.b64encode(image_file.read()).decode('utf-8')
    return encoded_img,img_cost
        
#https://github.com/openai/tiktoken/issues/250
def calculate_image_token_cost(dims, detail="auto"):
    # Constants
    LOW_DETAIL_COST = 85
    HIGH_DETAIL_COST_PER_TILE = 170
    ADDITIONAL_COST = 85

    if detail == "auto":
        # assume high detail for now
        detail = "high"

    if detail == "low":
        # Low detail images have a fixed cost
        return LOW_DETAIL_COST
    
    elif detail == "high":
        # Calculate token cost for high detail images
        width, height = dims
        # Check if resizing is needed to fit within a 2048 x 2048 square
        if max(width, height) > 2048:
            # Resize the image to fit within a 2048 x 2048 square
            ratio = 2048 / max(width, height)
            width = int(width * ratio)
            height = int(height * ratio)
        # Further scale down to 768px on the shortest side
        if min(width, height) > 768:
            ratio = 768 / min(width, height)
            width = int(width * ratio)
            height = int(height * ratio)
        # Calculate the number of 512px squares
        num_squares = math.ceil(width / 512) * math.ceil(height / 512)
        # Calculate the total token cost
        total_cost = num_squares * HIGH_DETAIL_COST_PER_TILE + ADDITIONAL_COST
        return total_cost
    else:
        # Invalid detail_option
        raise ValueError("Invalid value for detail parameter. Use 'low' or 'high'.")

def pretty_print_conversation(messages):
    role_to_color = {
        "system": "red",
        "user": "green",
        "assistant": "blue",
        "function": "magenta",
    }
    
    for message in messages:
        if message["role"] == "system":
            print(colored(f"system: {message['content']}\n", role_to_color[message["role"]]))
        elif message["role"] == "user":
            print(colored(f"user: {message['content']}\n", role_to_color[message["role"]]))
        elif message["role"] == "assistant" and message.get("function_call"):
            print(colored(f"assistant: {message['function_call']}\n", role_to_color[message["role"]]))
        elif message["role"] == "assistant" and not message.get("function_call"):
            print(colored(f"assistant: {message['content']}\n", role_to_color[message["role"]]))
        elif message["role"] == "function":
            print(colored(f"function ({message['name']}): {message['content']}\n", role_to_color[message["role"]]))

def get_image_bytes_from_url(image_url: str) -> bytes:
    im = Image.open(image_url)
    buf = io.BytesIO()
    im.save(buf, format="JPEG")
    return buf.getvalue()

def load_image_from_url(image_url: str) -> Image:
    image_bytes = get_image_bytes_from_url(image_url)
    return Image.from_bytes(image_bytes)    
    

def num_tokens_from_messages(message: Mapping[str, object], model: str) -> int:
    """
    Calculate the number of tokens required to encode a message.
    Args:
        message (Mapping): The message to encode, in a dictionary-like object.
        model (str): The name of the model to use for encoding.
    Returns:
        int: The total number of tokens required to encode the message.
    Example:
        message = {'role': 'user', 'content': 'Hello, how are you?'}
        model = 'gpt-3.5-turbo'
        num_tokens_from_messages(message, model)
        output: 11
    """

    encoding = tiktoken.encoding_for_model(model)
    num_tokens = 2  # For "role" and "content" keys
    for value in message.values():
        if isinstance(value, list):
            for item in value:
                num_tokens += len(encoding.encode(item["type"]))
                if item["type"] == "text":
                    print("detected text")
                    num_tokens += len(encoding.encode(item["text"]))
                elif item["type"] == "image_url":
                    print(f"""detected {item["image_url"]["detail"]} detail image""")
                    num_tokens += calculate_image_token_cost(item["image_url"]["url"], item["image_url"]["detail"])

        elif isinstance(value, str):
            num_tokens += len(encoding.encode(value))
        else:
            raise ValueError(f"Could not encode unsupported message value type: {type(value)}")
    return num_tokens

def get_image_dims(image):
    if re.match(r"data:image\/\w+;base64", image):
        image = re.sub(r"data:image\/\w+;base64,", "", image)
        image = Image.open(BytesIO(base64.b64decode(image)))
        return image.size
    else:
        raise ValueError("Image must be a base64 string.")
    
def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def createHuman(agents_file,bt_file,params):
    #Edit Agents.yaml
    yaml_agent = f"""
    agent{params['id']}:
      id: {params['id']}
      skin: 0
      behavior: {params['behavior']}
      group_id: {params['group_id']}
      max_vel: 1.5
      radius: 0.4
      init_pose:
        x: {params['x']}
        y: {params['y']}
        z: 1.250000
        h: {params['h']}
      goal_radius: 0.3
      cyclic_goals: {params['cyclic_goals']}
      goals:
    """
    #add goals
    for i,goal in enumerate(params['goals']):
        yaml_agent += f"""
                    - g{i}"""
    
    for i,goal in enumerate(params['goals']):
        yaml_agent+=f"""
                g{i}:
                    x:{goal['x']}
                    y:{goal['y']}
                    h:{goal['h']}"""
    
    with open(agents_file,"a") as f:
        f.write(yaml_agent)
    #Edit BT file
    with open(bt_file,'w') as f:
        f.write(params['bt'])



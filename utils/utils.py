
import base64
import json
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

def pix2world(px):
    return [-1.0*((px[1]/3.0034965034965) * 0.050000 + -7.000) ,-1*((px[0]/2.6604554865424) * 0.050000 + -10.500000)]

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


class SceneGraph:
    def __init__(self, serialized_graph):
        '''
        Nodes have attributes:
            - type
            - ID (name)
            - position (pixel)
        '''
        self.graph = nx.node_link_graph(serialized_graph) #nx graph
        nx.set_edge_attributes(self.graph, {e: self.dist(e[0],e[1]) for e in self.graph.edges()}, "cost")            

    def get_parent_nodes(self):
        '''
        This function returns the parent nodes in the graph
        '''
        return [n for n in self.graph.nodes if self.graph.nodes[n]['type']!='child']

    def dist(self,n1,n2):
        (x1,y1) = self.graph.nodes[n1]['pos']
        (x2,y2) = self.graph.nodes[n2]['pos']
        return ((x1-x2)**2+(y1-y2)**2)**0.5        

    def sampleNodeOfType(self,node_type):
        '''
        This function samples nodes in the graph that are of the input type
        '''
        nodes = [n for n in self.graph.nodes if self.graph.nodes[n]['type']==node_type]
        if len(nodes)==0:
            return f"No node of type {node_type} exists in the graph"
        return random.choice(nodes)
        
    def relativeDirection(self,node1,node2):
        '''
        This function returns the relative direction between two nodes.
        for example, 'right,below' means that node1 is to the right and below node2
        '''
        try:
            (x1,y1) = self.graph.nodes[node1]['pos']
            (x2,y2) = self.graph.nodes[node2]['pos']
        except KeyError:
            return "These nodes don't exist in the graph"
        direction = [None,None]
        if x1>=x2:
            direction[0] = 'right'
        elif x1<=x2:
            direction[0] = 'left'
        elif y1>=y2:
            direction[1] = 'below'
        elif y1<=y2:
            direction[1] = 'above'
        else:
            return 'None'
        return direction
    
    def connectedNodes(self,node):
        '''
        This function returns all the neighbours of the input node and their types and distance from the node
        '''
        cnodes = 'None'
        try:
            neighbours =  list(self.graph.neighbors(node))
            cnodes = {}
            for c in neighbours:
                cnodes[self.graph.nodes[c]] = self.graph.edges[(node,c)]['cost']
        except nx.NetworkXError:
            return "No such node exists in the graph"
        return cnodes
    
    
    def planPath(self,node1,node2):
        '''
        This function finds the shortest path between node1 and node2
        '''
        # returns -1 if no path exists
        cost = 0
        try:
            path =  nx.astar_path(self.graph,node1,node2, weight = "cost")
            n1 = path[0]
            if len(path) > 1:
                for n in path[1:]:
                    cost += self.graph.edges[n1,n]["cost"]
                    n1 = n
        except nx.NetworkXNoPath:
            cost = -1
        return cost
    
    def isvalidtrajectory(self,trajectory):
        #checks if a given node sequence is valid or not and returns the errors
        #input: trajectory: sequence of node names in the graph
        errors = []
        traj_valid = True
        for first,second in zip(trajectory,trajectory[1:]):
            if (first,second) not in self.graph.edges:
                errors.append((first,second))
                traj_valid = False
        return traj_valid, errors
        
    
def gpt_function_call(graph,message):
    for msg in message:
        fn_name = msg.message.function_call.name()
        fn_arguments = json.loads(msg.message.function_call.arguments)
        try:
            if fn_name == 'closestNodeOfType':
                graph.closestNodeOfType()    
            elif fn_name == "connectedNodes":
                graph.connectedNodes()
            elif fn_name == "closestNodes":
                graph.closestNodes()
            elif fn_name == "pathLength":
                graph.pathLength()
            else:
                raise Exception("Function not found")
            
        except Exception as e:
            print(f"Function Execution Failed")
            print(e)
            
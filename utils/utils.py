import base64
import json
import tiktoken
import math
import io
import re
import PIL.Image as Image
from io import BytesIO
from collections.abc import Mapping

#https://github.com/openai/tiktoken/issues/250
def calculate_image_token_cost(image, detail="auto"):
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
        width, height = get_image_dims(image)
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
    
### BT Related

### Function to add a custom node to Hunavsim
def add_custom_bt_node():
    
    #Add node to extended_bt_functions.hpp to header file
    
    #Add node to extended_bt_functions.cpp
    
    #Add node ports to extended_bt_node.cpp
    
    #Add node register to extended_bt_node.cpp
    
    pass

def add_agentmanager_function():
    
    #Add function definition to extended_agent_manager.hpp file
    
    #Add function to extended_agent_manager.cpp file
    
    pass

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

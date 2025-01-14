from .BasePrompt import BasePrompt
from pydantic import BaseModel,StrictInt
# This query uses the assistants API instead    
class TrajectoryQuery(BasePrompt): 
    def __init__(self,ref_img_encoded,ref_scene_graph,qa) -> None:
        '''
            scene_graph: scene graph of the location in text-serialized format.
            llm_scenario_out: output of the scenario generated by llm, parsed into a dictionary.
        '''
        
        super().__init__()
        self.payload = [
            {"role": "system", 
             "content": [{
                "type":"text",
                "text":"""
You are an expert floor planner and a software engineer. You ALWAYS provide output that in JSON which is fully parseable with json.loads in python. You will be provided with scene graphs in JSON and map images of locations overlaid with the corresponding scene graph. Your task is to answer the questions based on scene graph and image.
             """}]
            },
            {"role": "user", 
             "content": [{
                    "type":"text",
                    "text":f"""
The image shows a location which is represented by a scene graph. A scene graph is a graph with nodes (yellow squares with blue outline) representing locations and edges (red lines) connecting them. 
A person can only move from one node to another if the two nodes are connected by an edge. 
The scene graph in json file format is also given below, where each node and edge has a unique id, a type, the pixel position of the node in the image (pos). 
{ref_scene_graph}
The type of a node is one among: <NODE TYPES> while the edges are one among: <EDGE TYPES>. 
The graph is bidirectional and each edge is also represented with the edge list 'links' in the json given below.  
Remember that a node can only be reached from another node if they have an edge between them in the scene graph json.
            """},
            {         
                "type":"image_url",
                "image_url":{
                    "url":f"data:image/jpeg;base64,{ref_img_encoded}",
                    "detail":"high"
                }
            }]
            }
            ]
        
        for i,item in enumerate(qa):
            if i == 0:
                self.payload[-1]['content'].append(
                    {
                "type":"text",
                "text":item[0] #question
                }
                )
            else:
                self.payload.append({
                    "role":"user",
                    "content":[{
                    "type":"text",
                    "text":item[0] #question
                }]})
            
            self.payload.append({
                "role":"assistant",
                "content":[{
                "type":"text",
                "text":item[1] #answer
            }]})
        
        
    def get_full_prompt(self,**kwargs):
        full_prompt = self.payload
        self.payload.append(
            {"role": "user", "content": [{"type":"text","text":f"""
Now consider a new location with the following scene graph: 
{kwargs['scene_graph']}
The Node are of type: {kwargs['node_types']} while the edges are of type: {kwargs['edge_types']}. 
The map image with the scene graph is given in the next image."""},
{
    "type":"image_url",
    "image_url":{
        "url":f"data:image/jpeg;base64,{kwargs['encoded_img']}"
    }
},
{"type":"text",
"text":"""Now generate trajectories for the following scenario: 

<SCENARIO DESCRIPTION>

You must select the trajectories of the humans and the robot to orchestrate this scenario. You must also assign integer group ids to the HUMANS ONLY involved in the scenario according to the following rule:
1. All the members of the same group must have the same group id.
2. If a human is not involved in a group, they will have a group id of -1.
3. If the scenario does not specifically mention a group, then assign -1 to all humans
4. All humans involved in the scenario must be assigned a group id.
Ensure that you choose paths for the robot and the human accounting for the types of nodes and edges required for the scenario. 
Also select where the human and the robot should ideally encounter each other (INTERACTION POINTS) for the scenario to take place.
Unless a human is supposed to be stationary, ensure that the human's trajectory has at least one node after the interaction node.
Format your output in json as given below:
        {
            'reasoning': <Explain Scenario Location, Robot and human trajectory choice and group id assignment>,
            'trajectories': { <Output the trajectories, interaction points and groupids of the humans and the robot as a sequence of scene graph nodes>
                'robot': <...>,
                'humans':[
                    {   'name': ...,
                        'groupid': ...,
                        'trajectory':...,
                        'interaction_point':...
                    },
                    {   'name': ...,
                        'groupid': ...,
                        'trajectory':...,
                        'interaction_point':...
                    }
                    
                ]}
}"""}]}
        )
        full_prompt[-1]["content"][-1]["text"] = full_prompt[-1]["content"][-1]["text"].replace('<SCENARIO DESCRIPTION>',kwargs['sc_desc'])  
        #print(full_prompt)
        return full_prompt
from .BasePrompt import BasePrompt
class LocationDescriptionQuery(BasePrompt):
    def __init__(self,scene_img) -> None:
        super().__init__()
        self.background = """ """
        self.main_system = """
Write a 5 line description of the scene.
The image shows the nodes and edges that are present in the scene graph. Describe the scene using the scene graph as context to a robot such that it gets an idea of the location it will be navigating in.
A sample explanation is given below:
"The location contains a set of bedrooms and doorways. A passageway connects the bedroom to the living room. There are a series of racks that create passageways in the garage. The hall is an open area and a staircase connects it to the main bedroom". 
"""
        self.imgs = scene_img
    
    def get_full_prompt(self,**kwargs):
        full_prompt = self.main_system
        full_prompt = full_prompt.replace('<SCENE GRAPH>',kwargs['scene_graph'])
        prompt =  dict(
            user = [])
        for img in self.imgs:
            prompt['user'].append({'type':'img',
                                   'content':img,
                                   'detail':'high'})
        prompt['user'].append({
            'type':'text',
            'content':full_prompt})
        return prompt
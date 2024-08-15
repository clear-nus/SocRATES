from utils.llm_models import models
from utils.scene_graph import SceneGraph
from prompts import *
from utils import utils
import xml.etree.ElementTree as ET
import json
import tenacity
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

class QueryHandler:
    def __init__(self,config) -> None:
        self.retry_count = config['retry_count']
        self.model = models[config['model']['model_type']](
            model_name = config['model']['model_name'],
            project_id = config['model']['project_id'],
            debug = config['debug'])
        
        self.debug = config['debug']
    def query_scenario(self,context,task,rough_scenario,location_description):
        '''
        Sends a scenario query to the LLM and returns the response.
        '''
        scQ = ScenarioQuery()
        
        #generate full prompt
        scQ_full_prompt = scQ.get_full_prompt(
            context=context,
            task=task,
            rough_scenario=rough_scenario,
            location = location_description
        )
        print("Querying LLM for scenario")
        try:
            #retry N times
            for attempt in tenacity.Retrying(stop=tenacity.stop_after_attempt(self.retry_count),
                                                wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
                with attempt:
                    scq_response = self.model.get_response(messages = scQ_full_prompt)
                    if self.model.is_response_valid(scq_response,scQ.required_output_keys):
                        scq_response_json = json.loads(scq_response)
                        scq_response_json = {k.lower().replace('_','').replace(' ',''): v for k,v in scq_response_json.items()}
                        #check if 'human_behavior' key is a dict with N keys
                        if isinstance(scq_response_json['humanbehavior'],dict):
                            if len(scq_response_json['humanbehavior'].keys())!=scq_response_json['numberofhumans']:
                                raise ValueError('Invalid response, retrying')
                        
                    else:
                        if self.debug:
                            print("Scenario Response invalid")
                        raise ValueError('Invalid response, retrying')
        except tenacity.RetryError:
            print("Failed to generate scenario")
            return None           
        return scq_response_json
    
    def query_traj(self, scene_graph_json,node_types,edge_types,encoded_img,scenario_description):
        '''
        Queries the LLM for waypoints for human and robot (at once)
        '''    
        #generate full prompt
        with open('prompts/reference/scene_graph.json','r') as f:
            ref_scene_graph_json = json.load(f)
        trQ = TrajectoryQuery(
            ref_img_encoded = utils.encode_image('prompts/reference/scene_graph.png'),
            ref_scene_graph = ref_scene_graph_json,
            qa = parse_questions_answers('prompts/prompt_text/trajectory.txt')
        )
        trq_full_prompt = trQ.get_full_prompt(
            scene_graph = str(scene_graph_json),
            node_types = ','.join(node_types),
            edge_types = ','.join(edge_types),
            encoded_img = encoded_img,
            sc_desc = scenario_description
        )
        scgraph = SceneGraph(scene_graph_json)
        print("Querying LLM for Trajectories")
        valid_trajectories = 0
        all_trajectories_valid = False
        payload = trq_full_prompt.copy()
        
        reply = f"""
                                You made the following mistakes:
                                <ERRORS>
                                Retry and Return the answer in the same JSON format.
                """
        #try:
        for attempt in tenacity.Retrying(stop=(tenacity.stop_after_attempt(self.retry_count) | tenacity.stop_after_delay(15)),
                                            wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
            #retry from scratch
            payload = trq_full_prompt.copy()
            retr_traj = 0
            valid_trajectories = False
            with attempt:
                while retr_traj<3 and not valid_trajectories:
                    trq_response = self.model.get_response(messages = payload)
                    if not self.model.is_response_valid(trq_response,trQ.required_output_keys):
                        if self.debug:
                            print("Inavlid model response")
                        raise ValueError('Invalid model response')
                    all_trajectories_valid = True 
                    trq_response_json = json.loads(trq_response)
                    trq_response_json = {k.lower().replace('_','').replace(' ',''): v for k,v in trq_response_json.items()}
                    groupids =  {k.lower().replace('_','').replace(' ',''): v for k,v in trq_response_json['groupids'].items()}
                    trajectories =  {k.lower().replace('_','').replace(' ',''): v for k,v in trq_response_json['trajectories'].items()}
                    reasoning = trq_response_json['reasoning']
                    #test trajectory correctness
                    for k,v in trajectories.items():
                        traj_valid,errors = scgraph.isvalidtrajectory(v)
                        if not traj_valid: #requery LLM with error message
                            if self.debug:
                                print("Disconnected Trajectory Output, Retrying")
                            all_trajectories_valid = False
                            error_string = ""
                            for err in errors:
                                error_string+=f"There is no edge connecting {err[0]} and {err[1]}!"
                            
                            payload.append(
                                {
                                    "role":"assistant",
                                    "content":[
                                        {
                                            "type":"text",
                                            "text": trq_response
                                        }
                                    ]
                                }    
                                )
                            payload.append({
                                    "role": "user", 
                                    "content": [
                                        {
                                            "type":"text",
                                            "text": reply.replace('<ERRORS>',error_string)
                                        }
                                            ]
                                        }
                                        )
                            retr_traj+=1
                            break
                    if all_trajectories_valid:
                        #check if the robot meets any of the humans:
                        
                        rhmeet = False
                        rt = trajectories['robot'] 
                        for k,ht in trajectories.items():
                            if 'robot' in k:
                                continue
                        
                            if scgraph.areTrajectoriesIntersecting(ht,rt):
                                rhmeet = True
                                break
                        
                        if rhmeet:
                            valid_trajectories = True
                            break
                        else:
                            retr_traj+=1
                            payload.append(
                                    {
                                        "role":"assistant",
                                        "content":[
                                            {
                                                "type":"text",
                                                "text": trq_response
                                            }
                                        ]
                                    }    
                                    )
                            payload.append({
                                    "role": "user", 
                                    "content": [
                                        {
                                            "type":"text",
                                            "text": reply.replace('<ERRORS>',"The Paths of the human and the robot don't intersect, thus they don't meet in this scenario, which is wrong.")
                                        }
                                            ]
                                        }
                                        )
        # except tenacity.RetryError:
        #     print("Failed to generate trajectories")
        #     return None,None
        print(reasoning)
        return groupids,trajectories         
        
    def query_bt(self,behavior_description,node_library):
        '''
        Queries the LLM for a BT given behavior description
        '''
        btQ = BTQuery()
        btq_full_prompt = btQ.get_full_prompt(behavior = behavior_description)            
        payload = btq_full_prompt.copy()
        try:
            for attempt in tenacity.Retrying(stop=(tenacity.stop_after_attempt(self.retry_count) | tenacity.stop_after_delay(15)),
                                            wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
                retr_bt = 0
                valid_bt = False
                with attempt:
                    while retr_bt<3 and not valid_bt:
                        print(f'Querying for BT')
                        try:           
                            btq_response = self.model.get_response(messages = payload)
                        except Exception as e:
                            print(e)
                            retr_bt+=1
                            raise Exception
                        if not self.model.is_response_valid(btq_response,btQ.required_output_keys):
                            raise ValueError('Invalid model response')
                        btq_response_json = json.loads(btq_response)
                        btq_response_json = {k.lower().replace('_','').replace(' ',''): v for k,v in btq_response_json.items()}
                        try:
                            test_xml = ET.fromstring(btq_response_json['tree'])  
                            print("Recieved Valid XML")
                        except:
                            print("Recieved Invalid XML")
                            retr_bt+=1
                            payload.append({
                                "role":"assistant",
                                "content":[
                                    {
                                        "type":"text",
                                        "text": btq_response
                                    }
                                ]
                            })
                            payload.append({
                                "role":"user",
                                "content":[
                                    {
                                        "type":"text",
                                        "text": "Your output is not a valid XML tree. It is not parseable with python's xml.etree.ElementTree.fromstring function. Please try again."
                                    }
                                ]
                            })
                            continue
                        
                        if utils.validate_bt(test_xml,node_library):
                            print("Recieved Valid BT")
                            return btq_response_json
                        else:
                            print("Recieved Invalid BT")
                            retr_bt+=1
                            payload.append({
                                "role":"assistant",
                                "content":[
                                    {
                                        "type":"text",
                                        "text": btq_response
                                    }
                                ]
                            })
                            payload.append({
                                "role":"user",
                                "content":[
                                    {
                                        "type":"text",
                                        "text": " Please try again."
                                    }
                                ]
                            })    
        except tenacity.RetryError:
            print("Failed to generate BTs")
            return None

    
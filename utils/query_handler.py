from utils.llm_models import models
from utils.scene_graph import SceneGraph
from prompts import *
from utils import utils
from utils.utils import *
import xml.etree.ElementTree as ET
import json
import tenacity

class QueryHandler:
    def __init__(self,config) -> None:
        self.retry_count = config['retry_count']
        self.model = models[config['model']['model_type']](
            model_name = config['model']['model_name'],
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
        lprint("Querying LLM for scenario")
        try:
            #retry N times
            for attempt in tenacity.Retrying(stop=tenacity.stop_after_attempt(self.retry_count),
                                                wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
                with attempt:
                    scq_response = self.model.get_response(messages = scQ_full_prompt,response_format= StructuredScenarioResponse)
                    scq_response_structured = self.model.extract_response(scq_response)
                    if scq_response_structured:
                        #check if 'human_behavior' key is a dict with N keys
                        if len(scq_response_structured.humanbehavior)!=scq_response_structured.numberofhumans:
                            raise ValueError('Invalid response, retrying')
                    else:
                        if self.debug:
                            eprint("Scenario Response invalid")
                        raise ValueError('Invalid response, retrying')
        except tenacity.RetryError:
            eprint("Failed to generate scenario")
            return None           
        return scq_response_structured
    
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
        lprint("Querying LLM for Trajectories")
        valid_trajectories = 0
        all_trajectories_valid = False
        payload = trq_full_prompt.copy()
        
        reply = f"""
                                You made the following mistakes:
                                <ERRORS>
                                Retry and Return the answer in the same JSON format.
                """
        for attempt in tenacity.Retrying(stop=(tenacity.stop_after_attempt(self.retry_count) | tenacity.stop_after_delay(15)),
                                            wait=tenacity.wait_random_exponential(multiplier=1,max=40)):
            #retry from scratch
            payload = trq_full_prompt.copy()
            retr_traj = 0
            valid_trajectories = False
            with attempt:
                while retr_traj<5 and not valid_trajectories:
                    trq_response = self.model.get_response(messages = payload,response_format=StructuredTrajResponse)
                    trq_response_structured = self.model.extract_response(trq_response)
                    if not trq_response_structured:
                        if self.debug:
                            eprint("Inavlid model response")
                        raise ValueError('Invalid model response')
                    
                    all_trajectories_valid = True 
                    trajectories =  trq_response_structured.trajectories
                    robot_traj = trajectories.robot
                    human_traj = trajectories.humans
                    all_human_traj = [h.trajectory for h in human_traj]
                    reasoning = trq_response_structured.reasoning
                    #test trajectory correctness
                    for v in [robot_traj] + all_human_traj:
                        traj_valid,errors = scgraph.isvalidtrajectory(v)
                        if not traj_valid: #requery LLM with error message
                            all_trajectories_valid = False
                            error_string = ""
                            for err in errors:
                                error_string+=f"There is no edge connecting {err[0]} and {err[1]}!\n"

                    if not all_trajectories_valid:                        
                        if self.debug:
                                eprint("Disconnected Trajectory Output, Retrying")    
                        payload.append(
                            {
                                "role":"assistant",
                                "content":[
                                    {
                                        "type":"text",
                                        "text": trq_response.json()
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
                        
                    else:
                        #check if the robot meets any of the humans:
                        rhmeet = True
                        # for traj in all_human_traj: #every human should encounter the robot in some node
                        #     if not scgraph.areTrajectoriesIntersecting(robot_traj,traj):
                        #         rhmeet = False
                        #         if self.debug: 
                        #             eprint("Non intersecting trajectories, Retrying..")
                        #         break
                        for human in human_traj:
                            if not((human.interaction_point in human.trajectory) and (human.interaction_point in robot_traj)):
                                rhmeet = False
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
                                                "text": trq_response.json()
                                            }
                                        ]
                                    }    
                                    )
                            payload.append({
                                    "role": "user", 
                                    "content": [
                                        {
                                            "type":"text",
                                            "text": reply.replace('<ERRORS>',"The Paths of the human and the robot don't intersect or the interaction point is set incorrectly.")
                                        }
                                            ]
                                        }
                                        )
        if valid_trajectories:
            if self.debug:
                lprint("Reasoning: ")
                print(reasoning)
            return trajectories         
        else:
            return None
        
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
                        lprint(f'Querying for BT')
                        try:           
                            btq_response = self.model.get_response(messages = payload,response_format=StructuredBTResponse)
                        except Exception as e:
                            eprint(e)
                            retr_bt+=1
                            raise Exception
                        btq_response_structured = self.model.extract_response(btq_response)
                        if not btq_response_structured:
                            raise ValueError('Invalid model response')
                        try:
                            test_xml = ET.fromstring(btq_response_structured.tree)  
                            lprint("Recieved Valid XML")
                        except:
                            eprint("Recieved Invalid XML")
                            retr_bt+=1
                            payload.append({
                                "role":"assistant",
                                "content":[
                                    {
                                        "type":"text",
                                        "text": btq_response.choices[0].message.to_json()
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
                            lprint("Recieved Valid BT")
                            return btq_response_structured
                        else:
                            eprint("Recieved Invalid BT")
                            retr_bt+=1
                            payload.append({
                                "role":"assistant",
                                "content":[
                                    {
                                        "type":"text",
                                        "text": btq_response.choices[0].message.to_json()
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
        except tenacity.RetryError as e:
            eprint(f"Failed to generate BTs + {e}")
            return None

    
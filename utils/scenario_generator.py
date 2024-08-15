from utils.query_handler import QueryHandler
import os
from utils import utils
import json,yaml,copy
from utils.scene_graph import SceneGraph
import pprint
import tenacity
class ScenarioGenerator:
    '''
        Helper class for generating scenarios.
    '''
    def __init__(self,config):
        self.qh = QueryHandler(config)
        self.config = config
        self.debug = config['debug']
        with open(self.config['location']['scene_graph_file'],'r') as f:
            self.scene_graph_json = json.load(f)
            
        self.scgraph = SceneGraph(self.scene_graph_json)
        self.encoded_map_img = utils.encode_image(self.config['location']['annotated_map_img_file'])
        self.node_library = self.config['node_library']
        
        self.node_types = []
        self.edge_types = []
        for node in self.scgraph.get_parent_nodes():
            if self.scgraph.graph.nodes[node]['type'] not in self.node_types:
                self.node_types.append(self.scgraph.graph.nodes[node]['type']) 
        for edge in self.scgraph.graph.edges:
            if self.scgraph.graph.edges[edge]['type'] not in self.edge_types:
                self.edge_types.append(self.scgraph.graph.edges[edge]['type'])     
        
        with open(self.config['location']['map_params_file'],'r') as f:
            self.map_params = yaml.safe_load(f)
    
    def get_scenario_desc(self):
        if self.config['use_handcrafted_scenario']:
            return self.config['handcrafted_scenario']['scenario']
        else:
            return self.qh.query_scenario(
                context = self.config['context'],
                task = self.config['task'],
                rough_scenario = self.config['rough_scenario'],
                location_description = self.config['location']['description']
            )
  
    def get_trajectories(self,scenario_desc):
        if self.config['use_handcrafted_scenario']:
            return self.config['handcrafted_scenario']['groupids'],self.config['handcrafted_scenario']['trajectories']
        
        else:
            return self.qh.query_traj(
            scene_graph_json=self.scene_graph_json,
            node_types=self.node_types,
            edge_types=self.edge_types,
            encoded_img=self.encoded_map_img,
            scenario_description=scenario_desc
        )       
    
    def generate_scenario(self):
        '''
            Generates a scenario with hunavsim components using the query handler
        '''
        try:
            for attempt in tenacity.Retrying(stop=tenacity.stop_after_attempt(3)):
                with attempt:
                    if self.config['load_scenario_response']:
                        #LOAD SCENARIO
                        print('loading saved scenario proposal')
                        with open(self.config['scenario_file'],'r') as f:
                            scenario = json.load(f)
                        scenario_desc = scenario['scenariodescription']
                        behav_desc = scenario['humanbehavior']    
                    else:
                        #GENERATE SCENARIO
                        while True:
                            scenario = self.get_scenario_desc()
                            if scenario == None:
                                raise Exception
                            scenario_desc = scenario['scenariodescription']
                            behav_desc =  {k.lower().replace('_','').replace(' ',''): v for k,v in scenario['humanbehavior'].items()}
                            print("GENERATED SCENARIO:")
                            pprint.pprint(scenario)
                            
                            continue_choice = False
                            while True:
                                print("CONTINUE?:(Y/N)")
                                continue_choice = input().lower().strip()
                                if continue_choice!='y' and continue_choice!='n':
                                    print("invalid input")
                                    continue
                                else:
                                    if continue_choice=='y':
                                        continue_choice = True
                                    else:
                                        continue_choice = False                           
                                break
                            if continue_choice:
                            #reasoning = scenario['reasoning']    
                                with open(self.config['scenario_file'],'w') as f:
                                    json.dump(scenario,f)
                                break
                            
                    #LOAD
                    if self.config['load_trajectory_response']:
                        #LOAD TRAJECTORY
                        print('loading saved trajectory')
                        with open(self.config['trajectory_file'],'r') as f:
                            traj = json.load(f)
                        groupids = traj['groupids']
                        trajectories = traj['trajectories']
                        
                    else:
                        while True:
                            #GENERATE TRAJECTORY
                            groupids,trajectories = self.get_trajectories(scenario_desc)
                            if groupids == None or trajectories == None:
                                raise Exception
                            
                            print("GENERATED TRAJECTORIES:")
                            print(trajectories)
                            print("GROUP IDs:")
                            print(groupids)
                            
                            continue_choice = False
                            while True:
                                print("CONTINUE?:(Y/N)")
                                continue_choice = input().lower().strip()
                                if continue_choice!='y' and continue_choice!='n':
                                    print("invalid input")
                                    continue
                                else:
                                    if continue_choice=='y':
                                        continue_choice = True
                                    else:
                                        continue_choice = False                           
                                    break
                                        
                            if continue_choice:
                                #reasoning = scenario['reasoning']    
                                with open(self.config['trajectory_file'],'w') as f:
                                    json.dump({
                                        'groupids':groupids,
                                        'trajectories':trajectories
                                        },f)
                                break
                    
                    if self.config['load_bt_response']:
                        #LOAD BT
                        print('loading saved behaviors')
                        with open(self.config['bt_file'],'r') as f:
                            behavior_trees = json.load(f)
                    else:
                        while True:
                            #GENERATE BT
                            print("Generating Behavior Trees")
                            behavior_trees = {}
                            for human,behav in behav_desc.items():
                                print(f"Generating Tree for {human}...")
                                behavior_response =  self.qh.query_bt(
                                    behavior_description=behav,
                                    node_library=self.node_library
                                )
                                if behavior_response == None:
                                    if self.debug:
                                        raise Exception
                                behavior_trees[human] = behavior_response['tree']
                                print(f"Generated Behavior Tree for {human}")
                                
                                #continue_choice = False
                                # while True:
                                #     print("CONTINUE?:(Y/N)")
                                #     continue_choice = input().lower().strip()
                                #     if continue_choice!='y' and continue_choice!='n':
                                #         print("invalid input")
                                #         continue
                                #     else:
                                #         if continue_choice=='y':
                                #             continue_choice = True
                                #         else:
                                #             continue_choice = False                           
                                #         break
                                
                            with open(self.config['bt_file'],'w') as f:
                                json.dump(behavior_trees,f)
                            break
                    
                    #save generated scenario
                    #if not self.config['load_scenario_response'] and not self.config['load_trajectory_response'] and not self.config['load_bt_response']:
                    with open(os.path.join(self.config['paths']['save_dir'],self.config['experiment_name']+'_'+'response_traj.json'),'w') as f:
                        json.dump({
                            'scenario':scenario,
                            'groupids':groupids,
                            'trajectories':trajectories,
                            'behavior_trees':behavior_trees                      
                            },f)
        except tenacity.RetryError:
            print("Unable to generate scenario, please rerun script")
            exit()
        return scenario, groupids, trajectories, behavior_trees

    def instantiate_simulator(self,file_paths,groupids,trajectories,behaviors_trees):
        '''
        Instantiates hunavsim with trajectories and behaviors   
    
        '''
        # write trajectories
        agents_yaml = {'hunav_loader': {'ros__parameters': {'map': self.config['location']['name'],
                        'publish_people': True,
                        'agents': []}}}
        blank_human = {'id': None,
            'skin': 0,
            'behavior': 0,
            'group_id': -1,
            'max_vel': 1.5,
            'radius': 0.4,
            'init_pose': {'x': None, 'y': None, 'z': 1.25, 'h': 0.0},
            'goal_radius': 0.3,
            'cyclic_goals': False,
            'goals': [],
            }
        agents = {} 
        # transform from scene graph to world trajectories
        world_trajectories = {}        
        for k,v in trajectories.items():
            world_trajectories[k] = []
            for l in v:
                world_trajectories[k].append(self.pix2world(self.scgraph.graph.nodes[l]['pos']))
        
        #gather all peds trajectories
        for i in range(len(trajectories.keys())-1):
            agents_yaml['hunav_loader']['ros__parameters']['agents'].append(f'agent{i}')
            agents[f'agent{i}'] = copy.deepcopy(blank_human)
            agents[f'agent{i}']['id'] = i
            agents[f'agent{i}']['behavior'] = 7+i
            agents[f'agent{i}']['group_id'] = groupids[f'human{i+1}']
            for j,g in enumerate(world_trajectories[f'human{i+1}']):
                if j == 0:
                    agents[f'agent{i}']['init_pose'] = {
                        'x':g[0],
                        'y':g[1],
                        'z':1.25,
                        'h':0.0,
                    }
                    if len(world_trajectories[f'human{i+1}']) <= 1:
                        agents[f'agent{i}']['goals'].append(f'g{1}')
                        agents[f'agent{i}'][f'g{1}'] = {
                            'x':g[0],
                            'y':g[1],
                            'h':1.25
                        }
                else:
                    agents[f'agent{i}']['goals'].append(f'g{j}')
                    agents[f'agent{i}'][f'g{j}'] = {
                        'x':g[0],
                        'y':g[1],
                        'h':1.25
                    }
        agents_yaml['hunav_loader']['ros__parameters'].update(agents)
        
        #write trajectories in hunav_sim        
        with open(os.path.join(file_paths['hunav_sim_dir'],file_paths['agents_file']),'w') as f:
            yaml.dump(agents_yaml,f)

        #write waypoints for the robot
        robot_poses = {
            'initial_pose':{},
            'waypoints':[]
        }
        for j,g in enumerate(world_trajectories['robot']):
                if j == 0:
                    robot_poses['initial_pose'] = {
                        'x':g[0],
                        'y':g[1],
                        'yaw':0.0,
                    }
                else:
                    robot_poses['waypoints'].append({
                        'position':{
                        'x':g[0],
                        'y':g[1],
                        'yaw':0.0
                        }
                    })
        
        with open(os.path.join(file_paths['hunav_gazebo_wrapper_dir'],'config','robot_poses.yaml'),'w') as f:
            yaml.dump(robot_poses,f)
        
        #write behavior files for hunavsim
        i = 0
        for human, behav in behaviors_trees.items():
            with open(os.path.join(file_paths['hunav_sim_dir'],file_paths['bt_dir'],f'LLMBT_{i}.xml'),'w') as f:
                f.write(behav)
            i+=1
        print("Wrote trajectories and behaviors to simulation files")

    def pix2world(self,px):
        return [
            px[0]*self.map_params['resolution'] + self.map_params['origin'][0],
            -1.0*(px[1]*self.map_params['resolution'] + self.map_params['origin'][1])
        ]
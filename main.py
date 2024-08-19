from utils.scenario_generator import ScenarioGenerator
from omegaconf import OmegaConf
if __name__ == '__main__':
    #import the configs from config.yaml and inputs.yaml    
    config = OmegaConf.load('config.yaml')    
    inputs = OmegaConf.load('inputs.yaml')
    file_paths = OmegaConf.merge(config['hunav_sim'],inputs['paths'])
    full_conf = OmegaConf.merge(config,inputs)
    
    sg = ScenarioGenerator(full_conf)    
    scenario, groupids, trajectories, interaction_points, behavior_trees = sg.generate_scenario()
    sg.instantiate_simulator(
        file_paths= file_paths,
        groupids= groupids,
        trajectories= trajectories,
        behaviors_trees= behavior_trees,
    )
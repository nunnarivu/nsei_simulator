import json 
import random
import re
import os
from copy import deepcopy
import numpy as np
# from .scene_graph import SceneGraph
# from .program_engine import ProgramEngine
from  dataset.nsrm.configs import ParameterSettings as settings
from .base import DatasetConstructBase



class ProgramGenerator(DatasetConstructBase):
    def __init__(self, simulator_handle, configs:dict):
        super().__init__( simulator_handle, configs)

    def generate_random_scene(self, ):
        object_counts = self.configs['num_objects_per_type']
        
        #choose the random posistions for the objects
        block_positions = self.dataset_world.get_block_positions(sum(object_counts.values()))
        assert block_positions is not None, "Could not find valid positions for the objects. Try increasing the table size or decreasing the number of objects"
       
        #choose the orientations based on the configs
        if self.configs['rotation']:
            raise NotImplementedError
            self.obj_ornientations = self.dataset_world.get_random_orientations(sum(object_counts.values()))
        else:
            self.obj_ornientations = [[0,0,0,1]] * sum(object_counts.values())
        
        #add the objects to the world    
        for obj_type, count in object_counts.items():
            if obj_type not in ['cube', 'dice', 'lego']:
                raise ValueError(f"Object type {obj_type} is not supported")
            if count > len(settings.COLOR):
                raise ValueError(f"Object count {count} exceeds available colors for {obj_type}")
            color = list(settings.COLOR.keys())[:count]
            for i in range(count):
                position = block_positions.pop(0)
                model_path = os.path.join(self.simulator_handle.local_assets_path, settings.OBJECT_URDF_PATHS[f'{color[i]}_{obj_type}']) if obj_type == 'dice' else settings.OBJECT_URDF_PATHS[obj_type]
                obj = self.simulator_handle.world.add(name = f"{obj_type}_{color[i]}_{i}",
                                                             pos = position,
                                                             ori = self.obj_ornientations[i], 
                                                             obj_type = 'object', 
                                                             model_path = model_path,
                                                             )
            
                obj.change_color(settings.COLOR.get(color[i]))
                breakpoint()
        self.simulator_handle.step_simulation(num_steps=100)
        
        if self.dataset_world.check_collision():
            self.dataset_world.reset_world()
            self.generate_random_scene()

            
            
        
    def select_template(self, template_file:str):
        with open(template_file, 'r') as file:
            self.template = random.choice(json.load(file))

    def load_metadata(self, metadata_file:str):
        with open(metadata_file, 'r') as file:
            self.metadata = json.load(file)
        
    def get_substitute(self, ):
        pass
    
    def generate_instruction(self, ):
        pass
    
    def generate_grounded_functional_program(self, ):
        pass
    
    def get_program(self, ):
        pass
    
    def save_demonstration_info(self, ):
        pass
    
    def save_question_info(self, ):
        pass
    
    def cache_current_state(self, ):
        pass
    
    def reset_state(self, state):
        pass
    
    
    
    
    
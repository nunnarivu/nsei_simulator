import os
import argparse
import random

import numpy as np

import dataset.nsrm.configs as configs
from dataset.nsrm.configs import ParameterSettings as settings
from dataset.nsrm.construct.program_generator import ProgramGenerator



def construct_main(template_file:str, metadata_file:str, dataset_dir:str):
    """
    dataset_dir: Relative path to the Dataset Directory
    template_file: str, template file for generating program and instructions
    metadata_file: str, metadata file for possible concepts and synonyms
    """
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir)
    
    objects = configs.required_dataset_configs['required_object_types']
    object_counts = configs.required_dataset_configs['num_objects_per_type']
    
    if object_counts is None:
        while True:
            object_counts = {}
            for obj in objects:
                object_counts[obj.lower()] = 0
            for i in range(configs.required_dataset_configs['MAX_OBJECTS']):
                object_counts[random.choice(objects).lower()] += 1
            
            flag = True
            for key in object_counts:
                if object_counts[key] >= len(settings.COLOR):
                    object_counts[key] = len(settings.COLOR)
                    flag = False    
                    break
            if flag:
                break
    
    configs.required_dataset_configs['num_objects_per_type'] = object_counts
    
    Generator = DataConstructor(configs.simulator_configs, configs.required_dataset_configs, configs.additional_dataset_configs)
    Generator.construct_data(dataset_dir, template_file, metadata_file)
    Generator.kill()
        

def init_simulator_handle(sim_name:str, simulator_configs:dict):
    """
    Initialize the simulator handle based on the simulator name and configurations
    """
    if sim_name == 'bullet':
        from envs.bullet.simulate import SimulatorInterface
        return SimulatorInterface(simulator_configs)
    else:
        raise ValueError(f"Simulator {sim_name} is not supported")
    


class DataConstructor(object):
    def __init__(self, simulator_configs:dict, required_dataset_configs:dict, additional_dataset_configs:dict):
        
        self.construct_handle = ProgramGenerator(simulator_handle = init_simulator_handle('bullet', simulator_configs), 
                                                 configs = simulator_configs | required_dataset_configs | additional_dataset_configs)
        
        self.required_dataset_configs = required_dataset_configs
        self.additional_dataset_configs = additional_dataset_configs
        self.simulator_configs = simulator_configs
        
    def construct_data(self, dataset_dir:str, template_file:str, metadata_file:str):
        # self.construct_handle.load_metadata(metadata_file)
        self.construct_handle.select_template(template_file)
        
        #generate a random scene based on given configs and template
        self.construct_handle.dataset_world.reset_world()
        self.construct_handle.generate_random_scene()
        self.construct_handle.save_checkpoint(state_tag = "initial_state", dump_to_file=True, root_folder=dataset_dir)


        previous_programs = set()
        for i in range(self.required_dataset_configs['num_instruction_per_scene']):
           
            #create new dir for saving the sample
            sample_no = len(os.listdir(dataset_dir)) 
            smpl_dir = os.path.join(dataset_dir, "{0:0=4d}".format(sample_no))
            os.mkdir(smpl_dir)
            
            breakpoint()
            #reset the generator state to the cached state
            self.construct_handle.simulator_handle.reset()
            self.construct_handle.load_checkpoint(from_file = os.path.join(dataset_dir, "initial_state.pkl"))
            
            self.construct_handle.save_instance(smpl_dir) #save the initial scene S00
            
            status = self.construct_handle.generate_grounded_functional_program(object_choice = self.additional_dataset_configs.get('instantiation_type', 'random'),
                                                                                 max_attempts = self.additional_dataset_configs['max_program_generation_atempts'],)
            if status == False:
                os.system(f"rm -rf {smpl_dir}")
                print(f"Failed to generate a program for sample {sample_no}. Skipping this sample.")
                continue
            elif self.construct_handle.get_program() in previous_programs:
                print(f"Program already exists: {self.construct_handle.get_program()}. Skipping this sample.")
                os.system(f"rm -rf {smpl_dir}")
                continue
            
            program, command_lexed, command, language_complexity = self.construct_handle.generate_instruction(complexity = self.additional_dataset_configs.get('complexity', None))
            self.construct_handle.save_demonstration_info(command_lexed, command, language_complexity, program)
            previous_programs.add(self.construct_handle.get_program())   
            
    def kill(self):
        self.construct_handle.simulator_handle.disconnect()
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Construct dataset for the NSRM task')
    parser.add_argument('--dataset_dir', metavar='DIR', help='Relative path to the Dataset Directory')
    parser.add_argument('--template_file', required = True, type = str, help = "template file for generating program and instructions")
    parser.add_argument('--metadata_file', type = str, required = True, help = "metadata file for possible concepts and synonyms")
    parser.add_argument('--type', type=str, required=True)
    parser.add_argument('--language', type=str, required=True)
    parser.add_argument('--max_objects', type=int, required=True)
    parser.add_argument('--num_examples', type=int, required=True)
    parser.add_argument('--record', type = str, default = None, help = 'Path to record the video')
    args = parser.parse_args()

    category = {
        'type': args.type,
        'num_objects': args.max_objects,
        'language': args.language
    }
    
    if category['type'] == 'any':
        required_object_types = ['cube', 'lego', 'dice']
    elif category['type'] == 'cube':
        required_object_types = ['cube', "dice"]
    
    CommandLineConfigs = {
      'MAX_OBJECTS': category["num_objects"],
      'required_object_types': required_object_types,
      'rotation': False,
      'num_objects_per_type' : None
    }
    
    for key, value in CommandLineConfigs.items():
        configs.required_dataset_configs[key] = value
    if category['language'] == 'simple':
        configs.additional_dataset_configs['complexity'] = 'simple'
    else:
        configs.additional_dataset_configs['complexity'] = random.choice(['complex', 'compound'])
    
    if args.record is not None:
        configs.simulator_configs['record'][0] = args.record
        
    for i in range(args.num_examples):
        construct_main(args.template_file, args.metadata_file, args.dataset_dir)
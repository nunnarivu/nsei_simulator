import json 
import random
import re
import os
from copy import deepcopy
import numpy as np
from .scene_graph import SceneGraph
# from .program_engine import ProgramEngine
from  dataset.nsrm.configs import ParameterSettings as settings
from .base import DatasetConstructBase



class ProgramGenerator(DatasetConstructBase):
    def __init__(self, simulator_handle, configs:dict):
        super().__init__( simulator_handle, configs)
        self.cached_state = []

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
                                                             other_properties = dict(type = obj_type, color = color[i])
                                                             )
            
                if obj_type != 'dice':
                    obj.change_color(settings.COLOR.get(color[i]))

        self.simulator_handle.step_simulation(num_steps=100)
        
        if self.dataset_world.check_collision():
            self.dataset_world.reset_world()
            self.generate_random_scene()

    def select_template(self, template_file:str):
        self.template_file = template_file
        with open(template_file, 'r') as file:
            self.template = random.choice(json.load(file))

    def load_metadata(self, metadata_file:str):
        with open(metadata_file, 'r') as file:
            self.metadata = json.load(file)

    def get_substitute(self, char_token, index):
        if char_token == 'T':
            return self.objects[self.unique_objects[index -1 ]].type
        elif char_token == 'C':
            return self.objects[self.unique_objects[index -1 ]].color
        elif char_token == 'A':
            return self.unique_actions[index-1]
        elif char_token == 'R':
            return self.relations[index -1]
        else:
            raise ValueError("Unknown char token {}".format(char_token))
    
    def generate_instruction(self, complexity = None):
        '''
        Description: The instructions are read from the template and the <A>, <C>, <O> tokens are replaced with the appropriate concept words.
        Inputs: complexity(str): options = ['simple', 'complex', 'compound']. Please pass this in the config dict. If None, one of the options is randomly choosen.

        Output: program:(list of 3-tuples)
                instruction:(string)
        '''
        complexity = random.choice(['simple','complex','compound']) if complexity is None else complexity
        sent_lexed = random.choice(self.template['text'][complexity])
        words = sent_lexed.split()
        
        for idx,w in enumerate(words):
            match = re.search("<(\w)(\d+)>",w)
            if match:
                char_token = match.group(1)
                if char_token == "T":
                    substitute = self.get_substitute('T',int(match.group(2)))
                    substitute = substitute.lower()
                    if substitute == 'lego':
                        substitute += ' block'
                    words[idx] = substitute 
                elif char_token == 'C':
                    substitute = self.get_substitute('C', int(match.group(2)))
                    substitute = substitute.lower()
                    words[idx] = substitute 
                elif char_token == 'A':
                    substitute = self.get_substitute('A', int(match.group(2)))
                    substitute = substitute.lower()
                    words[idx] = substitute+'_a'
                elif char_token == 'R':
                    substitute = self.get_substitute('R', int(match.group(2)))
                    substitute = substitute.lower()
                    words[idx] = substitute
                else:
                    raise ValueError("Unknown char token {}".format(char_token))
        #At this point, the instruction will be created. But we replace some of the words with thier synonyms from the metadata. 
        sent = ' '.join(words)
        words = sent.split()
        synon = self.metadata['synonyms']
        for idx,w in enumerate(words):
            w = w.lower()
            if w in synon.keys():
                replace_text = random.choice(synon[w])
                words[idx] = replace_text
        
        return self.get_program(), sent_lexed, ' '.join(words), complexity
    
    def generate_grounded_functional_program(self, object_choice = 'default', max_attempts = 10000):
        template = self.template
        is_relational = template.get("relational", False)
        all_objects = self.simulator_handle.world.find(type='object')
        num_objects = len(all_objects)
        try:
            assert num_objects >= template['num_unique_objects']
        except AssertionError:
            print( " The number of  unique objects to be intialized is less than the no of objects in the world")
            return False
        for i in range(max_attempts):
            #choose actions and objects instances
            possible_actions = self.metadata['actions']['move'] if len(template.get('constraints')["actions"]) == 0 else template.get('constraints')["actions"]
            self.unique_actions = random.sample(possible_actions,template['num_unique_actions']) if "action_preference" not in template.get("constraints") else template.get("constraints")["action_preference"]
            num_uncons_objs = template['num_unique_objects'] - len(template.get('constraints').get("objects",[]))
            num_cons_objs = template['num_unique_objects'] - num_uncons_objs

            #select unconstrained objects
            self.unique_objects = [i for i in range(num_uncons_objs)] if object_choice == 'default' else random.sample([i for i in range(num_objects)], num_uncons_objs)

            #select constrained objects
            for constraint in template['constraints'].get("objects", []):
                possible_cons_objs = [i for i in range(num_objects) if getattr(all_objects[i], constraint[1]) == constraint[2]]
                if len(possible_cons_objs) == 0:
                    return False
                self.unique_objects.insert(constraint[0], random.choice(possible_cons_objs))
            
            #recheck if objects are unique
            if len(set(self.unique_objects)) != len(self.unique_objects):
                continue
            
            if is_relational:
                all_relations = self.configs['relations']
                self.relations = [random.choice(all_relations) for i in range(template['num_relations'])]
            self.symbolic_program = deepcopy(self.template['nodes'])
            self.scene_graph = SceneGraph(all_objects ,self.configs)
            self.program = list()
            for node in self.symbolic_program:
                # Replace the tokens by the respective concept words
                for idx,str in enumerate(node['value_inputs']):
                    match = re.search("<(\w)(\d)>", str)
                    if match:
                        node['value_inputs'][idx] = self.get_substitute(match.group(1),int(match.group(2)))
                    else:
                        breakpoint()
                        raise NotImplementedError
            
            # Try Executing the program on the scene
            status = self.executor.execute_symbolic_program()
            if status == True:
                print("##### Found one program ###### ")
                print("\n...........Requesting PyBullet to perform the Simulation...............")
                block_positions = [self.objects[i].pos for i in range(len(self.objects))]
                _, self.plan = self.check_action_compatibility(self.get_program(), block_positions)
                assert self.plan is not None , "There is a bug in program generation"
                self.execute_plan(self.plan, use_robot = False, save_keyframes=True)
                return True

        if len(self.program) == 0:
            print("*******NO compatible program found *******")
            return False
        
    def get_program(self, ):
        return self.program
    
    def save_demonstration_info(self, command_lexed, command, complexity, program):
        info = dict()
        info['instruction'] = command
        info['instruction_lexed'] = command_lexed
        info['template_json_filename'] = self.template_file
        info['template_id'] = self.template['template_id']
        info['language_complexity'] = complexity
        info['grounded_program'] = program
        info['program'] = self.symbolic_program
        info['scene_info'] = self.get_scene_info()
        info['transition_info'] = self.get_transition_info()
        with open(f"{self.default_save_folder}/demo.json", 'w') as write_file:
            json.dump(info, write_file)
    
    def save_checkpoint(self, state_tag:str = None, dump_to_file:bool = False, root_folder:str = None):
        tag, checkpoint_dict = self.simulator_handle.save_checkpoint(state_tag, dump_to_file, root_folder)
        self.cached_state.append(dict(tag = tag, checkpoint = checkpoint_dict))

    def load_checkpoint(self, state_tag = None, from_file = None):
        self.simulator_handle.load_checkpoint(state_tag, from_file)
        
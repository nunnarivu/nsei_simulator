import json
import random, copy
import os
import re
from dataset.spg.construct.base import SPGConstructBase
from dataset.spg.configs import ParameterSettings as settings
from dataset.spg.construct.templates.program_lib import PROGRAM_LIB


class ProgramGenerator(SPGConstructBase):
    def __init__(self, simulator_handle, configs):
        super().__init__(simulator_handle, configs)

    def select_template(self, template_file:str):
        with open(template_file, 'r') as f:
            self.template = random.choice(json.load(f))

    def load_metadata(self, metadata_file:str):
        with open(metadata_file, 'r') as f:
            self.metadata = json.load(f)
    
    def _get_param_value(self, type):
        if type == 'int':
            return random.randint(settings.MIN_INT_VALUE, settings.MAX_INT_VALUE)
        elif type == '(Color, Type)':
            color = random.choice(list(settings.COLOR.keys()))
            obj_type = random.choice(self.configs['required_object_types'])
            return (color, obj_type)
        else:
            raise ValueError(f"Unsupported parameter type: {type}")

    def initialize_template(self,):
        template = copy.deepcopy(self.template)
        initializations = {}
        eval_expressions = []
        instruction = self.template['instruction']
        # find strings enclosed within __ and __
        matches = re.findall(r"__\w+__(?:\.\w+)*", instruction)
        for match in matches:
            if match == '__IC__':
                initializations[match] = random.choice(list(PROGRAM_LIB.keys()))
            elif match.startswith('__param'):
                if len(match.split('.')) > 1:
                    param_no, param_attr = match.split('.')
                    param_no = param_no[2:-2].split('_')[1] #exclude the __ and __
                    param_attr = 0 if param_attr == 'NAME' else 1 if param_attr == 'TYPE' else None
                    eval_expressions.append(match)
                    initializations[match] = f"PROGRAM_LIB[initializations['__IC__']][{param_no}][{param_attr}]"
                else:
                    param_no, param_attr = match[2:-2], None
                    initializations[match] = self._get_param_value(self.template['parameters'][param_no])
            else:
                raise ValueError(f"Unsupported placeholder: {match}")
        for expr in eval_expressions:
            initializations[expr] = eval(initializations[expr])
        self.initializations = initializations
        return initializations
    
    def generate_instruction(self,):
            instruction = self.template['instruction']
            for place_holder, value in self.initializations.items():
                if place_holder in instruction:
                    instruction = instruction.replace('<'+place_holder+'>', str(value)) # replace the direct place holders enclosed with in < >
            
            #Now evaluate the expressions enclosed within { }
            instruction_grounded = eval("f'"+f"{instruction}"+"'", {}, self.initializations)
            return self.template['instruction'], instruction_grounded

    def generate_random_scene(self, ):
        raise NotImplementedError("This method should be implemented by subclasses.")
    
    def generate_initial_scene(self, ):
        if not "constraints" in self.template:
            self.generate_random_scene()
            return
        
        constraint_objs = []
        unconstraint_objs = []
        constraints = self.template['constraints']
        for cons in constraints:
            for key,value in cons.items():
                for place_holder in self.initializations.keys():
                    if place_holder in str(value):
                        cons[key] = eval(f"{value.replace(place_holder, str(self.initializations[place_holder]))}", {}, self.initializations)
        
            constr_type = cons['cons_type']
            if constr_type == 'multiple_objects':
                num_objects = cons['num_objects']
                for _ in range(num_objects):
                    constraint_objs.append((cons['color'], cons['type']))
            else:
                raise ValueError(f"Unsupported constraint type: {cons['cons_type']}")

        for _ in range(self.configs['num_distractors']):
            color = random.choice(list(settings.COLOR.keys()))
            obj_type = random.choice(self.configs['required_object_types'])
            unconstraint_objs.append((color, obj_type))
        
        all_objs = constraint_objs + unconstraint_objs
        
        program = self.template['program']
        for key, value in self.initializations.items():
            if key in program:
                program = program.replace(key, str(value))
        status, structure_positions, _ = self.execute_program(program, dry_run = True)
        block_positions = self.dataset_world.get_block_positions(num_blocks=len(all_objs), avoid_positions=structure_positions)
        assert block_positions is not None, "Failed to sample block positions without collisions. Try increasing the table size or reducing the number of objects."
        
        #choose the orientations based on the configs
        if self.configs['rotation']:
            raise NotImplementedError
            self.obj_ornientations = self.dataset_world.get_random_orientations(sum(object_counts.values()))
        else:
            self.obj_ornientations = [[0,0,0,1]] * len(all_objs)
        
        for i in range(len(all_objs)):
            color, obj_type = all_objs[i]
            position = block_positions.pop(0)
            model_path = os.path.join(self.simulator_handle.local_assets_path, settings.OBJECT_URDF_PATHS[f'{color}_{obj_type}']) if obj_type == 'dice' else settings.OBJECT_URDF_PATHS[obj_type]
            obj = self.simulator_handle.world.add(name = f"{obj_type}_{color}_{i}",
                                                             pos = position,
                                                             ori = self.obj_ornientations[i], 
                                                             obj_type = 'object', 
                                                             model_path = model_path,
                                                             other_properties = dict(type = obj_type, color = color)
                                                             )
            if obj_type != 'dice':
                obj.change_color(settings.COLOR.get(color))
        self.simulator_handle.step_simulation(num_steps=100)
        
        if self.dataset_world.check_collision():
            self.dataset_world.reset_world()
            self.generate_initial_scene()

    def generate_grounded_functional_program(self):
        program  = self.template['program']
        for key, value in self.initializations.items():
            if key in program:
                program = program.replace(key, str(value))
        self.program = program
        status, moved_positions, plan =  self.execute_program(program, dry_run = False)
        return status
    
    def save_demonstration_info(self, command_lexed, command):
        info = dict()
        info['instruction_lexed'] = command_lexed
        info['instruction'] = command
        info['program_lexed'] = self.template['program']
        info['program'] = self.program
        info['initializations'] = self.initializations
        info['concept'] = self.initializations['__IC__']
        info['scene_info'] = self.get_scene_info()
        info['transition_info'] = self.get_transition_info()
        with open(f'{self.default_save_folder}/demo.json', 'w') as f:
            json.dump(info, f, indent=4)
            
    
    



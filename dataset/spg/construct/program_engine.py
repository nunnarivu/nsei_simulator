from dataset.spg.configs import ParameterSettings as settings

class Executor(object):
    def __init__(self, construct_handle, program_library):
        self.construct_handle = construct_handle
        self.concept_library = dict()
        self.focus = None
        self.initialize_primitives()
        self.initialize_programs(program_library)
    
    
    def _register_primitives(self, primitives:list):
        for primitive in primitives:
            func_obj, arguments = primitive
            self.concept_library[func_obj.__name__] = func_obj

    def initialize_primitives(self):
        self._register_primitives([(self.filter, ('obj_list', 'prop')),
                                   (self.shift_focus, ('direction',)),
                                    (self.assign_focus, ('position',)),
                                    (self.place_at_focus, ('obj_id',))
                                   ])

    def initialize_programs(self, program_library):
        for program in program_library:
            exec(program_library[program][2], self.concept_library)

    def filter(self, prop):
        if self.dry_run:
            #return dummy infinite object id list
            return [i for i in range(1,settings.MAX_NUM_OBJECTS)]
        color, type = prop
        return [obj.obj_id for obj in self.construct_handle.objects if obj.color == color and obj.type == type]

    def assign_focus(self, position:tuple, dim:float=0.05):
        self.focus = (position, dim)
    
    def shift_focus(self, direction:str):
        #Assumes the object is cube. dimensions are same in all axis : [self.focus[1]]*3
        relative_pos = self.construct_handle.dataset_world.relative_target_pos(direction.upper(), self.focus[0], [self.focus[1]]*3)
        self.focus = (relative_pos, self.focus[1])

    def place_at_focus(self, obj_id:int):
        if self.focus is None:
            raise ValueError("Focus is not assigned.")
      
        self.placed_positions.append(self.focus[0])
        
        if not self.dry_run:
            #check if object above the move_object
            move_object_position = self.construct_handle.objects[obj_id].pos
            above = self.construct_handle.dataset_world.relative_target_pos('TOP', move_object_position, self.construct_handle.objects[obj_id].dim)
            if self.construct_handle.dataset_world.is_positions_nearby(object_positions = [o.pos for o in self.construct_handle.objects], 
                                                         target_position = self.focus[0], skip_objects = [obj_id]):
                self.execution_status = False
                return 
            
            self.plan.append((obj_id, self.focus[0]))

    def execute(self, program:str, dry_run:bool = False):
        self.plan = []
        self.placed_positions = []
        self.execution_status = True
        self.dry_run = dry_run
        local_env = dict()
        local_env.update(self.concept_library)
        
        exec(program, local_env)
        if not self.dry_run and self.execution_status:
            self.construct_handle.execute_plan(self.plan, save_keyframes = True)
            
        return self.execution_status, self.placed_positions, self.plan

from collections import defaultdict
import os
import cv2
from PIL import Image
import numpy as np
from dataset.nsrm.configs import ParameterSettings as settings
from dataset.nsrm.construct.program_engine import ProgramExecutor





class DatasetWorld(object):
    def __init__(self, simulator_handle=None, configs=None):
        self.simulator_handle = simulator_handle
        self.configs = configs

    def reset_world(self, ):
        self.simulator_handle.reset()
        plane = self.simulator_handle.world.add(name = 'plane', pos=(0, 0, 0), ori=(0, 0, 0, 1), obj_type='entity', model_path=settings.OBJECT_URDF_PATHS['plane'])
        table = self.simulator_handle.world.add(name = 'table', pos=(0, 0, 0), ori=(0, 0, 0, 1), obj_type='entity', model_path=settings.OBJECT_URDF_PATHS['table'])
        table.change_color([0.48, 0.435, 0.2, 1])
        self.simulator_handle.step_simulation(num_steps=100)
        #ADD ROBOT: TODO
        
    def relative_target_pos(self, relation, *args):
        handlers = {
            'TOP': self._top_target_pos,
            'LEFT': self._left_target_pos,
            'RIGHT': self._right_target_pos,
            'FRONT': self._front_target_pos,
            'BACK': self._back_target_pos,
        }
        if relation in handlers:
            return handlers[relation](*args)
        else:
            raise ValueError(f"Unknown relation: {relation}")
        
    def in_camera_view(self, pos):
        return self.simulator_handle.sensors.get_sensor('rgbd_1').is_inside_camera_view(pos)

    def get_random_table_position(self, ):
        table_dim  = self.simulator_handle.world.get_body_dimensions('table')[0]
        table_pos = self.simulator_handle.world.get_body_position('table')
        x = np.random.uniform(table_pos[0] - (table_dim[0] / 2)*settings.TABLE_WORK_AREA, table_pos[0] + (table_dim[0] / 2)*settings.TABLE_WORK_AREA)
        y = np.random.uniform(table_pos[1] - (table_dim[1] / 2)*settings.TABLE_WORK_AREA, table_pos[1] + (table_dim[1] / 2)*settings.TABLE_WORK_AREA)
        z = table_pos[2] + settings.TABLE_OFFSET + table_dim[2] / 2  # Place above the table
        return (x, y, z)

    def check_collision(self, ):
        obj_list = self.simulator_handle.world.find(type='object')
        if self.simulator_handle.world.check_collision(obj_list):
            return True
        return False

    def is_position_clear(self, pos):
        return not self.simulator_handle.world.is_position_colliding(pos, obj_type = 'object')
    
    def is_positions_nearby(self, object_positions, target_position = None, skip_objects:list = [], threshold = 0.05):
        if target_position is not None:
            for obj_idx, obj_pos in enumerate(object_positions):
                if obj_idx in skip_objects:
                    continue
                if np.linalg.norm(np.array(obj_pos) - np.array(target_position)) < threshold:
                    return True
            return False
        else:
            nearby = False
            for i in range(len(object_positions)):
                if i in skip_objects:
                    continue
                target_position = object_positions[i]
                nearby = nearby or self.is_positions_nearby(object_positions, target_position, skip_objects = [i], threshold = threshold)
            return nearby

    def get_block_positions(self, num_blocks, ensure_visibility:bool = False, avoid_positions:list = [] ):
        if ensure_visibility:
            raise NotImplementedError
        block_positions =  None
        
        for i in range(1000):
            temp_block_positions = [self.get_random_table_position() for _ in range(num_blocks)]
            valid = all(self.is_position_clear(pos) for pos in temp_block_positions)
            valid = valid and not self.is_positions_nearby(temp_block_positions+avoid_positions)
            if valid:
                if ensure_visibility and (not all(self.simulator_handle.is_inside_camera_view(pos) for pos in temp_block_positions)):
                    continue
                block_positions = temp_block_positions
                break
        return block_positions

    def _noise(self, ):
        pass
    
    def _inside_target_pos(self, ):
        pass

    def _top_target_pos(self, base_obj_idx, move_obj_idx, all_block_positions = None):
        '''
			Inputs: (Note that the indices are not the object id w.r.t bullet client. The bullet may have additional objects like table and plane.)
				base_obj_id(int): Index of the id w.r.t the action is performed
				move_obj_id(int): The index of the object being moved
				all_block_positions:(list of 3-tuples) The list of positions of all objects in the PandaWorld
			Description: The target z co-ordinate = The left-bottom corner of base_obj + the height of the base object.
		'''
        if all_block_positions is None:
            all_block_positions = [obj.pos for obj in self.simulator_handle.world.find(type='object')]
        base_pos = all_block_positions[base_obj_idx]
        base_dim = self.simulator_handle.world.find(type='object')[base_obj_idx].dim
        move_z_pos = base_pos[2] + base_dim[2]
        # print(base_pos[2],base_dim)
        target_pos = [base_pos[0], base_pos[1], move_z_pos]
        return target_pos

    def _left_target_pos(self, base_obj_idx, move_obj_idx, all_block_positions = None):
        if all_block_positions is None:
            all_block_positions = [obj.pos for obj in self.simulator_handle.world.find(type='object')]
        base_pos = all_block_positions[base_obj_idx]
        move_dim  = self.simulator_handle.world.find(type='object')[move_obj_idx].dim
        move_left_pos = base_pos[0] -move_dim[0] - settings.SPATIAL_TARGET_MARGIN
        target_pos = [move_left_pos, base_pos[1], base_pos[2]]
        return target_pos

    def _right_target_pos(self, base_obj_idx, move_obj_idx, all_block_positions = None):
        if all_block_positions is None:
            all_block_positions = [obj.pos for obj in self.simulator_handle.world.find(type='object')]
        base_pos = all_block_positions[base_obj_idx]
        base_dim = self.simulator_handle.world.find(type='object')[base_obj_idx].dim
        move_right_pos = base_pos[0] + base_dim[0] + settings.SPATIAL_TARGET_MARGIN
        target_pos = [move_right_pos, base_pos[1], base_pos[2]]
        return target_pos

    def _back_target_pos(self, base_obj_idx, move_obj_idx, all_block_positions = None):
        if all_block_positions is None:
            all_block_positions = [obj.pos for obj in self.simulator_handle.world.find(type='object')]
        base_pos = all_block_positions[base_obj_idx]
        move_dim  = self.simulator_handle.world.find(type='object')[move_obj_idx].dim
        move_back_pos = base_pos[1] - move_dim[1] - settings.SPATIAL_TARGET_MARGIN
        target_pos = [base_pos[0], move_back_pos, base_pos[2]]
        return target_pos

    def _front_target_pos(self, base_obj_idx, move_obj_idx, all_block_positions = None):
        if all_block_positions is None:
            all_block_positions = [obj.pos for obj in self.simulator_handle.world.find(type='object')]
        base_pos = all_block_positions[base_obj_idx]
        base_dim = self.simulator_handle.world.find(type='object')[base_obj_idx].dim
        move_front_pos = base_pos[1] + base_dim[1] + settings.SPATIAL_TARGET_MARGIN
        target_pos = [base_pos[0], move_front_pos, base_pos[2]]
        return target_pos



class DatasetConstructBase(object):
    def __init__(self, simulator_handle, configs):
        self.dataset_world = DatasetWorld(simulator_handle, configs)
        self.executor = ProgramExecutor(self)
        self.default_save_folder = None
        self.cached_state = []
        

    @property
    def simulator_handle(self):
        return self.dataset_world.simulator_handle

    @property
    def configs(self):
        return self.dataset_world.configs
    
    @property
    def objects(self):
        return self.dataset_world.simulator_handle.world.find(type='object')

    def hide_robot_body(self, ):
        if self.simulator_handle.robot is not None:
            self.simulator_handle.robot.change_color([0, 0, 0, 0])
    
    def  show_robot_body(self, ):
        if self.simulator_handle.robot is not None:
            self.simulator_handle.robot.change_color([1, 1, 1, 1])

    def save_instance(self, root_folder = None, state_tag = None, show_panda = False):
        if root_folder is None:
            root_folder = self.default_save_folder
        demo_state_no = sum([os.path.isdir(os.path.join(root_folder, s)) for s in os.listdir(root_folder)])
        this_demo_folder = os.path.join(root_folder, 'S'+"{0:0=2d}".format(demo_state_no))
        os.makedirs(this_demo_folder)
        
        if show_panda == False:
            self.hide_robot_body()
            
        for view in settings.CAPTURE_CAMERA_VIEWS:
            camera = self.simulator_handle.sensors.get_sensor(type = 'rgbd', name='rgbd_'+view)
            camera_output = camera.get_camera_image(use_gpu = True)
            rgba, depth, mask = camera_output['rgb'], camera_output['depth'], camera_output['mask']
            #make values in mask -1 if it is not in self.objects
            obj_ids = [obj.id for obj in self.objects]
            mask[~np.isin(mask, obj_ids)] = -1
            
            os.makedirs(os.path.join(this_demo_folder, view), exist_ok=True)
            Image.fromarray(rgba, 'RGBA').save(os.path.join(this_demo_folder, view, 'rgb.png'))
            Image.fromarray(depth, 'L').save(os.path.join(this_demo_folder, view, 'depth.png'))
            Image.fromarray(mask, 'L').save(os.path.join(this_demo_folder, view, 'mask.png'))
            np.save(os.path.join(this_demo_folder, view, 'mask.npy'), mask)
            np.save(os.path.join(this_demo_folder, view, 'depth.npy'), depth)

    def get_scene_info(self, ):
        current_state_info = self.simulator_handle.world.get_state_info()
        scene_info = defaultdict(list)
        for key, value in current_state_info.items():
            if value['type'] == 'object':
                scene_info['object_names'].append(key)
                scene_info['object_ids'].append(value['info']['obj_id'])
                scene_info['model_paths'].append(value['info']['model_path'])
        return scene_info

    def get_transition_info(self, ):
        state_cache_info = self.simulator_handle.world.state_cache
        transition_info = defaultdict(lambda: defaultdict(list))
        for _, this_state_info in state_cache_info.items():
            for obj_name, obj_info in this_state_info.items():
                if obj_info['type'] == 'object':
                    transition_info[obj_name]['position'].append(obj_info['pos'])
                    transition_info[obj_name]['orientation'].append(obj_info['ori'])
                    transition_info[obj_name]['linear_velocity'].append(obj_info['linear_vel'])
                    transition_info[obj_name]['angular_velocity'].append(obj_info['angular_vel'])
        return transition_info

    def execute_plan(self, plan, use_robot = False, save_keyframes = True):
        for move_obj_idx, target_pos in plan:
            self.move_object(move_obj_idx, target_pos, use_robot)
        
            if save_keyframes:
                self.simulator_handle.world.cache_state()
                self.save_instance()

    def move_object(self, move_obj_idx, target_pos, use_robot = False):
        if use_robot:
            raise NotImplementedError
        else:
            move_obj = self.simulator_handle.world.find(type='object')[move_obj_idx]
            move_obj.reset_pose(pos = target_pos)
            self.simulator_handle.step_simulation(num_steps=50)

    def check_action_compatibility(self, program, block_positions):
        import copy
        target_positions = list()
        plan = list()
        block_positions_cur = copy.deepcopy(block_positions)
        for subtasks in program:
            action, move_obj_idx, base_obj_idx = subtasks
            if move_obj_idx == base_obj_idx: return None, None
            
            # check if move object doesn't have anything on top
            up_pos = self.dataset_world._top_target_pos(move_obj_idx, move_obj_idx, block_positions_cur)
            if self.dataset_world.is_positions_nearby(object_positions = block_positions_cur, target_position = up_pos, skip_objects = [move_obj_idx,]):
                print("Not clear from top ")
                return None, None
            
            # check if target position valid
            if action == 'TOP':
                tr_pos = self.dataset_world._top_target_pos(base_obj_idx, move_obj_idx, block_positions_cur)
                if not self.dataset_world.is_positions_nearby(block_positions_cur, tr_pos, skip_objects=[base_obj_idx,]):
                    target_positions.append(tr_pos)
                    block_positions_cur[move_obj_idx] = tr_pos
                else:
                    print("top positon not empty")
                    return None, None
            elif action == 'LEFT':
                tr_pos = self.dataset_world._left_target_pos(base_obj_idx, move_obj_idx, block_positions_cur)
                if not self.dataset_world.is_positions_nearby(block_positions_cur, tr_pos):
                    target_positions.append(tr_pos)
                    block_positions_cur[move_obj_idx] = tr_pos
                else:
                    print("Shift position not empty")
                    # print("Target position is ",tr_pos)
                    # print("block_positions right now are ",block_positions) 
                    # self.save_instance()
                    return None, None
            elif action == 'RIGHT':
                tr_pos = self.dataset_world._right_target_pos(base_obj_idx, move_obj_idx, block_positions_cur)
                if not self.dataset_world.is_positions_nearby(block_positions_cur, tr_pos):
                    target_positions.append(tr_pos)
                    block_positions_cur[move_obj_idx] = tr_pos
                else:
                    print("right position not empty")
                    return None, None
            elif action == 'FRONT':
                tr_pos = self.dataset_world._front_target_pos(base_obj_idx, move_obj_idx, block_positions_cur)
                if not self.dataset_world.is_positions_nearby(block_positions_cur, tr_pos):
                    target_positions.append(tr_pos)
                    block_positions_cur[move_obj_idx] = tr_pos
                else:
                    return None, None
            elif action == 'BACK':
                tr_pos = self.dataset_world._back_target_pos(base_obj_idx, move_obj_idx, block_positions_cur)
                if not self.dataset_world.is_positions_nearby(block_positions_cur, tr_pos):
                    target_positions.append(tr_pos)
                    block_positions_cur[move_obj_idx] = tr_pos
                else:
                    return None, None
            else:
                raise ValueError("Unknown action %s" % action)
            plan.append((move_obj_idx, tr_pos))

        return target_positions, plan


    def save_checkpoint(self, state_tag:str = None, dump_to_file:bool = False, root_folder:str = None):
        tag, checkpoint_dict = self.simulator_handle.save_checkpoint(state_tag, dump_to_file, root_folder)
        self.cached_state.append(dict(tag = tag, checkpoint = checkpoint_dict))

    def load_checkpoint(self, state_tag = None, from_file = None):
        self.simulator_handle.load_checkpoint(state_tag, from_file)
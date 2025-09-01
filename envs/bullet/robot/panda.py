import pybullet as p
import time
import math




class PandaRobot(object):
    PANDA_NUM_DOF =7
    PANDA_END_EFFECTOR_INDEX = 11
    PANDA_REVOLUTE_JOINTS = [0,1,2,3,4,5,6]
    PANDA_PRISMATIC_JOINTS = [9,10]
    PANDA_FIXED_JOINTS = [7,8,11]
    JOINT_LOWER_LIMITS = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    JOINT_UPPER_LIMITS = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
    JOINT_MAX_TORQUES = [87, 87, 87, 87, 12, 12, 12]
    JOINT_MAX_VELOCITIES = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]
    jOINT_RANGES = [5.795, 3.525, 5.795, 3.141, 5.795, 6.283, 5.795]
    PANDA_FINGER_OPEN = 0.04
    PANDA_FINGER_CLOSED = 0.0
    PANDA_FINGER_VERTICAL = [0, math.pi, 0]
    
    def __init__(self, physics_client, base_position = None, base_orientation = None, **kwargs) -> None:
        self.physics_client = physics_client
        self.base_position = base_position if base_position is not None else [0,0.5,0.625]
        self.base_orientation = base_orientation if base_orientation is not None else p.getQuaternionFromEuler([0,0,-math.pi/2])
        self.robot_id = physics_client.loadURDF('franka_panda/panda.urdf', self.base_position, self.base_orientation, useFixedBase = 1)
        self.name = 'franka_panda'
        self.home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04] if 'home_position' not in kwargs else kwargs['home_position']
        
        finger_constraint = self.physics_client.createConstraint(
			self.robot_id, 9, self.robot_id, 10,
			jointType=p.JOINT_GEAR,
			jointAxis=[1, 0, 0],
			parentFramePosition=[0, 0, 0],
			childFramePosition=[0, 0, 0])
        
        self.physics_client.changeConstraint(finger_constraint, gearRatio=-1, erp=0.1, maxForce=30)
    
    def is_moving(self, ):
        #Panda is moving if any of the joint velocities is not close to zero
        for j in self.PANDA_REVOLUTE_JOINTS:
            state = self.physics_client.getJointState(self.robot_id, j)
            this_joint_velocity = state[1]
            if not math.isclose(this_joint_velocity, 0, abs_tol=1e-5):
                return True
        return False
    
    def execute_action(self, max_steps = 5000):
        #manually simulate the each step of the simulation to make the robot move to the home position
        #we define a function is_moving to check if panda is in a moving state. 
        #If the panda is moving, we keep simulating the environment. Once the panda stops moving, we stop the simulation.  
        timestep = p.getPhysicsEngineParameters()['fixedTimeStep']
        p.stepSimulation() #Initiate the first simulation step
        step, status = 0, True
        while self.is_moving() and step < max_steps:
            p.stepSimulation()
            time.sleep(timestep)
            step+=1
            if step == 2400:
                status = False
        return status 
    
    def move_to_home_position(self, open_fingers:bool = True):
        
        index =0 
        for j in range(p.getNumJoints(self.robot_id)):
            p.changeDynamics(self.robot_id, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.robot_id, j)
            jointName = info[1]
            jointType = info[2]
            if jointType == p.JOINT_REVOLUTE:           
                p.setJointMotorControl2(bodyIndex = self.robot_id, jointIndex=index, controlMode = p.POSITION_CONTROL, targetPosition = self.home_position[index], maxVelocity = 0.6)
                index=index+1

        #Open the gripper
        if open_fingers:
            for i in self.PANDA_PRISMATIC_JOINTS:
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, targetPosition=self.PANDA_FINGER_OPEN, force=100)
        status = self.execute_action()
        if not status:
            print("Warning: Action move_to_home_position is taking long time.")
    
    def move_to_joint_position(self, joint_positions:list, include_fingers:bool = False):
        if include_fingers:
            assert len(joint_positions) == self.PANDA_NUM_DOF + 2, f"The joint_positions should be of length {self.PANDA_NUM_DOF + 2} if include_fingers is True"
        else:
            assert len(joint_positions) == self.PANDA_NUM_DOF, f"The joint_positions should be of length {self.PANDA_NUM_DOF} if include_fingers is False"
        
        index = 0
        for j in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, j)
            jointType = info[2]
            if jointType == p.JOINT_REVOLUTE:
                p.setJointMotorControl2(bodyIndex = self.robot_id, jointIndex=index, controlMode = self.physics_client.POSITION_CONTROL, targetPosition = joint_positions[index], maxVelocity = 0.6 )
                index=index+1
        status = self.execute_action()
        if include_fingers:
            p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL, targetPosition=joint_positions[-2], force=100)
            p.setJointMotorControl2(self.robot_id, 10, p.POSITION_CONTROL, targetPosition=joint_positions[-1], force=100) 
        if not status:
            print("Warning: Action move_to_joint_position is taking long time.")

    def open_fingers(self, ):
        p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL, targetPosition=self.PANDA_FINGER_OPEN, force=100)
        p.setJointMotorControl2(self.robot_id, 10, p.POSITION_CONTROL, targetPosition=self.PANDA_FINGER_OPEN, force=100)
        self.execute_action(max_steps = 50)
    
    def close_fingers(self, ):
        p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL, targetPosition=self.PANDA_FINGER_CLOSED, force=100)
        p.setJointMotorControl2(self.robot_id, 10, p.POSITION_CONTROL, targetPosition=self.PANDA_FINGER_CLOSED, force=100)
        self.execute_action(max_steps = 50)
    
    def _pick_object(self, position, orientation = None):
        assert orientation is None, "The orientation is not implemented yet."
        #restposes for null space
        rp = self.home_position

        #We will use the inverse kinematics to move the panda end effector to the cube position.
        #The function p.calculateInverseKinematics is used to calculate the inverse kinematics. The function returns the joint positions which will move the panda end effector to the cube position.
        joint_positions = p.calculateInverseKinematics(self.robot_id, self.PANDA_END_EFFECTOR_INDEX, targetPosition=position, lowerLimits=self.JOINT_LOWER_LIMITS, upperLimits=self.JOINT_UPPER_LIMITS, 
                                                    jointRanges=self.jOINT_RANGES, restPoses=rp, maxNumIterations=10000, residualThreshold=1e-4)

        # Note that we just use targetPosition. We can also optionaly give targetOrientation. 
        # However, the targetOrientation is not same as the cube orientation because we have arbitrarity rotated the cube.

        #Move to home position, move to the joint_positions found by inverse kinematics and close the fingers to grasp the object
        self.move_to_home_position(open_fingers=True)
        self.move_to_joint_position(joint_positions, include_fingers=True)
        self.close_fingers()
        self.move_to_home_position(open_fingers=False)
        
    def _place_at_target_position(self, position, orientation = None):
        assert orientation is None, "The target_cube_orientation is not implemented yet."
        
        rp = self.home_position
        #We will use the inverse kinematics to move the panda end effector to the target_cube position.
        joint_positions = p.calculateInverseKinematics(self.robot_id, self.PANDA_END_EFFECTOR_INDEX, targetPosition=position,lowerLimits=self.JOINT_LOWER_LIMITS, upperLimits=self.JOINT_UPPER_LIMITS, 
                                                jointRanges=self.jOINT_RANGES, restPoses=rp, maxNumIterations=10000, residualThreshold=1e-4)

        self.move_to_joint_position(joint_positions[0:7], include_fingers=False)
        self.open_fingers()
        self.move_to_home_position(open_fingers=False)
        
    def crane_pick_object(self, position, orientation = None):
        assert orientation is None, "The orientation is not implemented yet."
        #restposes for null space
        rp = self.home_position

        #We will use the inverse kinematics to move the panda end effector to the cube position.
        #The function p.calculateInverseKinematics is used to calculate the inverse kinematics. The function returns the joint positions which will move the panda end effector to the cube position.
        move_up_joint_positions = p.calculateInverseKinematics(self.robot_id, self.PANDA_END_EFFECTOR_INDEX,  targetPosition=[position[0], position[1], position[2]+0.3], targetOrientation = [0, math.pi, 0], 
                                                    lowerLimits=self.JOINT_LOWER_LIMITS, upperLimits=self.JOINT_UPPER_LIMITS, 
                                                    jointRanges=self.jOINT_RANGES, restPoses=rp, maxNumIterations=10000, residualThreshold=1e-4)
        
        grasp_position = p.calculateInverseKinematics(self.robot_id, self.PANDA_END_EFFECTOR_INDEX,  targetPosition = position, targetOrientation = [0, math.pi, 0], 
                                                    lowerLimits=self.JOINT_LOWER_LIMITS, upperLimits=self.JOINT_UPPER_LIMITS, 
                                                    jointRanges=self.jOINT_RANGES, restPoses=rp, maxNumIterations=10000, residualThreshold=1e-4)

        # Note that we just use targetPosition. We can also optionaly give targetOrientation. 
        # However, the targetOrientation is not same as the cube orientation because we have arbitrarity rotated the cube.

        #Move to home position, move to the joint_positions found by inverse kinematics and close the fingers to grasp the object
        self.move_to_home_position(open_fingers=True)
        self.move_to_joint_position(move_up_joint_positions, include_fingers=True)
        self.move_to_joint_position(grasp_position, include_fingers=True)
        self.close_fingers()
        self.move_to_home_position(open_fingers=False)
    
    def crane_place_at_position(self, position, orientation = None):
        assert orientation is None, "The target_cube_orientation is not implemented yet."
        
        rp = self.home_position
        #We will use the inverse kinematics to move the panda end effector to the target_cube position.
        move_up_joint_positions = p.calculateInverseKinematics(self.robot_id, self.PANDA_END_EFFECTOR_INDEX,  targetPosition=[position[0], position[1], position[2]+0.3], targetOrientation = [0, math.pi, 0], 
                                                    lowerLimits=self.JOINT_LOWER_LIMITS, upperLimits=self.JOINT_UPPER_LIMITS, 
                                                    jointRanges=self.jOINT_RANGES, restPoses=rp, maxNumIterations=10000, residualThreshold=1e-4)
        
        place_position = p.calculateInverseKinematics(self.robot_id, self.PANDA_END_EFFECTOR_INDEX,  targetPosition = position, targetOrientation = [0, math.pi, 0], 
                                                    lowerLimits=self.JOINT_LOWER_LIMITS, upperLimits=self.JOINT_UPPER_LIMITS, 
                                                    jointRanges=self.jOINT_RANGES, restPoses=rp, maxNumIterations=10000, residualThreshold=1e-4)

        self.move_to_joint_position(move_up_joint_positions[0:7], include_fingers=False)
        self.move_to_joint_position(place_position[0:7], include_fingers=False)
        self.open_fingers()
        self.move_to_joint_position(move_up_joint_positions[0:7], include_fingers=False)
        self.move_to_home_position(open_fingers=False)


    def is_occupying(self, position):
        '''Use collision model to check if robot body is occupying <position>'''
        for link_id in range(p.getNumJoints(self.robot_id)):
            aabb = p.getAABB(self.robot_id, link_id)
            if (aabb[0] <= position[0] <= aabb[1] and
                aabb[2] <= position[1] <= aabb[3] and
                aabb[4] <= position[2] <= aabb[5]):
                return True
        return False
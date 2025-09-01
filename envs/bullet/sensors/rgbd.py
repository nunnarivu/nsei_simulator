import numpy as np


class RGBDSensor:
    def __init__(self, client, camera_info, image_info):
        self.client = client  #pybullet client
        valid_param, _ = self.is_camera_info_valid(camera_info)

        assert valid_param, "Camera info is not valid"
        assert self.is_image_info_valid(image_info), "Image info is not valid. Height and width must be specified and positive integers."
        self.set_camera_params(camera_info)
        self.set_image_info(image_info)
        
    def is_camera_info_valid(self, camera_info):
        # Validate camera_info
        if "camera_eye_position" in camera_info and \
           "camera_target_position" in camera_info and \
           "camera_up_vector" in camera_info and \
           "fov" in camera_info and \
           "aspect" in camera_info and \
           "near_val" in camera_info and \
           "far_val" in camera_info:
            return True, 1

        elif "camera_eye_position" in camera_info  and \
            "camera_target_position" in camera_info and \
            "camera_yaw" in camera_info and \
            "camera_pitch" in camera_info and \
            "camera_roll" in camera_info and \
            "fov" in camera_info and \
           "aspect" in camera_info and \
           "near_val" in camera_info and \
           "far_val" in camera_info:
            return True, 2

        elif "camera_intrinsics" in camera_info and \
             "camera_extrinsics" in camera_info:
            return True, 3
        
        return False, -1

    def is_image_info_valid(self, image_info):
        # Validate image_info
        if "width" in image_info and "height" in image_info:
            if isinstance(image_info["width"], int) and image_info["width"] > 0 and \
               isinstance(image_info["height"], int) and image_info["height"] > 0:
                return True
            return False
        return False

    def set_camera_params(self, camera_info, reset_visualizer = True):
        valid_param, camera_param_type = self.is_camera_info_valid(camera_info)
        assert valid_param, "Camera info is not valid"
        self.camera_info = camera_info

        if camera_param_type == 1:
            view_matrix = self.client.computeViewMatrix(
                cameraEyePosition = self.camera_info["camera_eye_position"],
                cameraTargetPosition = self.camera_info["camera_target_position"],
                cameraUpVector = self.camera_info["camera_up_vector"]
            )
        elif camera_param_type == 2:
            view_matrix = self.client.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition = self.camera_info["camera_target_position"],
                distance = np.linalg.norm(np.array(self.camera_info["camera_eye_position"]) - np.array(self.camera_info["camera_target_position"])),
                cameraYaw = self.camera_info["camera_yaw"],
                cameraPitch = self.camera_info["camera_pitch"],
                cameraRoll = self.camera_info["camera_roll"]
            )
        else: 
            raise NotImplementedError("Camera parameters type 3 (intrinsics/extrinsics) not implemented yet")

        self.view_matrix = view_matrix
        projection_matrix = self.client.computeProjectionMatrixFOV(
            fov = self.camera_info["fov"],
            aspect = self.camera_info["aspect"],
            nearVal = self.camera_info["near_val"],
            farVal = self.camera_info["far_val"]
        )
        self.projection_matrix = projection_matrix

        if reset_visualizer:
            vis_cam_params = self._view_to_debug_params(view_matrix)
            self.client.resetDebugVisualizerCamera(*vis_cam_params)

    def set_image_info(self, image_info):
        assert self.is_image_info_valid(image_info), "Image info is not valid. Height and width must be specified and positive integers."
        self.image_info = image_info

    def get_camera_image(self, use_gpu = True, from_visualizer = False):
        assert self.image_info is not None, "Image info is not set"
        renderer = self.client.ER_BULLET_HARDWARE_OPENGL if use_gpu else self.client.ER_TINY_RENDERER
        
        if from_visualizer:
            return self.get_camera_image_from_visualizer()
        
        width, height, rgb, depth,seg = self.client.getCameraImage(
            width=self.image_info["width"],
            height=self.image_info["height"],
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            renderer=renderer
        )
        return dict(width=width, height=height, rgb=rgb, depth=depth, mask=seg)

    def get_camera_image_from_visualizer(self):
        visualizer_info = self.client.getDebugVisualizerCamera()
        view_matrix = visualizer_info[2]
        projection_matrix = visualizer_info[3]
        
        width, height, rgb, depth,seg = self.client.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix
        )
        return dict(width=width, height=height, rgb=rgb, depth=depth, mask=seg)

    def copy_visualizer_camera(self):
        visualizer_info = self.client.getDebugVisualizerCamera()
        view_matrix = visualizer_info[2]
        projection_matrix = visualizer_info[3]

        self.view_matrix = view_matrix
        self.projection_matrix = projection_matrix

    def is_inside_camera_view(self, position: list):
        # Transform the position into homogeneous coordinates
        pos_homogeneous = np.array([position[0], position[1], position[2], 1.0])

        # Compute the clip space position
        view_matrix_np = np.array(self.view_matrix).reshape(4, 4).T
        projection_matrix_np = np.array(self.projection_matrix).reshape(4, 4).T
        clip_space_pos = projection_matrix_np @ view_matrix_np @ pos_homogeneous

        # Perform perspective divide to get normalized device coordinates
        ndc = clip_space_pos[:3] / clip_space_pos[3]

        # Check if the position is within the normalized device coordinates
        return -1.0 <= ndc[0] <= 1.0 and -1.0 <= ndc[1] <= 1.0 and -1.0 <= ndc[2] <= 1.0


    def _view_to_debug_params(view_matrix):
        """Convert a 4x4 OpenGL-style view matrix into PyBullet debug camera params."""
        M = np.array(view_matrix).reshape(4, 4).T
        R = M[:3, :3]
        t = M[:3, 3]

        # Camera eye = -R^T * t
        eye = -R.T @ t

        # Forward vector = -R^T * [0,0,1] (OpenGL looks down -Z)
        forward = -R.T @ np.array([0, 0, 1])

        # Pick target point = eye + forward * d
        distance = np.linalg.norm(forward)
        target = eye + forward

        # PyBullet yaw/pitch convention
        dx, dy, dz = (target - eye)
        yaw = np.degrees(np.arctan2(dx, dy))
        pitch = np.degrees(np.arctan2(dz, np.sqrt(dx**2 + dy**2)))
        distance = np.linalg.norm(target - eye)

        return distance, yaw, pitch, target.tolist()
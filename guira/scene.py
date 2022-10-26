import ctypes
import time

import numpy as np
import pandas as pd

from guira import sim # simulation lib
from guira.guira_exceptions import SimulatorException 
from guira.utils import get_normals_orientation, map_points_to_global

class Scene:
    def __init__(self):
        """Scene obj instancing.  

        Returns:
            None
        """
        return None

    def connect(self,
                connection_addr = '127.0.0.1',
                connection_port=19999, 
                wait_until_connected = True,
                do_not_reconnect = True,
                timeout_ms = 5000,
                comm_thread_cycle_ms = 5):
        """
        Close all opened connections and connects to the simulator API (if it is already running).

        Args:
            connection_addr (str, optional): CoppeliaSim server ip. Defaults to '127.0.0.1'.
            connection_port (int, optional): Port number to connect (same as defined in simulation scene). Defaults to 19999.
            wait_until_connected (bool, optional): if True, function blocks until connected. Defaults to True.
            do_not_reconnect (bool, optional): if True, a new connection attempt is not tried (if connection was lost). Defaults to True.
            timeout_ms (int, optional): miliseconds time-out for the first connection atempt (if positive). If negative, the positive 
                                        value is the time-out for block function calls. For more information please read the remoteApi docs. Defaults to 5000.
            comm_thread_cycle_ms (int, optional): how often data packets are sent back and forth. Defaults to 5.

        Raises:
            SimulatorException: Raised if not successfully connected.

        Returns:
            client_id(int): the client ID.
        """

        self.connection_address = connection_addr
        self.connection_port = connection_port
        self.wait_until_connected = wait_until_connected
        self.do_not_reconnect = do_not_reconnect
        self.timeout_ms = timeout_ms
        self.comm_thread_cycle_ms = comm_thread_cycle_ms

        sim.simxFinish(-1) # just in case, close all opened connections

        client_id = sim.simxStart(self.connection_address,
                                 self.connection_port,
                                 self.wait_until_connected,
                                 self.do_not_reconnect,
                                 self.timeout_ms,
                                 self.comm_thread_cycle_ms) # Connect to CoppeliaSim

        if client_id != -1:
            print('Connected to remote API server')
        else:
            raise SimulatorException("Could not connect")

        self.client_id = client_id   
        return self.client_id     
    
    def test_connection(self,):
        """Tests if the connection was successful. If so, the number of objects in the scene is 
           returned.
        """
        res, objs = sim.simxGetObjects(self.client_id,
                                       sim.sim_handle_all,
                                       sim.simx_opmode_blocking)
                                
        if res==sim.simx_return_ok:
            return len(objs)
        else:
            raise SimulatorException('Remote API function call returned with error code: ',res)

    def send_points_to_sim(self, points, sleep_time = 0.07):
        """Send points to simulator.

        Args:
            points (list): the points to be sent.
            sleep_time (float, optional): The amount of time to wait between transmissions. 
                                          The bigger this number is the more acurate the
                                           points will be displayed. Defaults to 0.07.

        Raises:
            ValueError: raised when sleep time is smaller than 0.07
            SimulatorException: raised when transmission is not successful.
        """
        #the bigger the sleep time the more accurate the points are 
        #placed but you have to be very patient :D
        if sleep_time < 0.07:
            raise ValueError('Sleep Time must be greater than 0.07')

        i = 0
        print('Sending Points ...')
        for p in points:
            packedData=sim.simxPackFloats(p.flatten())
            raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
            
            returnCode = sim.simxWriteStringStream(self.client_id,
                                                   "point_coord",
                                                   raw_bytes,
                                                   sim.simx_opmode_oneshot)

            if returnCode != 0 and returnCode != 1:
                raise SimulatorException(f'Point {p.round(3)} not sent. Error {returnCode}')
            else:
                i = i + 1
            time.sleep(sleep_time)
        print(f'Points sent: {i}')

    def get_object_handle(self,object_name):
        """This method returns the object handle.

        Args:
            object_name (string): object's name as seen in the hierarchy tree.

        Raises:
            SimulatorException: this error is raised when any of the error codes is not zero (returned when the handle is retrieved).

        Returns:
            handle_obj (int):  an ID related to the simulated object.
        """
        error_obj, handle_obj = sim.simxGetObjectHandle(clientID=self.client_id,
                                                        objectName=object_name,
                                                        operationMode=sim.simx_opmode_blocking)
        if error_obj != 0:
            raise SimulatorException('something went wrong during object handle retrieval. Error code = {error_obj}')
        else:
            return handle_obj

    def get_handlers_scene(self,scene_objects:list):
        """This function returns the scene objects handlers.

        Args:
            scene_objects (list): a list of scene objects' names whose handlers should be retrieved.

        Returns:
            list_of_handlers (list): a list of dictionary. Each dictionary has the object name and the handler.
        """
        list_of_handlers = []
        for element in scene_objects:
            current_obj_handle = self.get_object_handle(client_id=self.client_id, 
                                                         object_name = element)
            current_dict = {element: current_obj_handle}
            list_of_handlers.append(current_dict)
        return list_of_handlers

    def get_scene_objects_info(self, scene_objects:list):
        """This function return compilled information regarding the scene's objects. It wrapps get_object_handle() and get_configuration() to optimize list iteration.

        Args:
            scene_objects (list): a list of scene objects' names whose handlers should be retrieved.

        Returns:
            info_list (list of dictionary): a list of dictionary. Each dictionary has the object name of the object, its handler, and its configuration (position and orientation).
        """
        info_list = []
        for element in scene_objects:
            current_obj_handle = self.get_object_handle(object_name=element)
            position, angle = self.get_configuration(object_handle=current_obj_handle)
            current_dict = {'name': element,
                            'handler': current_obj_handle,
                            'position': position,
                            'orientation':angle}
            info_list.append(current_dict)
        return info_list

    def get_configuration(self, object_handle):
        """This function returns the object configuration (position and orientation vectors).

        Args:
            object_handle (int): an ID related to the simulated object whose configuration will be retrieved 

        Raises:
            SimulatorException: this error is raised when any of the error codes is not zero (returned when the vectors are retrieved).

        Returns:
            tuple: the position vector and the angle vector.
        """
        position_error, position_vector = sim.simxGetObjectPosition(self.client_id,
                                                                    object_handle,
                                                                    -1, 
                                                                    sim.simx_opmode_blocking)
        angle_error, angle_vector = sim.simxGetObjectOrientation(self.client_id,
                                                                 object_handle,
                                                                 -1, 
                                                                 sim.simx_opmode_blocking)
        if position_error+angle_error != 0:
            raise SimulatorException('an inexpected error occurred when configuration were retrieved')
        return position_vector, angle_vector

    ################# FUNCTIONS FOR PROJECT 2, GOAL 1 #################################
    def get_bounding_box(self, 
                         object_handle:int, 
                         frame:str ='global', 
                         parameter_id_type:str ='model', 
                         consider_up_layer:bool = False): # get_bounding_box_corners_local_coordinates
        if frame != 'global' and frame != 'local':
            raise ValueError(f'Wrong Value of Frame')  
        else:
            local_coordinates = self.__get_bounding_box_corners_local_coordinates(object_handle, 
                                                                                 parameter_id_type = parameter_id_type,
                                                                                 consider_up_layer = consider_up_layer)
            if frame == 'local':
                return local_coordinates
            else:
                position_vector, angle_vector = self.get_configuration(object_handle=object_handle)
                angle_vector = angle_vector.reverse()
                global_coordinates = map_points_to_global(local_coordinates=local_coordinates,
                                                          euler_angles=angle_vector,
                                                          frame_origin_position=position_vector)
                return global_coordinates

                
            
            

    def __get_bounding_box_corners_positions(self, object_handle, parameter_id_type='model'):
        """This method returns the coordinates (on the robot's reference frame) of the two extreme corners of the bounding box.

        Args:
            object_handle (int): an ID related to the simulated object whose bounding box's corners will be retrieved 
            parameter_id_type (str, optional): can be "model" or "object". Defaults to 'model'.

        Raises:
            RuntimeError: this error is raised when any of the error codes is not zero (returned when the corners are retrieved).
            RuntimeError: this error is raised when any of the error codes is not zero (returned when the corners are retrieved). 
            ValueError: this error is raised when the parameter_id_type is not one of the two types showed.

        Returns:
            tuple: the coordinates of the two extreme corners of the bounding box (min_x, min_y, min_z, max_x, max_y, max_z).
        """
        if parameter_id_type == 'model':
            error_min_x, min_x = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_modelbbox_min_x,
                                    operationMode=sim.simx_opmode_blocking)
            error_min_y, min_y = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_modelbbox_min_y,
                                    operationMode=sim.simx_opmode_blocking)
            error_min_z, min_z =  sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_modelbbox_min_z,
                                    operationMode=sim.simx_opmode_blocking)
            error_max_x, max_x = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_modelbbox_max_x,
                                    operationMode=sim.simx_opmode_blocking)
            error_max_y, max_y = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_modelbbox_max_y,
                                    operationMode=sim.simx_opmode_blocking)
            error_max_z, max_z = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_modelbbox_max_z,
                                    operationMode=sim.simx_opmode_blocking)
            if error_min_x+error_min_y+error_min_z+error_max_x+error_max_y+error_max_z != 0:
                raise RuntimeError('an inexpected error occurred when corners were retrieved')
        elif parameter_id_type == 'object':
            error_min_x, min_x = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_objbbox_min_x,
                                    operationMode=sim.simx_opmode_blocking)
            error_min_y, min_y = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_objbbox_min_y,
                                    operationMode=sim.simx_opmode_blocking)
            error_min_z, min_z =  sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_objbbox_min_z,
                                    operationMode=sim.simx_opmode_blocking)
            error_max_x, max_x = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_objbbox_max_x,
                                    operationMode=sim.simx_opmode_blocking)
            error_max_y, max_y = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_objbbox_max_y,
                                    operationMode=sim.simx_opmode_blocking)
            error_max_z, max_z = sim.simxGetObjectFloatParameter(clientID=self.client_id,
                                    objectHandle= object_handle,
                                    parameterID=sim.sim_objfloatparam_objbbox_max_z,
                                    operationMode=sim.simx_opmode_blocking)
            if error_min_x+error_min_y+error_min_z+error_max_x+error_max_y+error_max_z != 0:
                raise RuntimeError('an inexpected error occurred when corners were retrieved')
        else:
            raise ValueError('Invalid parameter')
        return min_x, min_y, min_z, max_x, max_y, max_z
    
    def __get_bounding_box_corners_local_coordinates(self, object_handle, parameter_id_type='model', consider_up_layer = False):
        """This method returns the coordinates of the bounding box corners, in the local reference frame. The order starts at the top right point and goes anti-clockwise until the down right point.

        Args:
            object_handle (int): an ID related to the simulated object whose bounding box's corners will be retrieved 
            parameter_id_type (str, optional): can be "model" or "object". Defaults to 'model'.
            consider_up_layer (boolean, optional): this indicates if the up layer points will be returned. Defaults to "False".

        Returns:
            tuple: vectors from point 1 to point 8
        """
        # the idea is to get the coordinates and return it anti-clockwise (to match mapping algorithm)
        x_min, y_min, z_min, x_max, y_max, z_max = self.__get_bounding_box_corners_positions(object_handle,
                                                                                            parameter_id_type)

        # down layer points
        point_1 = np.array([x_max,y_max,z_min])
        point_2 = np.array([x_min,y_max,z_min])
        point_3 = np.array([x_min,y_min,z_min])
        point_4 = np.array([x_max,y_min,z_min])
        
        if consider_up_layer:
            point_5 = np.array([x_max,y_max,z_max])
            point_6 = np.array([x_min,y_max,z_max])
            point_7 = np.array([x_min,y_min,z_max])
            point_8 = np.array([x_max,y_min,z_max])
            return point_1, point_2, point_3, point_4, point_5, point_6, point_7, point_8
        
        else:
            return point_1, point_2, point_3, point_4
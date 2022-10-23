import ctypes
import time

import numpy as np
import pandas as pd

from guira import sim # simulation lib
from guira.guira_exceptions import SimulatorException 

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
            clientID(int): the client ID.
        """

        self.connection_address = connection_addr
        self.connection_port = connection_port
        self.wait_until_connected = wait_until_connected
        self.do_not_reconnect = do_not_reconnect
        self.timeout_ms = timeout_ms
        self.comm_thread_cycle_ms = comm_thread_cycle_ms

        sim.simxFinish(-1) # just in case, close all opened connections

        clientID = sim.simxStart(self.connection_address,
                                 self.connection_port,
                                 self.wait_until_connected,
                                 self.do_not_reconnect,
                                 self.timeout_ms,
                                 self.comm_thread_cycle_ms) # Connect to CoppeliaSim

        if clientID != -1:
            print('Connected to remote API server')
        else:
            raise SimulatorException("Could not connect")

        self.clientID = clientID   
        return self.clientID     
    
    def test_connection(self,):
        """Tests if the connection was successful. If so, the number of objects in the scene is 
           returned.
        """
        res, objs = sim.simxGetObjects(self.clientID,
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
            
            returnCode = sim.simxWriteStringStream(self.clientID,
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
        error_obj, handle_obj = sim.simxGetObjectHandle(clientID=self.clientID,
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
            current_obj_handle = self.get_object_handle(client_id=self.clientID, 
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
        position_error, position_vector = sim.simxGetObjectPosition(self.clientID,
                                                                    object_handle,
                                                                    -1, 
                                                                    sim.simx_opmode_blocking)
        angle_error, angle_vector = sim.simxGetObjectOrientation(self.clientID,
                                                                 object_handle,
                                                                 -1, 
                                                                 sim.simx_opmode_blocking)
        if position_error+angle_error != 0:
            raise SimulatorException('an inexpected error occurred when configuration were retrieved')
        return position_vector, angle_vector

    ################# FUNCTIONS FOR PROJECT 2, GOAL 1 #################################
    def get_bounding_box(self, object_handle, frame='global'): # get_bounding_box_corners_local_coordinates
        if frame != 'global' and frame != 'local':
            raise ValueError(f'Wrong Value of Frame')  
        else:
            pass
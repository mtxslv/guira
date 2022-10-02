import ctypes
import time

from guira import sim # simulation lib
from guira.guira_exceptions import SimulatorException 

class Scene:
    def __init__(self,):
        """Scene instancing. 
        """
        pass

    def connect_to_sim(self,):
        """Close all opened connections and connects the code with the simulator API,
           if it is already running.

        Raises:
            SimulatorException: Raised if not successfully connected.
        
        Returns:
            clientID(int): the client ID.
        """

        sim.simxFinish(-1) # just in case, close all opened connections

        clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

        if clientID != -1:
            print('Connected to remote API server')
        else:
            raise SimulatorException("Could not connect")

        self.clientID = clientID   
        return self.clientID     
    
    def test_connection(self,):
        """Tests if the connection was successful. If so, the number of objects in the scene is 
           displayed on the terminal.
        """
        res, objs = sim.simxGetObjects(self.clientID,
                                       sim.sim_handle_all,
                                       sim.simx_opmode_blocking)
                                
        if res==sim.simx_return_ok:
            print('Number of objects in the scene: ',len(objs))
        else:
            print('Remote API function call returned with error code: ',res)

    def send_points_to_sim(self, points, sleep_time = 0.07):
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
            client_id (int): an ID related to the running simulation
            object_name (string): object's name as seen in the hierarchy tree.

        Raises:
            RuntimeError: this error is raised when any of the error codes is not zero (returned when the handle is retrieved).

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
            client_id (int): an ID related to the running simulation
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
        """This function return compilled information regarding the scene's objects.

        Args:
            client_id (int):  an ID related to the running simulation
            scene_objects (list): a list of scene objects' names whose handlers should be retrieved.

        Returns:
            info_list (list of dictionary): a list of dictionary. Each dictionary has the object name of the object, its handler, and its configuration (position and orientation).
        """
        info_list = []
        for element in scene_objects:
            current_obj_handle = self.get_object_handle(client_id=self.clientID, 
                                                object_name=element)
            position, angle = self.get_configuration(client_id=self.clientID,
                                                object_handle=current_obj_handle)
            current_dict = {'object_name': element,
                            'object_handler': current_obj_handle,
                            'object_position': position,
                            'object_orientation':angle}
            info_list.append(current_dict)
        return info_list

    def get_configuration(self, object_handle):
        """This function returns the object configuration (position and orientation vectors).

        Args:
            client_id (int): an ID related to the running simulation
            object_handle (int): an ID related to the simulated object whose configuration will be retrieved 

        Raises:
            RuntimeError: this error is raised when any of the error codes is not zero (returned when the vectors are retrieved).

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

    def get_robot_from_info_list(self,info_list:list, object_name):
        """This function separates the list of objects' dictionary from a object dictionary. 
           It does not alter the original list variable. 
           Notice info_list_copy is a list containing information about the obstacles.

        Args:
            info_list (list): robot info, the output of get_scene_objects_info()
            obj_name (string): the object_name (dictionary keyword) 

        Raises:
            KeyError: raised when the robot_name is not found in info_list

        Returns:
            (tuple): the list of dictionaries containing objects' information and the dictionary containing the robot information, in this order.
        """
        info_robot = info_list[0] # this assignment has no effect in code. It is meant for assigning a value to info_robot outside of the for loop scope 
        obj_key_was_not_found = True
        info_list_copy = info_list.copy()
        for element in info_list_copy:
            if element['object_name'] == object_name:
                obj_key_was_not_found = False    
                info_robot_position = info_list_copy.index(element)
                info_robot = element
                del info_list_copy[info_robot_position]
        if obj_key_was_not_found:
            raise KeyError("Object name was not found")
        return info_list_copy, info_robot
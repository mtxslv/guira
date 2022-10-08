from guira import sim # simulation lib
from guira.guira_exceptions import SimulatorException 

class Robot:
    def __init__(self,
                 left_motor_handle:int, 
                 right_motor_handle:int,
                 client_id: int):

        self.left_motor_handle = left_motor_handle
        self.right_motor_handle = right_motor_handle
        self.client_id = client_id

    def run(self,left_target_velocity, right_target_velocity):
        return_value_left_motor_control = sim.simxSetJointTargetVelocity(clientID=self.client_id,
                                                                         jointHandle=self.left_motor_handle,
                                                                         targetVelocity=left_target_velocity,
                                                                         operationMode=sim.simx_opmode_blocking)
        return_value_right_motor_control = sim.simxSetJointTargetVelocity(clientID=self.client_id,
                                                                          jointHandle=self.right_motor_handle,
                                                                          targetVelocity=right_target_velocity,
                                                                          operationMode=sim.simx_opmode_blocking)                                 
        return return_value_left_motor_control,return_value_right_motor_control

    def get_robot_handles(self,):
        pass
    def get_configuration(self,):
        pass
    def get_bounding_box_corners_positions(self,):
        pass
from guira import sim # simulation lib
from guira.guira_exceptions import SimulatorException 

class Robot:
    def __init__(self,
                 left_motor_handle:int, 
                 right_motor_handle:int,
                 client_id: int):
        """A mobile differential-wheeled robot instance.

        Args:
            left_motor_handle (int): The handle of the left motor;
            right_motor_handle (int): The handle of the right motor;
            client_id (int): The client id associated with the running simulation.
        """

        self.left_motor_handle = left_motor_handle
        self.right_motor_handle = right_motor_handle
        self.client_id = client_id

    def run(self,left_target_velocity, right_target_velocity):
        """Run a mobile robot by asserting each differential wheel a target velocity.

        Args:
            left_target_velocity (float): The left wheel target velocity;
            right_target_velocity (float): The right wheel target velocity.

        Returns:
            return_value_left_motor_control,return_value_right_motor_control (tuple): simxSetJointTargetVelocity return value. -1 if operation was not successful.
        """
        return_value_left_motor_control = sim.simxSetJointTargetVelocity(clientID=self.client_id,
                                                                         jointHandle=self.left_motor_handle,
                                                                         targetVelocity=left_target_velocity,
                                                                         operationMode=sim.simx_opmode_blocking)
        return_value_right_motor_control = sim.simxSetJointTargetVelocity(clientID=self.client_id,
                                                                          jointHandle=self.right_motor_handle,
                                                                          targetVelocity=right_target_velocity,
                                                                          operationMode=sim.simx_opmode_blocking)                                 
        return return_value_left_motor_control,return_value_right_motor_control
import numpy as np

from guira import sim # simulation lib

class Ultrassonic():
    def __init__(self, client_id : int, sensor_handle: int):
        """An Ultrassonic Sensor object.

        Args:
            client_id (int): the scene's client id;
            sensor_handle (int): the sensor's handle.
        """
        self.client_id = client_id
        self.sensor_handle = sensor_handle
        
    def read(self):
        """Returns the distance read by the sensor.

        Returns:
            distance (float): the distance read by the sensor.
        """
        _, _, detected_point, _, _ = sim.simxReadProximitySensor(clientID=self.client_id, 
                                                                 sensorHandle=self.sensor_handle, 
                                                                 operationMode=sim.simx_opmode_blocking)
        distance = np.sqrt(detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2)
        return distance
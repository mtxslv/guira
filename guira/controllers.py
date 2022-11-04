class PositionController():
    """The Position Controller decouple input signals in order 
       to explicit the angular and linear errors. The position
       is controlled by minimizing such errors.
    """
    def __init__(self, 
                 linear_coef: float = 0.3,
                 angular_coef: float = 0.2,
                 left_wheel_radius: float = 0.0423005,
                 right_wheel_radius: float = 0.0423005,
                 distance_between_wheels: float = 0.214):
        """

        Args:
            linear_coef (float, optional): _description_. Defaults to 0.3.
            angular_coef (float, optional): _description_. Defaults to 0.2.
            left_wheel_radius (float, optional): The radius of the robot's left wheel. Defaults to 0.0423005.
            right_wheel_radius (float, optional): The radius of the robot's right wheel. Defaults to 0.0423005.
            distance_between_wheels (float, optional): Distance between the wheels. Defaults to 0.214.
        """
        self.linear_coef = linear_coef
        self.angular_coef = angular_coef
        self.left_wheel_radius = left_wheel_radius
        self.right_wheel_radius = right_wheel_radius
        self.distance_between_wheels = distance_between_wheels

    def set_positions(self, positions: list):
        pass
    def control(self):
        pass
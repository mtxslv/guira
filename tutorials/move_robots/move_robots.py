# This example shows basic robot motion
# with DR20 and P3DX.
# In order to execute this tutorial, go
# in Model Browser and browse the 
# robot/mobile robots folder. Drag both
# DR20 and P3DX into the work screen.
# Go in Tools > Scripts. In the pop up 
# screen disable both DR20 and P3DX 
# scripts by clicking in the script 
# property checkbox 'Disabled'.

import time

from guira.scene import Scene
from guira.robot import Robot

def main():
    scene = Scene()
    client_id = scene.connect()
    how_many_objs = scene.test_connection()
    if how_many_objs > 0:
        # dr20 robot definition
        dr20_left_motor_handle = scene.get_object_handle("./leftWheelJoint_")
        dr20_right_motor_hanle = scene.get_object_handle("./rightWheelJoint_")
        dr20 = Robot(dr20_left_motor_handle,
                     dr20_right_motor_hanle, 
                     client_id) 
                                       
        # p3dx robot definition
        p3dx_left_motor_handle = scene.get_object_handle('./leftMotor')
        p3dx_right_motor_handle = scene.get_object_handle('./rightMotor')
        p3dx = Robot(p3dx_left_motor_handle,
                     p3dx_right_motor_handle,
                     client_id)
        
        # robots go to the center of the scene
        dr20.run(3,3)
        p3dx.run(1,1)
        time.sleep(5)

        # robots go back to where they were
        dr20.run(-3,-3)  
        p3dx.run(-1,-1)
        time.sleep(5)

        # robots spin forever
        dr20.run(-3,3)  
        p3dx.run(1,-1)


if __name__ == "__main__":
    main()

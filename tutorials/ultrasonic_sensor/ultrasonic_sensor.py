# copy connecting_to_a_scene.ttt object into this 
# folder. Rename it 'ultrasonic_sensor.ttt'. Put 
# a 1 x 1 x 1 cuboid somewhere in the scene and 
# make it Detectable, Collidable and Measurable. 
# Put a P3DX robot in the scene and disable its
# script. Rotate it such that the 3rd and 4th 
# ultrasonic sensors will be facing the cuboid. 

from time import sleep

from guira.scene import Scene
from guira.robot import Robot
from guira.sensors import Ultrasonic

def main():
    scene = Scene()
    client_id = scene.connect()
    how_many_objs = scene.test_connection()
    if how_many_objs > 0:
        # p3dx robot definition
        p3dx_left_motor_handle = scene.get_object_handle('./leftMotor')
        p3dx_right_motor_handle = scene.get_object_handle('./rightMotor')
        p3dx = Robot(p3dx_left_motor_handle,
                     p3dx_right_motor_handle,
                     client_id)

        # ultrasonic sensors definition
        ultrasonic_3rd_handle = scene.get_object_handle('/PioneerP3DX/visible/ultrasonicSensor[3]')
        ultrasonic_4th_handle = scene.get_object_handle('./PioneerP3DX/ultrasonicSensor[4]')  

        ultrasonic_3rd = Ultrasonic(client_id = client_id,
                                      sensor_handle = ultrasonic_3rd_handle)

        ultrasonic_4th = Ultrasonic(client_id = client_id,
                                      sensor_handle = ultrasonic_4th_handle)
        while True:
            readings_avg = (ultrasonic_3rd.read() + ultrasonic_4th.read())/2  
            print(f'Averaged Readings: {readings_avg}')
            if readings_avg < 0.75:
                p3dx.run(-1,-1)
                print(f'Moving backwards')
            elif readings_avg > 3:
                p3dx.run(1,1)
                print(f'Moving onwards')
            sleep(1)


if __name__ == '__main__':
    main()
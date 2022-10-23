import numpy as np
import pytest

from guira.utils import map_points_to_global, local_to_global, rotation_matrix_from_euler_angles

@pytest.mark.parametrize('if_inverse,return_rotation_matrix',[
    (False,[[0, -1, 0],
            [1,  0, 0],
            [0,  0, 1]]),
    (True,[[ 0,  1,  0],
           [-1,  0,  0],
           [ 0,  0,  1]])
])
def test_rotation_matrix(if_inverse,return_rotation_matrix):
    phi = 3.14/2 # anti clockwise 90º rotation around Z axis
    theta = 0
    psi = 0
    rotation_matrix = rotation_matrix_from_euler_angles(phi = phi,
                                                        theta = theta,
                                                        psi = psi,
                                                        inverse=if_inverse)
    mat = np.round(rotation_matrix, 2)
    expected_matrix = np.array(return_rotation_matrix)
    assert ( mat == expected_matrix ).all()

@pytest.mark.parametrize('original_vector, euler_angles, frame_origin, final_vector',[
    ([1,0,0],
     {'phi':3.14/2,'theta':0,'psi':0},
     [0,0,0],
     [0, 1, 0]),
    ([[1],[1],[0]],
     {'phi':3.14,'theta':0,'psi':0}, 
     [[0],[10],[0]], 
     [[-1.],[ 9.],[ 0.]])
])
def test_local_to_global(original_vector, euler_angles,frame_origin, final_vector):
    phi = euler_angles['phi']
    theta = euler_angles['theta']
    psi = euler_angles['psi']
    point_in_original_frame = np.array(original_vector)
    rotated_vector = local_to_global(point_in_object_frame=point_in_original_frame,
                                     euler_angles=[phi,theta,psi],
                                     frame_origin_position=frame_origin)
    rotated_vector = np.round(rotated_vector,1)                                     
    expected_vector = np.array(final_vector)
    assert ( rotated_vector == expected_vector ).all() 

def test_map_points_to_global():
    local_coordinates = [[ 1,  0, 0],
                         [ 0,  1, 0],
                         [-1,  0, 0],
                         [ 0, -1, 0]]
    final_coordinates = np.array([[ 0.707,  0.707, 0],
                                  [-0.707,  0.707, 0],
                                  [-0.707, -0.707, 0],
                                  [ 0.707, -0.707, 0]])
    ans = map_points_to_global(local_coordinates=local_coordinates,
                               euler_angles=[3.14/4,0,0],
                               frame_origin_position=[0,0,0])      
    assert (ans == final_coordinates).all()
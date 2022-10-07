# Send points to the Sim

# For this tutorial:
# 1. Just open the scene sending_points.ttt on Coppelia
# 2. Run this file
# Or...
# 1. Prepare the Sim to accept a connection
# (i.e., add 'simRemoteApi.start(19999)' at the end of the
# scene's main script). 
# 2. Add a associated non-threaded Lua child script on 
# DefaultLights object (you can do it by right-clicking on
# the object, and following Add -> Associated Child Script
# -> Non-Threaded -> Lua).
# 3. Copy the receiving_script.lua content and paste on 
# the associated child script.
# 4. Run the simulation and then run this file.

import math

from guira.scene import Scene
from guira.curves import CubicPolynomials, Lemniscate

def main():
    scene = Scene()
    scene.connect()
    how_many_objs = scene.test_connection()
    print(f'There are {how_many_objs} objects in the scene') # 15 objects

    lem = Lemniscate(2.0,2.0)

    top_left = CubicPolynomials([0,0,math.pi],
                               [-2.5, 2.5, 0.75*math.pi])
    top_right = CubicPolynomials([0,0,0],
                               [2.5, 2.5, 0.25*math.pi])   
    bottom_left = CubicPolynomials([0,0,math.pi],
                               [-2.5, -2.5, 1.25*math.pi])
    bottom_right = CubicPolynomials([0,0,0],
                               [2.5, -2.5, -0.25*math.pi]) 

    lem_points, _, _, _ =  lem.get_curve_points(300)                                                                                                                        
    
    points_top_left = top_left.get_curve_points(50)
    points_top_right = top_right.get_curve_points(50)
    points_bottom_right = bottom_right.get_curve_points(50)
    points_bottom_left = bottom_left.get_curve_points(50)

    scene.send_points_to_sim(lem_points)
    scene.send_points_to_sim([p[1:3] for p in points_top_left])
    scene.send_points_to_sim([p[1:3] for p in points_top_right])
    scene.send_points_to_sim([p[1:3] for p in points_bottom_left])
    scene.send_points_to_sim([p[1:3] for p in points_bottom_right])

if __name__ == "__main__":
    main()

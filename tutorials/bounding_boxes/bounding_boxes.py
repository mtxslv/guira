# Copy move_robots.ttt and rename it mapping.ttt
# Remove p3dx robot.
# Move dr20 to the 9th column, between the 5th and 6th row (from top left)
# (On Scene Hierarchy) Add -> Primitive Shape -> Cuboid (size 1e0 x 1e0 x 1e0)
# Rename it to BigCuboid
# The cube will appear on the center
# Click on the cube
# Go Tools -> Scene Object Properties -> Common (on the pop up window, top right) 
# Mark collidable, detectable, measurable
# Repeat the same process to a (0.5e0 x 0.5e0 x 0.5e0) cube at 4|8. Call it SmallCuboid
# Rotate the SmallCuboid such its X axis points towards the negative global X axis.
# Close the pop up window

import numpy as np
from guira.scene import Scene

def main():
    scene = Scene()
    scene.connect()
    info_list = scene.get_scene_objects_info(['dr20','BigCuboid','SmallCuboid'])
    for it in range(0,len(info_list)):
        position = np.round(info_list[it]["position"],2)
        orientation = np.round(info_list[it]["orientation"],2)/np.pi
        print(f'Information about {info_list[it]["name"]}: \n\t handler:{info_list[it]["handler"]}\n\t position (l.u.):{position}\n\t orientation (pi rad) :{orientation}')
        g_c = scene.get_bounding_box(object_handle=info_list[it]['handler'], frame='global') # global corners
        u_r = np.round(g_c[0][:2],2) # upper right
        u_l = np.round(g_c[1][:2],2) # upper left
        b_l = np.round(g_c[2][:2],2) # bottom left
        b_r = np.round(g_c[3][:2],2) # bottom right 
        print(f"\t global corners (l.u.):\n\t\t {u_r} \n\t\t {u_l} \n\t\t {b_l} \n\t\t {b_r}")

    print('l.u.: length units')

if __name__ == '__main__':
    main()
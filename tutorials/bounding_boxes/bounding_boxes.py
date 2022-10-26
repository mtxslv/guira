# Copy move_robots.ttt and rename it mapping.ttt
# Remove p3dx robot.
# Move dr20 to the 9|4 square (from top left)
# (On Scene Hierarchy) Add -> Primitive Shape -> Cuboid (size 1e0 x 1e0 x 1e0)
# Rename it to BigCuboid
# The cube will appear on the center
# Click on the cube
# Go Tools -> Scene Object Properties -> Common (on the pop up window, top right) 
# Mark collidable, detectable, measurable
# Repeat the same process to a (0.5e0 x 0.5e0 x 0.5e0) cube at 4|8. Call it SmallCuboid
# Close the pop up window

from guira.scene import Scene

def main():
    scene = Scene()
    client_id = scene.connect()
    info_list = scene.get_scene_objects_info(['dr20','BigCuboid','SmallCuboid'])
    dr20_info = info_list[0]
    info_list.pop(0)
    cuboids_info = info_list
    print(f'Information about Robot {dr20_info["name"]}: \n\t handler:{dr20_info["handler"]}\n\t position:{dr20_info["position"]}\n\t orientation:{dr20_info["orientation"]}')
    for it in range(0,len(cuboids_info)):
        print(f'Information about {cuboids_info[it]["name"]}: \n\t handler:{cuboids_info[it]["handler"]}\n\t position:{cuboids_info[it]["position"]}\n\t orientation:{cuboids_info[it]["orientation"]}')
    a = scene.get_bounding_box(object_handle=dr20_info['handler'],
                           frame='local')
    print(a)

if __name__ == '__main__':
    main()
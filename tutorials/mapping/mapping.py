# Copy move_robots.ttt and rename it mapping.ttt
# Remove p3dx robot.
# Move dr20 to the 9,4 square (from top left)
# (On Scene Hierarchy) Add -> Primitive Shape -> Cuboid (size 1e0 x 1e0 x 1e0)
# Rename it to BigCuboid
# The cube will appear on the center
# Click on the cube
# Go Tools -> Scene Object Properties -> Common (on the pop up window, top right) 
# Mark collidable, detectable, measurable
# Close the pop up window

from guira.scene import Scene

def main():
    scene = Scene()
    client_id = scene.connect()
    info_list = scene.get_scene_objects_info(['dr20','BigCuboid'])
    dr20_info = info_list[0]
    bigcuboid_info = info_list[1]
    print(f'Information about Robot {dr20_info["name"]}: \n\t handler:{dr20_info["handler"]}\n\t position:{dr20_info["position"]}\n\t orientation:{dr20_info["orientation"]}')
    print(f'Information about {bigcuboid_info["name"]}: \n\t handler:{bigcuboid_info["handler"]}\n\t position:{bigcuboid_info["position"]}\n\t orientation:{bigcuboid_info["orientation"]}')
    

if __name__ == '__main__':
    main()
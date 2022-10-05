# To run this tutorial do:
# Open the Scene connecting_to_the_scene.ttt
# Or...
# 1. Open a new scene in CoppeliaSim Edu V4.3.0
# 2. At the end of the scene's main script, add: simRemoteApi.start(19999)
# 3. Run the simulation

from guira.scene import Scene

def main():
    scene = Scene()
    scene.connect()
    how_many_objs = scene.test_connection()
    print(f'There are {how_many_objs} objects in the scene') # 15 objects

if __name__ == "__main__":
    main()

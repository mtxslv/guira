![Guira Cuckoo, by David Schenfeld](https://live.staticflickr.com/8117/8621923793_67f83e3a32_b.jpg)
# Welcome! 👋👋

Guira is a Python Library for Robotic Simulation with CoppeliaSim (EDU 4.3.0). It works as an interface to CoppeliaSim's Remote API, simplifying the basic use of the simulator.

# 🤔 What does this lib do? 🤔

Guira is meant to process CoppeliaSim's basic information, such that you can focus on your robotic algorithms. Here it goes a full list of Guira's capabilities:

- Module Curves:
    - Create a parametric curve (Cubic Polynomials or Lemniscate);
    - Generate points over those curves. 
- Module Robot:
    - Actuate the robot's motors to drive it around.
- Module Scene:
    - Connect to a simulation scene;
    - Test if connection is ok;
    - Send points to simulator;
    - Get an object's (or a set of objects') handlers;
    - Retrieve an object's configuration (position and orientation);
    - Retrieve all objects' configurations;
    - Get an object's bounding box corners' points.
- Module Sensors:
    - Read an ultrasonic sensor.

#  👩‍💻 How to install? 👨‍💻

In order to install the lib in a Linux based system, run:

```pip install git+https://github.com/mtxslv/guira```.

If you have Poetry available in your system, you can do:

```poetry add git+https://github.com/mtxslv/guira```.

# It seems amazing 🤩! But I have no idea how to start 😓. What should I do? 🧐

The folder `tutorials/` contains a lot of examples (each one shows an application). The folder contains both the code and the scene. You can open the scene with CoppeliaSim and run the simulation. Then you run the code with a python interpreter (the lib needs python 3.9 at least) and that is it! Here it goes a list of available examples:

* `bounding_boxes` shows how to retrieve objects configuration, as well as the bounding boxes' corners positions;
* `connecting_to_a_scene` explains how to connect to a simulation scene;
* [`moving_robots`](https://youtu.be/07_oNcmhMhY) teach you how to drive a robot;
* [`sending_points`](https://youtu.be/M4mCUekoWF8) exemplify the use of _Curves_ and _Scene_ package by generating a curve and sending some samples to the simulated scene;
* `ultrasonic_sensor` shows how to read an ultrasonic sensor.

## 👀 Ok, but I don't want to download it by hand... 👀

Ok, you have installed the lib. Now you want to get the tutorials to starting playing around. You don't need to clone the repo or download each file manually. [There is a script for that](https://github.com/mtxslv/guira/blob/master/get_tutorials.sh). You can download the `get_tutorials` shell script directly or you can get it by running

```shell
curl https://raw.githubusercontent.com/mtxslv/guira/master/get_tutorials.sh --output get_tutorials.sh
```

in a terminal. Now you have the script responsible for copying the [./tutorials/](https://github.com/mtxslv/guira/tree/master/tutorials) folder available in your machine. 

In order to mount the folder locally, allow the system to execute the script by running

```shell
chmod +x get_tutorials.sh
```

and run the script itself:

```shell
./get_tutorials.sh
```

**This will mount the tutorials in the folder you executed it**. So, if you are running the script inside _~/Videos_, the tutorials will be available in _~/Videos/tutorials/_.

# ✍🏼 Some Last Words... ✍🏼

Your feedback is much appreciated 🫂. If you developed something interesting using the lib, please consider showing the world. I introduced the lib in a [Medium post](https://medium.com/@mateus.d.assis.silva/learning-robotics-made-easier-536ec16a4b8a) and in [Linkedin](https://www.linkedin.com/posts/mateus-assis-013a46140_learning-robotics-made-easier-activity-7020390171539836929-mCQM?utm_source=share&utm_medium=member_desktop). If you want to share some thoughts, these places are great for it. 

If you want to share your own project, don't be shy! You can tag [my Linkedin account](https://www.linkedin.com/in/mateus-assis-013a46140/) or [my Github](https://github.com/mtxslv/). If you find any bugs, or unexpected behaviour, submit an issue in the project. I'll answer it ASAP 😉

#

_**That said, happy coding!**_

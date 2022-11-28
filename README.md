# Welcome! 👋👋

Guira is a Python Library for Robotic Simulation with CoppeliaSim (EDU 4.3.0).

# 🤔 What does this lib do? 🤔

Guira is meant to process CoppeliaSim's basic information, such that you can focus on your robotic algorithms. Here it goes a full list of Guira's capabilities:
- Class Scene:
    - Connect to a simulation scene;
    - Test if connection is ok;
    - Send points to simulator;
    - Get an object's (or a set of objects') handlers;
    - Retrieve an object's configuration (position and orientation);
    - Retrieve all objects' configurations;
    - Get an object's bounding box corners' points.
- Class Robot:
    - Actuate the robot's motors to drive it around.
- Class Ultrasonic:
    - Read an ultrasonic sensor.

# It seems amazing 🤩! But I have no idea how to start 😓. What should I do? 🧐

The folder `tutorials/` contain a lot of examples (each one shows an application). The folder contains both the code and the scene. You can open the scene with CoppeliaSim and run the simulation. Then you run the code with a python interpreter (the lib needs python 3.9 at least) and that is it! Here it goes a list of available examples:

* `connecting_to_a_scene` explains how to connect to a simulation scene;
* `bounding_boxes` shows how to retrieve objects configuration, as well as the bounding boxes' corners positions;
* `moving_robots` teach you how to drive a robot;
* `ultrasonic_sensor` shows how to read an ultrasonic sensor.

#  👩‍💻 How to install? 👨‍💻

In order to install the lib in a Linux based system, run:

```pip install git+https://github.com/mtxslv/guira```.

If you have Poetry available in your system, you can do:

```poetry add git+https://github.com/mtxslv/guira```.


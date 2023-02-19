#!/bin/bash

# Downloads the tutorials (scenes and scripts).
# First, generate a folder structure like the one in github.
# Then download the scripts, and then the scenes.
# At last, display the final tree structure.

# python files to be downloaded
text_files=('https://raw.githubusercontent.com/mtxslv/guira/master/tutorials/bounding_boxes/bounding_boxes.py'
            'https://raw.githubusercontent.com/mtxslv/guira/master/tutorials/connecting_to_a_scene/connecting_to_a_scene.py'
            'https://raw.githubusercontent.com/mtxslv/guira/master/tutorials/move_robots/move_robots.py'
            'https://raw.githubusercontent.com/mtxslv/guira/master/tutorials/sending_points/receiving_script.lua'
            'https://raw.githubusercontent.com/mtxslv/guira/master/tutorials/sending_points/sending_points.py'
            'https://raw.githubusercontent.com/mtxslv/guira/master/tutorials/ultrasonic_sensor/ultrasonic_sensor.py')

# local text files
file_names=('./tutorials/bounding_boxes/bounding_boxes.py'
            './tutorials/connecting_to_a_scene/connecting_to_a_scene.py'
            './tutorials/move_robots/move_robots.py'
            './tutorials/sending_points/receiving_script.lua'
            './tutorials/sending_points/sending_points.py'
            './tutorials/ultrasonic_sensor/ultrasonic_sensor.py')

# simulation scenes to be downloaded
simulation_scenes=('https://github.com/mtxslv/guira/blob/master/tutorials/bounding_boxes/bounding_boxes.ttt'
                   'https://github.com/mtxslv/guira/blob/master/tutorials/connecting_to_a_scene/connecting_to_a_scene.ttt'
                   'https://github.com/mtxslv/guira/blob/master/tutorials/move_robots/move_robots.ttt'
                   'https://github.com/mtxslv/guira/blob/master/tutorials/sending_points/sending_points.ttt'
                   'https://github.com/mtxslv/guira/blob/master/tutorials/ultrasonic_sensor/ultrasonic_sensor.ttt')

# local simulation scenes
simulation_names=('./tutorials/bounding_boxes/bounding_boxes.ttt'
                  './tutorials/connecting_to_a_scene/connecting_to_a_scene.ttt'
                  './tutorials/move_robots/move_robots.ttt'
                  './tutorials/sending_points/sending_points.ttt'
                  './tutorials/ultrasonic_sensor/ultrasonic_sensor.ttt')
                  
# folders
folder_structure=('bounding_boxes'
                  'connecting_to_a_scene'
                  'move_robots'
                  'sending_points'
                  'ultrasonic_sensor')

echo 'Creating folder structure...'
mkdir 'tutorials/'
for index in "${!folder_structure[@]}";
do
    mkdir "tutorials/${folder_structure[$index]}"
done 

echo 'Downloading text files...'

for index in "${!text_files[@]}";
do
    curl ${text_files[$index]} --output ${file_names[$index]}
done 

echo 'Downloading the simulation scenes...'

for index in "${!simulation_scenes[@]}";
do
    curl ${simulation_scenes[$index]} --output ${simulation_names[$index]}
done 

clear
echo 'Done!'
echo ';)'
tree ./tutorials
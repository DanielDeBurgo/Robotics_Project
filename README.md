Instructions for running solution (after gazebo and simulated localisation are running):

1) Start a singularity shell
2) cd into your catkin workspace
3) catkin_make
4) source devel/setup.sh
5) change directory into the find_cluedo package
6) cd launch
7) roslaunch find_cluedo.launch world:= <path-to-directory-containing-input_points.yaml>

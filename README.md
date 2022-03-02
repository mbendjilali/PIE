# PIE
Python version of the RRT-Star algorithm in a 3D environment. This code is meant to be run in a ROS environment.

You do not need to touch to ClassDefinition.py nor Geometry.py, except if you want to use a line-cube collision check
instead of a line-sphere one.

PathPlanning.py implements both RRT-Star and RRT-Connect, although RRT-Connect is still buggy and will be fixed soon.
It also uses a simple process to retrieve the correct path from start to goal when a tree is successfully generated.

main.py just loads an environment for the simulation to run, plots it, and lets you choose between RRT-Star and
RRT-Connect.

#!/usr/bin/env python

print('\x1B[1;31mThis script has been deprecated.\x1B[0m')
print('\x1B[1;31mPlease use the following command.\x1B[0m')
print('\x1B[1;32m> rosrun planning_simulator_launcher scenario_roslaunch_server.py\x1B[0m')

import os
import sys

os.system('rosrun planning_simulator_launcher scenario_roslaunch_server.py ' + ' '.join(sys.argv[1:]))

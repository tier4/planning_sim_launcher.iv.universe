#!/usr/bin/env python

print('\x1B[1;31mThis script has been deprecated.\x1B[0m')
print('\x1B[1;31mPlease use the following command.\x1B[0m')
print('\x1B[1;32m> ros2 run planning_simulator_launcher show_result.py\x1B[0m')

import os

os.system('ros2 run planning_simulator_launcher show_result.py')

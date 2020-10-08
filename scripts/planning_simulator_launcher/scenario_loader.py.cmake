#!/usr/bin/env python

import os
import rospkg


class ScenarioLoader:

    def __init__(self):
        print('Load Paths')

        self.package_name = '${PROJECT_NAME}'
        print('    Package Name =\x1B[36m'),
        print(self.package_name),
        print('\x1B[0m')

        self.package_source_directory = '${CMAKE_CURRENT_SOURCE_DIR}'
        print('    Package Source Directory =\x1B[36m'),
        print(self.package_source_directory),
        print('\x1B[0m')

        self.package_install_directory = rospkg.RosPack().get_path(
                self.package_name)
        print('    Package Install Directory =\x1B[36m'),
        print(self.package_install_directory),
        print('\x1B[0m')

        self.default_scenario_database_basename = 'scenario_database.json'
        self.default_scenario_database_pathname = os.path.join(
                self.package_source_directory,
                self.default_scenario_database_basename)
        print('    Default Scenario Database =\x1B[36m'),
        print(self.default_scenario_database_pathname),
        print('\x1B[0m')

        self.default_launch_basename = 'planning_simulator.launch'
        self.default_launch_pathname = os.path.join(
                self.package_source_directory, 'launch',
                self.default_launch_basename)
        print('    Default Launch File =\x1B[36m'),
        print(self.default_launch_pathname),
        print('\x1B[0m')

        print('')

    def absolute(self, path):
        if os.path.isabs(path):
            return path
        else:
            return os.path.join(self.package_source_directory, path)

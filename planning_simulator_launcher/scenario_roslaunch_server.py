#!/usr/bin/env python

from os.path import join, dirname
from scenario_parser import ScenarioParser
import SimpleXMLRPCServer as xmlrpc_server
import json
import os
import roslaunch
import signal
import sys
import time


class ScenarioRoslaunchServer():

    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        self.server = xmlrpc_server.SimpleXMLRPCServer(
                ("localhost", 10000), allow_none=True, logRequests=False)
        self.server.register_introspection_functions()
        self.parent = None
        self.simulation_runnig = False

        self.duration = 0
        self.exit_status = 42
        self.mileage = 0

    def signal_handler(self,signal,frame):
        sys.exit(0)

    def start(self,launch_path,scenario_path,scenario_id,map_path,log_output_dir, log_output_type):
        parser = ScenarioParser(scenario_path,generate_mode=False)
        arg_scenario_path = "scenario_path:=" + scenario_path
        arg_map_path = "map_path:=" + map_path
        arg_log_output_path = "log_output_dir:=" + log_output_dir
        if os.path.exists(log_output_dir) == False:
            os.makedirs(log_output_dir)
        arg_scenario_id = "scenario_id:=" + parser.getScenarioId()
        arg_log_output = "scenario_runner_output:=" + (log_output_type if log_output_type is not None else "screen")
        cli_args = [launch_path, arg_map_path, arg_scenario_path, arg_log_output_path, arg_scenario_id, arg_log_output]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        self.parent.start()
        self.simulation_runnig = True

    def simulationRunnig(self):
        return self.simulation_runnig

    def terminate(self, exit_status):
        if self.parent != None:
            self.exit_status = exit_status
            self.simulation_runnig = False
            self.parent.shutdown()
            self.parent = None

            # if self.exit_status == 0:
            #     print('    \x1b[32mSimulation succeeded\x1b[0m')
            # elif self.exit_status == 1:
            #     print('    \x1b[31mSimulation failed\x1b[0m')
            # else:
            #     print('    \x1b[31mSimulation failed (unexpected)\x1b[0m')

    def status(self):
        return self.exit_status

    def update_mileage(self, new_mileage):
        # print('update mileage from '),
        # print(self.mileage),
        # print('to'),
        # print(new_mileage)
        self.mileage = new_mileage

    def current_mileage(self):
        return self.mileage

    def update_duration(self, new_duration):
        # print('update duration from '),
        # print(self.duration),
        # print('to'),
        # print(new_duration)
        self.duration = new_duration

    def current_duration(self):
        return self.duration

    def run(self):
        self.server.register_function(self.current_duration)
        self.server.register_function(self.current_mileage)
        self.server.register_function(self.simulationRunnig)
        self.server.register_function(self.start)
        self.server.register_function(self.status)
        self.server.register_function(self.terminate)
        self.server.register_function(self.update_duration)
        self.server.register_function(self.update_mileage)
        self.server.serve_forever()


if __name__ == "__main__":
    launcher = ScenarioRoslaunchServer()
    launcher.run()

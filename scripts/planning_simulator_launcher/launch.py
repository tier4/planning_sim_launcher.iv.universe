#!/usr/bin/env python

from scenario_loader import ScenarioLoader
from scenario_parser import ScenarioParser
import argparse
import json
import os
import roslaunch
import sys
import time
import xmlrpclib


class Launcher:

    def __init__(self, args, loader):
        self.timeout = args.timeout
        self.log_output_type = args.log_output
        self.server = None
        self.client = xmlrpclib.ServerProxy('http://0.0.0.0:10000')
        self.loader = loader
        self.args = args

        print('Loading Databse')

        self.json_path = args.database
        sys.stdout.write('    Opening  \x1b[36m')
        sys.stdout.write(self.json_path)
        sys.stdout.write('\x1b[0m => ')

        with open(self.json_path) as file:
            print('\x1b[32mSuccess\x1b[0m')
            self.database = json.load(file)

    def map_path(self):
        sys.stdout.write('    load map \x1b[36m')
        sys.stdout.write(self.parser.getMapPath())
        sys.stdout.write('\x1b[0m as ')

        map_path = self.database['map'].get(
                self.parser.getMapPath(),
                self.parser.getMapPath());

        if map_path == self.parser.getMapPath():
            print('is')
        else:
            sys.stdout.write('\x1b[36m')
            sys.stdout.write(map_path)
            print('\x1b[0m')

        return self.loader.absolute(map_path)

    def launch_path(self):
        sys.stdout.write('    via \x1b[36m')

        path = None

        if self.args.launch:
            path = os.path.abspath(self.args.launch)
        else:
            path = self.loader.absolute(self.database["launch_path"])

        sys.stdout.write(path)
        print('\x1b[0m')
        return path

    def start(self, scenario_path):
        if self.args.log_output_base_dir:
            log_output_base_dir = os.path.abspath(self.args.log_output_base_dir)
        else:
            log_output_base_dir = self.loader.absolute(self.database["log_output_base_dir"])

        if not os.path.exists(log_output_base_dir):
            os.makedirs(log_output_base_dir)

        self.client.start(
            self.launch_path(),
            scenario_path,
            self.parser.getScenarioId(),
            self.map_path(),
            log_output_base_dir,
            self.log_output_type)

    def terminate(self):
        self.client.terminate(1)

    def waitUntilSimulationFinished(self):
        start = time.time()

        print('    The simulation is forced to terminate after'),
        print(str(self.timeout)),
        print('seconds.')

        while self.client.simulationRunnig() and (time.time() - start) < self.timeout:
            time.sleep(1)

        if self.client.simulationRunnig():
            print('    The specified maximum simulation time (option --timeout) has been reached.')
            print('    Terminate simulation.')
            print('    The scenario is likely to be in a deadlock.')
            self.terminate()

    def write_result(self, path, uuid):
        result = {}

        result['code']     = self.client.status()
        result['duration'] = self.client.current_duration()
        result['mileage']  = self.client.current_mileage()

        if self.client.status() == 0: # boost::exit_success
            result['message'] = 'exit_success'
            print('    \x1b[1;32m=> Success\x1b[0m')

        elif self.client.status() == 201: # boost::exit_test_failure
            result['message'] = 'exit_test_failure'
            print('    \x1b[1;31m=> Failure\x1b[0m')

        elif self.client.status() == 1: # boost::exit_failure
            result['message'] = 'exit_failure'
            print('    \x1b[1;31m=> Aborted\x1b[0m')

        elif self.client.status() == 200: # boost::exit_exception_failure
            result['message'] = 'exit_exception_failure'
            print('    \x1b[1;31m=> Invalid\x1b[0m')

        else:
            result['message'] = 'scenario_runner broken'
            sys.stdout.write('    \x1b[1;33m=> Broken (')
            sys.stdout.write(str(self.client.status()))
            print(')\x1b[0m')

        if self.args.log_output_base_dir:
            log_output_base_dir = os.path.abspath(self.args.log_output_base_dir)
        else:
            log_output_base_dir = self.loader.absolute(self.database["log_output_base_dir"])

        if not os.path.exists(log_output_base_dir):
            os.makedirs(log_output_base_dir)

        result_path = os.path.join(log_output_base_dir, 'result-of-' + uuid + '.json')

        with open(result_path, 'w') as file:
            json.dump(result, file, indent=2, ensure_ascii=False, sort_keys=True, separators=(',', ': '))

    def run_all_scenario(self):
        if self.args.scenario:
            scenarios = [ os.path.abspath(self.args.scenario) ]
        else:
            scenarios = self.database["scenario"]

        for scenario in scenarios:
            scenario_path = self.loader.absolute(scenario)

            print('')
            print('Running scenario \x1b[36m' + os.path.relpath(scenario_path) + '\x1b[0m')

            print('    Parameters')
            self.parser = ScenarioParser(scenario_path, generate_mode=True)

            n = 0

            for path in self.parser.scenario_files_path:
                n = n + 1

                print('')
                print('Launch ' + str(n) + ' of ' + str(len(self.parser.scenario_files_path)))

                print('    scenario \x1b[36m' + path + '\x1b[0m')

                self.start(path)

                self.waitUntilSimulationFinished()

                self.write_result(
                        scenario_path,
                        os.path.splitext(os.path.basename(path))[0])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='launch simulator')

    parser.add_argument('--timeout', type=int, default=180,
            help='Specify simulation time limit in seconds. \
                  The default is 180 seconds.')

    loader = ScenarioLoader()

    parser.add_argument('--database',
            default=os.path.relpath(loader.default_scenario_database_pathname),
            help='Specify relative path to your scenario_database.json.')

    parser.add_argument('--launch',
            help='Specify relative path to your launch file.')

    parser.add_argument('--log-output', default='screen',
            help='Specify the type of log output.')

    parser.add_argument('--log-output-base-dir',
            help='Specify the directory log output to.')

    parser.add_argument('--scenario',
            help='Specify the scenario you want to execute.')

    args = parser.parse_args()

    launcher = Launcher(args, loader)
    launcher.run_all_scenario()

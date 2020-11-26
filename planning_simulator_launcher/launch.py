import argparse
import json
from pathlib import Path
from typing import Optional, Union

import launch
from planning_simulator_launcher.launch_script import launch_description
from planning_simulator_launcher.scenario_parser import ScenarioParser


# If a base dir is given, resolves relative paths relative to the base dir.
# Otherwise, ensures that the path is absolute.
def absolute_path(path: Union[Path, str], base_dir: Optional[Path] = None):
    path = Path(path)
    if path.is_absolute():
        return path.resolve()
    if base_dir:
        return (base_dir / path).resolve()
    raise ValueError(f"Relative path '{path}' is not allowed.")


class Database:
    def __init__(
        self,
        database_path: Path,
        *,
        base_dir: Optional[Path] = None
    ):
        print('Loading Database')
        print(f'    Opening \x1b[36m{database_path}\x1b[0m => ', end='')
        with database_path.open() as file:
            database = json.load(file)
            print('\x1b[32mSuccess\x1b[0m')

        print('    Validating => ', end='')
        self.launch_path = absolute_path(database['launch_path'], base_dir)
        # Check if it exists, because the error message if we try to access it
        # later is not helpful.
        if not self.launch_path.is_file():
            raise ValueError(f"launch_path '{self.launch_path}' is not a file")

        self.log_output_base_dir = absolute_path(database['log_output_base_dir'], base_dir)

        self.map = {name: absolute_path(path, base_dir) for name, path in database['map'].items()}
        for path in self.map.values():
            if not path.is_file():
                raise ValueError(f"map '{path}' is not a file")

        self.scenario = [absolute_path(path, base_dir) for path in database['scenario']]
        for path in self.scenario:
            if not path.is_file():
                raise ValueError(f"scenario '{path}' is not a file")
        print('\x1b[32mSuccess\x1b[0m')


class Launcher:
    def __init__(self, args):
        # Only created for validation, doesn't need to be stored
        database = Database(args.database)

        # Only use database if command line arg is None
        self.launch_path = args.launch_path or database.launch_path
        self.log_output_base_dir = args.log_output_base_dir or database.log_output_base_dir
        # Similar, but wrap it in a list
        self.scenarios = [args.scenario] if args.scenario else database.scenario
        self.map = database.map
        self.timeout = args.timeout
        self.log_output_type = args.log_output
        self.vehicle_model = args.vehicle_model
        self.sensor_model = args.sensor_model

    # Look up the map name in the scenario in scenario_database.json
    def map_path(self, parser: ScenarioParser) -> Path:
        scenario_map_path = parser.getMapPath()
        print(f'    Loading map \x1b[36m{scenario_map_path}\x1b[0m as ',
              end='')

        # If the key is not found, return the key itself
        map_path = self.map.get(
            parser.getMapPath(),
            parser.getMapPath()
        )

        if map_path == parser.getMapPath():
            print('is')
        else:
            print(f'\x1b[36m{map_path}\x1b[0m')

        return absolute_path(map_path)

    def run_all_scenarios(self):
        # This allows running launch files from this script
        ls = launch.LaunchService()

        if not self.log_output_base_dir.is_dir():
            self.log_output_base_dir.mkdir(parents=True)

        for scenario_path in self.scenarios:
            print(f'\nRunning scenario \x1b[36m{scenario_path}\x1b[0m')
            print('    Parameters')
            parser = ScenarioParser(str(scenario_path), generate_mode=True)

            n_total = len(parser.scenario_files_path)
            for n, scenario_path in enumerate(parser.scenario_files_path, start=1):
                scenario_path = Path(scenario_path)
                print(f'\nLaunch {n} of {n_total}')
                print(f'    Scenario \x1b[36m{scenario_path}\x1b[0m')

                scenario_id = parser.getScenarioId()
                scenario_runner_args = {
                    'scenario_id': scenario_id,
                    'scenario_path': str(scenario_path),
                    'log_output_path': f'{self.log_output_base_dir}/{scenario_id}.json',
                    'json_dump_path': f'{self.log_output_base_dir}/result-of-{scenario_path.stem}.json',
                    'camera_frame_id': 'camera5/camera_optical_link',
                    'initial_engage_state': False
                }
                included_launch_file_args = {
                    'map_path': self.map_path(parser),
                    'log_dir': str(self.log_output_base_dir),
                    'sensor_model': self.sensor_model,
                    'vehicle_model': self.vehicle_model,
                    'initial_engage_state': False,
                }
                ld = launch_description(
                    launch_path=str(self.launch_path),
                    vehicle_model=self.vehicle_model,
                    scenario_runner_args=scenario_runner_args,
                    included_launch_file_args=included_launch_file_args
                )
                ls.include_launch_description(ld)
                ls.run()
                ls.shutdown()


def main():
    parser = argparse.ArgumentParser(description='launch simulator')
    parser.add_argument(
        '--timeout', type=int, default=180,
        help='Simulation time limit in seconds. The default is 180 seconds.')
    parser.add_argument(
        '--database',
        type=Path,
        required=True,
        help='Path to your scenario_database.json.')
    parser.add_argument(
        '--launch-path',
        type=absolute_path,
        help='Path to your launch file [overrides the value from scenario_database.json]')
    parser.add_argument(
        '--log-output',
        choices=['screen', 'log', 'both', 'own_log', 'full'],
        default='screen',
        help='Specify the type of log output.')
    parser.add_argument(
        '--log-output-base-dir',
        type=absolute_path,
        help='Log directory [overrides the value from scenario_database.json]')
    parser.add_argument(
        '--scenario',
        help='Scenario path [overrides the value from scenario_database.json]')
    parser.add_argument(
        '--vehicle_model',
        default='lexus',
        help='Vehicle model')
    parser.add_argument(
        '--sensor_model',
        default='aip_xx1',
        help='Sensor model')
    args = parser.parse_args()

    launcher = Launcher(args)
    launcher.run_all_scenarios()


if __name__ == "__main__":
    main()

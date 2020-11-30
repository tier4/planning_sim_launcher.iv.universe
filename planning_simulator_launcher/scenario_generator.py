import itertools
import json
import os
import re
import uuid
from copy import deepcopy

import yaml


class ScenarioGenerator:

    def __init__(self, params, path):
        self.params = params
        self.yaml_path = path

        self.num_scenarios = 0

        self.scenario_directory = os.path.dirname(
                os.path.abspath(self.yaml_path))

        scenario_name = os.path.splitext(os.path.basename(self.yaml_path))[0]

        self.generated_directory = self.scenario_directory + "/generated"

        if not os.path.exists(self.generated_directory):
            os.makedirs(self.generated_directory)

        self.generate_log = open(
                self.generated_directory + "/generate_log.json", 'w')

        with open(path, "r+") as file:
            self.yaml_str = file.read()

        self._yaml_generate_dir = self.generated_directory + "/" + scenario_name + "/"

        if os.path.exists(self._yaml_generate_dir):
            print('')
            print('Cleanup target directory ' + os.path.relpath(self._yaml_generate_dir))

            for each in os.listdir(self._yaml_generate_dir):
                print('    Removing ' + each)
                os.remove(os.path.join(self._yaml_generate_dir, each))
        else:
            os.makedirs(self._yaml_generate_dir)

        print('')
        print("Generating scenario " + scenario_name)
        self._generate()

    def __del__(self):
        self.generate_log.close()

    def getScenarioFilesPath(self):
        return self._yaml_files_path

    def _generate(self):
        self._yaml_files_path = []
        self.num_scenarios = 0

        print('    Parameters')

        for variable, configuration in self.params.items():
            print(f'        {variable} = {configuration.values}'),

        generate_logs = []

        if len(self.params):
            list_of_list_of_bindings = []

            for variable, configuration in self.params.items():
                list_of_bindings = []

                for value in configuration.values:
                    list_of_bindings.append([variable, value])

                list_of_list_of_bindings.append(list_of_bindings)

            for bindings in itertools.product(*list_of_list_of_bindings):
                scenario_id = str(uuid.uuid4())

                yaml_path = self._yaml_generate_dir + scenario_id + ".yaml"
                self._yaml_files_path.append(yaml_path)
                print('')
                print('    ' + os.path.relpath(yaml_path))

                result = deepcopy(self.yaml_str)

                for variable, value in bindings:
                    print(f'        {variable} => {value}'),
                    result = result.replace(variable, str(value))

                # NOTE: Expand macro after variable assignment (to use variable in expression)
                result = self.macroexpand(result)

                with open(yaml_path, 'w') as file:
                    yaml.dump(yaml.safe_load(result), file)

                self.num_scenarios = self.num_scenarios + 1

                param_dict = {}
                param_dict["name"] = variable
                param_dict["value"] = value

                generate_log = {}
                generate_log["id"] = scenario_id
                generate_log["param"] = param_dict
                generate_log["parent_path"] = self.yaml_path
                generate_log["path"] = yaml_path

                generate_logs.append(generate_log)
        else:
            scenario_id = str(uuid.uuid4())

            yaml_path = self._yaml_generate_dir + scenario_id + ".yaml"
            self._yaml_files_path.append(yaml_path)
            print('    ' + os.path.relpath(yaml_path))

            result = deepcopy(self.yaml_str)
            result = self.macroexpand(result)

            with open(yaml_path, 'w') as file:
                yaml.dump(yaml.safe_load(result), file)

            self.num_scenarios = 1

            generate_log = {}
            generate_log["id"] = scenario_id
            generate_log["parent_path"] = self.yaml_path
            generate_log["path"] = yaml_path

            generate_logs.append(generate_log)

        json.dump(generate_logs, self.generate_log)

    def _getDefaultValueKeys(self, used_key):
        variables = []

        for variable in self.params.keys():
            if variable != used_key:
                variables.append(variable)

        # print('       '),
        # print(variables),
        # print('=> default value')

        return variables

    def _update_default_params(self, name, yaml_string):
        for variable in self._getDefaultValueKeys(name):
            print(f'        {variable} => {self.params[variable].default_value} (default)'),
            yaml_string = yaml_string.replace(
                    variable, str(self.params[variable].default_value))

        return yaml_string

    def evaluate(self, match):
        return match.group(1) + str(eval(match.group(2))) + match.group(3)

    def macroexpand(self, source):
        regex = r"^(.*)\${{\s+((?:pi|[\d\s+\-\*/%\(\).])*)\s+}}(.*)$"
        return re.sub(regex, self.evaluate, source, flags=re.M)

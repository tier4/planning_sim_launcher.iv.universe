#!/usr/bin/env python

from scenario_generator import ScenarioGenerator
from scenario_parameter import ScenarioParameter
import os
import yaml


class ScenarioParser:

    def __init__(self, yaml_path, generate_mode=False):
        self.yaml_path = yaml_path

        with open(yaml_path, "r+") as f:
            self.yaml = yaml.load(f)

        if not self.parseScenarioParameter():
            print('    No parameter specified.')

        if generate_mode:
            generator = ScenarioGenerator(
                    self.scenario_params,
                    self.yaml_path)

            print('')
            print(str(generator.num_scenarios) + " Scenarios Generated")

            self.num_generated_scenarios = generator.num_scenarios
            self.scenario_files_path = generator.getScenarioFilesPath()

    def getScenarioId(self):
        basename = os.path.basename(self.yaml_path)
        return os.path.splitext(basename)[0]

    def getMapPath(self):
        return self.yaml["Map"]

    def parseScenarioParameter(self):
        self.scenario_params = {}
        try:
            for param in self.yaml["ParameterDeclaration"]:
                scenario_parameter = ScenarioParameter(param)
                if scenario_parameter.is_valid:
                    self.scenario_params[scenario_parameter.name] = scenario_parameter
                else:
                    print("Failed to load parameter: " + scenario_parameter.name)
                    return False
        except Exception as exception:
            # print(exception)
            return False

        return True


if __name__ == "__main__":
    pass

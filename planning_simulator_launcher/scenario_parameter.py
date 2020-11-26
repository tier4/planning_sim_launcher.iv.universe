#!/usr/bin/env python

import numpy as np


class ScenarioParameter:

    def __init__(self, yaml):
        self.yaml = yaml

        if 'Name' in yaml:
            self.name = yaml["Name"].strip()
            print('      - Name: ' + self.name)
        else:
            self.is_valid = False
            print('Error: Each parameter must have a name.')
            return

        if 'Min' in yaml:
            self.__min = yaml['Min']
        elif 'Value' in yaml:
            self.__min = yaml['Value']
        else:
            self.__min = 0

        print('        Min: ' + str(self.__min))

        if 'Max' in yaml:
            self.__max = yaml['Max']
        else:
            self.__max = self.__min

        print('        Max: ' + str(self.__max))

        if 'NumDivisions' in yaml:
            self.__num_divisions = yaml['NumDivisions']
        else:
            self.__num_divisions = 1

        print('        NumDivisions: ' + str(self.__num_divisions))

        self.values = np.linspace(self.__min, self.__max, self.__num_divisions)
        # print('        Values:'),
        # print(self.values)

        self.is_valid = True

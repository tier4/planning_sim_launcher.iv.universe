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
            self._min = yaml['Min']
        elif 'Value' in yaml:
            self._min = yaml['Value']
        else:
            self._min = 0

        print('        Min: ' + str(self._min))

        if 'Max' in yaml:
            self._max = yaml['Max']
        else:
            self._max = self._min

        print('        Max: ' + str(self._max))

        if 'NumDivisions' in yaml:
            self._num_divisions = yaml['NumDivisions']
        else:
            self._num_divisions = 1

        print('        NumDivisions: ' + str(self._num_divisions))

        self.values = np.linspace(self._min, self._max, self._num_divisions)
        # print('        Values:'),
        # print(self.values)

        self.is_valid = True

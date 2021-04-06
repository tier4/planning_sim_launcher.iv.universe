# Autoware Planning Simulator Launcher

Planning Simulator Launcher is being developed to assist in the definitive planning
simulation of Autoware.

Simulations are described in a YAML-based format called a "scenario".
The format has been specified at [Scenario Format Specification (TBU)](https://github.com/tier4/ScenarioFormat/blob/master/format/definition.md) .
This repository provides a reference implementation of the format's interpreter (Scenario Runner).

<br/>

## Contents

1. [Setup](#Setup)
1. [Usage](#Usage)
1. [Known Problems](#Known-Problems)
1. [For Developers](#For-Developers)
1. [Log file format](#Log-file-format)

<br/>

## 1. Setup

All the components needed to run the scenario, including this repository, are installed together in the AutowareArchitectureProposal setup.
See [https://github.com/tier4/AutowareArchitectureProposal#how-to-setup](https://github.com/tier4/AutowareArchitectureProposal#how-to-setup).

<br/>

## 2. Usage

 (2021/02/25 last modified)

### 2.1 Get Sample Scenario

Tested scenarios can be found at [https://github.com/tier4/planning_sim_launcher.iv.universe/tree/master/scenario](https://github.com/tier4/planning_sim_launcher.iv.universe/tree/master/scenario).
The examples placed in the repository tier4/ScenarioFormat are "examples of desired descriptions" and do not imply that this repository has already implemented them.

### 2.2 Procedure

#### (1) Make `scenario_database.json`

``` shell
cd /path/to/AutowareArchitectureProposal/src/simulator/planning_simulator_launcher
cp scenario_database_template.json scenario_database.json
```

#### (2) Edit `scenario_database.json`

Example:

```
{
  "log_output_base_dir": "$USER_HOME/AutowareArchitectureProposal/src/simulator/planning_simulator_launcher/scenario/log/",
  "map": {
    "odaiba": "$USER_HOME/AutonomousDrivingScenarios/map/odaiba"
  },
  "scenario": [
    "$USER_HOME/AutonomousDrivingScenarios/odd2/scenario/UC-001-0002/PSim_1/UC-001-0002_PSim_1_case1.yaml"
  ],
}
```

You can use the name of the map registered in the above json file as the value of the Map tag in the scenario file.
Alternatively, it is also possible to specify it directly by an absolute path.
Note that the Map path must be rewritten when sharing a scenario with multiple users when the value of the Map tag is specified by an absolute path.

#### (3) Execute

``` shell
source /opt/ros/foxy/setup.bash
source /path/to/AutowareArchitectureProposal/install/setup.bash

ros2 run planning_simulator_launcher launch_main --database /path/to/AutowareArchitectureProposal/src/simulator/planning_simulator_launcher/scenario_database.json

```

list option 
#### --database _D_ ... use scenario_database.json in _D_
#### --vehicle_model _V_... Use vehicle Model _V_
#### --sensor_model _S_ ... Use sensor Model _S_
#### --record-rosbag ... Record rosbag


<br/>

## 3. Known Problems

### 3.1 The simulator is actually non-deterministic.

You get slightly different simulation results with each run.

<br/>

## 4. For Developers

 (2020/07/01 last modified)

This section contains information for developers.

### Action Supporting Status

| Private Action   |  Ego  | Vehicle | MotorBike | Bicycle | Pedestrian |
| :--------------- | :---: | :-----: | :-------: | :-----: | :--------: |
| Acceleration     |   -   |    ✓    |     ✓     |    ✓    |     ✓      |
| Enable           |   -   |    ✓    |     ✓     |    ✓    |     ✓      |
| FollowRoute      |   ✓   |    ✓    |     ✓     |    ✓    |     ✓      |
| FollowTrajectory |   -   |    -    |     ✗     |    ✗    |     ✗      |
| LaneChange       |   -   |    ✓    |     ✓     |    ✓    |     -      |
| Pose             |   -   |    ✗    |     ✗     |    ✗    |     ✗      |
| RelativeSpeed    |   -   |    ✗    |     ✗     |    ✗    |     ✗      |
| Speed            |  (1)  |    ✓    |     ✓     |    ✓    |     ✓      |

| Global Action | Supporting |
| :------------ | :--------: |
| ChangeSignal  |     ✓      |

(1) Because Autoware calculates speed automatically, the scenario runner implicitly treats Speed Action for the Type: Ego entity as the maximum speed setting.

### Condition Supporting Status

| Logical Operator | Supporting |
| :--------------- | :--------: |
| All              |     ✓      |
| Any              |     ✓      |

| Private Condition |  Ego  | Vehicle | MotorBike | Bicycle | Pedestrian |
| :---------------- | :---: | :-----: | :-------: | :-----: | :--------: |
| Acceleration      |   ✓   |    ✓    |     ✓     |    ✓    |     ✓      |
| CollisionByEntity |   ✓   |    ✓    |     ✓     |    ✓    |     ✓      |
| CollisionByType   |   ✗   |    ✗    |     ✗     |    ✗    |     ✗      |
| EndOfRoad         |   ✗   |    ✗    |     ✗     |    ✗    |     ✗      |
| Offroad           |   ✗   |    ✗    |     ✗     |    ✗    |     ✗      |
| ReachPosition     |   ✓   |    ✓    |     ✓     |    ✓    |     ✓      |
| RelativeDistance  |   ✓   |    ✓    |     ✓     |    ✓    |     ✓      |
| Speed             |   ✓   |    ✓    |     ✓     |    ✓    |     ✓      |

| Global Condition | Supporting |
| :--------------- | :--------: |
| Signal           |     ✓      |
| SimulationTime   |     ✓      |

## 5. Log file format

The log file was outputted into the directory which you targeted in the scenario_database.json.
Log file named as (scenario_id).json and (scenario_id).bag.
(scenario_id).json contains information like below.
move_distance is a move distance of the ego vehicle in the simulation.

```
    "metadata": {
        "scenario_id": "b56add4b-a5fb-46d7-bcc3-864a0590a17b",
        "start_datetime": "2020-05-27T05:17:20.276897",
        "end_datetime": "2020-05-27T05:18:29.850055",
        "duration": "69.573157929000004",
        "autoware_commit_hash": "",
        "simulator_commit_hash": "",
        "move_distance": "151.54248"
    },
```

#!/usr/bin/env python

import argparse
import glob
import json
import termcolor

class Viewer:
    def __init__(self,scenario_generated_path,log_path,verbose,quiet,output):
        self.quiet = quiet
        self.verbose = verbose
        self.log_path = log_path
        generate_log_f = open(scenario_generated_path+'/generate_log.json', 'r')
        self.generate_log = json.load(generate_log_f)
        generate_log_f.close()
        self.raw_data = {}
        self.output_data = []
        json_log_list = glob.glob(self.log_path+'/*.json')
        for json_path in json_log_list:
            json_open = open(json_path, 'r')
            json_load = json.load(json_open)
            scenario_id = json_load["metadata"]["scenario_id"]
            self.raw_data[scenario_id] = json_load
            if self.getGenerateLog(scenario_id) is not None:
                self.output_data.append(self.getResultDict(scenario_id))
        if self.quiet == False:
            self.outputResult(output)
            self.printResult()
    def printResult(self):
        if self.verbose:
            self.printVerboseResult()
        else:
            self.printSimpleResult()
        self.printSummary()
    def printVerboseResult(self):
        i = 1
        for data in self.output_data:
            message = "[" + str(i) + "/" + str(len(self.output_data)) + "] "
            if data["passed"]:
                message = message +  "OK \n"
            else:
                message = message + "NG \n"
            message = message + "id = " + data["id"] + "\n"
            message = message + "parent = " + data["parent"] + "\n"
            message = message + "rosbg = " + data["rosbag"] + "\n"
            message = message + "json = " + data["json_log"] + "\n"
            metadata = data["metadata"]
            message = message + "start_datetime = " + metadata["start_datetime"] + "\n"
            message = message + "end_datetime = " + metadata["end_datetime"] + "\n"
            message = message + "duration = " + metadata["duration"] + "\n"
            message = message + "params\n"
            if len(data["params"]) == 0:
                message = message + "    no scenario parameters"
            param_index = 1
            for param in data["params"]:
                message = message + "    (" + str(param_index) + "/" + str(len(data["params"])) + ") " + param + " : " + str(data["params"][param])
                param_index = param_index + 1
            message = message + "\n"
            if data["passed"]:
                message = termcolor.colored(message, 'green')
            else:
                message = termcolor.colored(message, 'red')
            print(message)
            i = i + 1
    def printSimpleResult(self):
        i = 1
        for data in self.output_data:
            message = "[" + str(i) + "/" + str(len(self.output_data)) + "] "
            if data["passed"]:
                message = message +  "OK "
            else:
                message = message + "NG "
            message = message + "id = " + data["id"]
            if data["passed"]:
                message = termcolor.colored(message, 'green')
            else:
                message = termcolor.colored(message, 'red')
            print(message)
            i = i + 1
    def printSummary(self):
        print("[summary]")
        i = 0
        for data in self.output_data:
            if data["passed"]:
                i = i + 1
        print("passed " + str(i) + "/" + str(len(self.output_data)) + " test cases.")
    def outputResult(self,output):
        if output != None:
            with open(output, 'w') as f:
                json.dump(self.output_data, f, indent=4)
    def getGenerateLog(self,id):
        for log in self.generate_log:
            if log["id"] == id:
                return log
        return None
    def getResultDict(self,id):
        ret = {}
        if self.getResult(id):
            ret["passed"] = True
        else:
            ret["passed"] = False
        gen_log = self.getGenerateLog(id)
        ret["parent"] = gen_log["parent_path"]
        ret["id"] = id
        ret["json_log"] = self.log_path +  id + ".json"
        ret["rosbag"] = self.log_path + id + ".bag"
        metadata = self.raw_data[id]["metadata"]
        ret["metadata"] = metadata
        if "param" in gen_log:
            param = {}
            param[gen_log["param"]["name"]] = gen_log["param"]["value"]
            ret["params"] = param
        else:
            ret["params"] = {}
        return ret
    def getResult(self,id):
        logs = self.raw_data[id]["log"]
        scenario_succeded = False
        for log in logs:
            categories = log["categories"]
            if "endcondition" in categories:
                if log["description"] == "simulation succeeded":
                    scenario_succeded = True
                else:
                    scenario_succeded = False
        return scenario_succeded


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Show result')
    parser.add_argument('scenario_generated_path', type=str, help='path of the generated scenario')
    parser.add_argument('log_path', type=str,help='path of the log')
    parser.add_argument('--verbose', help='show detail or not', action="store_true")
    parser.add_argument('--quiet', help='does not show output', action="store_true")
    parser.add_argument('--output',type=str,help='output path of the result(if not set, only output result to the screen)')
    args = parser.parse_args()
    viwer = Viewer(args.scenario_generated_path,args.log_path,args.verbose,args.quiet,args.output)

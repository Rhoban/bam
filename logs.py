import glob
import copy
import random
import json


class Logs:
    def __init__(self, directory: str):
        # Directories
        self.directory: str = directory
        self.json_files = glob.glob(f"{self.directory}/*.json")

        self.logs = []
        for json_file in self.json_files:
            with open(json_file) as f:
                data = json.load(f)
                data["filename"] = json_file
                if "arm_mass" not in data:
                    data["arm_mass"] = 0.0
                self.logs.append(data)

    def split(self, ratio: float) -> "Logs":
        """
        Extract ratio% of the logs from the current object to 
        """        
        to_extract = int(len(self.logs) * ratio)
        indices = random.sample(range(len(self.logs)), to_extract)
        other_logs = copy.deepcopy(self)

        self.json_files = [
            self.json_files[i] for i in range(len(self.json_files)) if i not in indices
        ]
        self.logs = [self.logs[i] for i in range(len(self.logs)) if i not in indices]

        other_logs.json_files = [
            other_logs.json_files[i]
            for i in range(len(other_logs.json_files))
            if i in indices
        ]
        other_logs.logs = [
            other_logs.logs[i] for i in range(len(other_logs.logs)) if i in indices
        ]

        return other_logs

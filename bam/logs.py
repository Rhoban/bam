import glob
import numpy as np
import copy
import random
import json
from . import message


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

    def split(self, selector_kp: int) -> "Logs":
        """
        Extract ratio% of the logs from the current object to
        """
        indices = []
        for k, log in enumerate(self.logs):
            if log["kp"] == selector_kp:
                indices.append(k)

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

    def make_batch(self) -> dict:
        """
        Make a batch log from all the logs. In a batch log, all entries are vectorized.
        For example, batch["mass"] is a vector of all masses
        batch["entries"][0]["position"] will be a vector of all positions
        """
        batch: dict = {"entries": []}

        for key in self.logs[0]:
            if key != "entries":
                batch[key] = np.array([log[key] for log in self.logs])

        entries_min_length = min([len(log["entries"]) for log in self.logs])
        entries_max_length = max([len(log["entries"]) for log in self.logs])
        if entries_max_length > entries_min_length + 1:
            print(
                message.yellow(
                    f"WARNING: logs have significantly different lengths ({entries_min_length} to {entries_max_length})"
                )
            )

        entry_keys = self.logs[0]["entries"][0].keys()
        for k in range(entries_min_length):
            batch["entries"].append(
                {
                    key: np.array([log["entries"][k][key] for log in self.logs])
                    for key in entry_keys
                }
            )

        return batch

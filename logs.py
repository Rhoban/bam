import glob
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
                self.logs.append(data)

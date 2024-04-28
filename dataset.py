import numpy as np
from torch.utils.data import Dataset as TorchDataset
import json

class FrictionDataset(TorchDataset):
    """
    Dataset class compatible with PyTorch DataLoader. 

    Inputs are 1D numpy ndarray of size 4 * window_size containing for the n-th entry:
        - volts_[n-w_s+1,n]
        - dtheta_[n-w_s+1,n]
        - torque_[n-w_s+1,n]
        - tau_l_[n-w_s+1,n]

    Outputs are 1D numpy ndarray of size 2 containing for the n-th entry:
        - ddtheta_n
        - I_l
    """
    def __init__(self, window_size: int):
        self.inputs = []
        self.outputs = []
        self.window_size = window_size
        self.size = 0
    
    def __len__(self):
        return self.size
    
    def __getitem__(self, index: int):
        return {"input": self.inputs[index], "output": self.outputs[index]}
    
    def __getitems__(self, indices: list):
        return [{"input": self.inputs[index], "output": self.outputs[index]} for index in indices]
    
    def add_entry(self, input: np.ndarray, output: float) -> None:
        """
        Add an entry to the dataset.
        """
        self.inputs.append(np.float32(input))
        self.outputs.append(np.float32(output))
        self.size += 1

    def add_log(self, processed_log, offset=5): 
        with open(processed_log, 'r') as file:
            data = json.load(file)
        
        velocity = [entry["velocity"] for entry in data["entries"]]
        acceleration = [entry["acceleration"] for entry in data["entries"]]
        volts = [entry["volts"] for entry in data["entries"]]
        torque_enable = [entry["torque_enable"] for entry in data["entries"]]
        tau_l = [entry["tau_l"] for entry in data["entries"]]

        mass = data["mass"]
        length = data["length"]

        for i in range(self.window_size + offset, len(volts) - offset):
            input = np.array(volts[i - self.window_size:i] +
                             velocity[i - self.window_size:i] +
                             torque_enable[i - self.window_size:i] +
                             tau_l[i - self.window_size:i])
            
            output = np.array([acceleration[i-1]] +
                              [mass * length**2])
            
            self.add_entry(input, output)

    def shuffle(self) -> None:
        """
        Shuffle the dataset entries.
        """
        indices = np.arange(self.size)
        np.random.shuffle(indices)
        self.inputs = [self.inputs[index] for index in indices]
        self.outputs = [self.outputs[index] for index in indices]

    def split(self, ratio: float) -> ("Dataset", "Dataset"):
        """
        Split the dataset into two datasets of sizes ratio and 1 - ratio.
        Do not shuffle the dataset before splitting.
        """
        if ratio < 0 or ratio > 1:
            raise ValueError(f"Ratio should be between 0 and 1 but is {ratio}")
        
        dataset_1 = FrictionDataset(self.window_size)
        dataset_2 = FrictionDataset(self.window_size)

        index = int(ratio * self.size)

        dataset_1.inputs = self.inputs[:index]
        dataset_1.outputs = self.outputs[:index]
        dataset_1.size = index

        dataset_2.inputs = self.inputs[index:]
        dataset_2.outputs = self.outputs[index:]
        dataset_2.size = self.size - index

        return dataset_1, dataset_2

    def save(self, filename: str) -> None:
        """
        Save the dataset in a compressed numpy file.
        """
        np.savez_compressed(filename, 
                            window_size=self.window_size,
                            inputs=self.inputs, 
                            outputs=self.outputs)

    @classmethod
    def load(cls, filename: str) -> "FrictionDataset":
        """
        Load the dataset from a compressed numpy file.
        """
        data = np.load(filename)

        dataset = cls(data["window_size"])
        dataset.inputs = np.float32(data["inputs"])
        dataset.outputs = np.float32(data["outputs"])
        dataset.size = len(dataset.inputs)
        
        return dataset
    

class FrictionMaxDataset(TorchDataset):
    """
    Dataset class compatible with PyTorch DataLoader. 

    Inputs are 1D numpy ndarray of size 4 * window_size containing for the n-th entry:
        - volts_[n-w_s+1,n]
        - dtheta_[n-w_s+1,n]
        - torque_[n-w_s+1,n]
        - tau_l_[n-w_s+1,n]

    Outputs are 1D numpy ndarray of size 2 containing for the n-th entry:
        - ddtheta_n
        - I_l
    """
    def __init__(self, window_size: int):
        self.inputs = []
        self.outputs = []
        self.window_size = window_size
        self.size = 0
    
    def __len__(self):
        return self.size
    
    def __getitem__(self, index: int):
        return {"input": self.inputs[index], "output": self.outputs[index]}
    
    def __getitems__(self, indices: list):
        return [{"input": self.inputs[index], "output": self.outputs[index]} for index in indices]
    
    def add_entry(self, input: np.ndarray, output: float) -> None:
        """
        Add an entry to the dataset.
        """
        self.inputs.append(np.float32(input))
        self.outputs.append(np.float32(output))
        self.size += 1

    def add_log(self, processed_log, offset=5): 
        with open(processed_log, 'r') as file:
            data = json.load(file)
        
        velocity = [entry["velocity"] for entry in data["entries"]]
        acceleration = [entry["acceleration"] for entry in data["entries"]]
        volts = [entry["volts"] for entry in data["entries"]]
        torque_enable = [entry["torque_enable"] for entry in data["entries"]]
        tau_l = [entry["tau_l"] for entry in data["entries"]]

        mass = data["mass"]
        length = data["length"]

        for i in range(self.window_size + offset, len(volts) - offset):
            input = np.array(volts[i - self.window_size:i] +
                             velocity[i - self.window_size:i] +
                             torque_enable[i - self.window_size:i] +
                             tau_l[i - self.window_size:i])
            
            output = np.array([acceleration[i-1]] +
                              [mass * length**2])
            
            self.add_entry(input, output)



if __name__ == "__main__":
    import argparse
    import os

    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-l", "--logdir", type=str, required=False, default=None, help="Processed log directory used to build a dataset")
    arg_parser.add_argument("-d", "--datasetdir", type=str, required=False, default=None, help="Dataset directory")
    arg_parser.add_argument("-w", "--window_size", type=int, required=False, default=10, help="Window size for the dataset")
    args = arg_parser.parse_args()

    # Processing data
    if args.logdir != None and args.datasetdir != None:
        print(f"Processing log files from {args.logdir} ...")
        
        train_dataset = FrictionDataset(args.window_size)
        test_dataset = FrictionDataset(args.window_size)
        for log in os.listdir(args.logdir):
            print(f"Loading {log} ...")
            with open(args.logdir + "/" + log, 'r') as file:
                data = json.load(file)

            # Selecting logs based on Kp to have drastically different training and testing datasets
            if data["kp"] == 16: 
                test_dataset.add_log(args.logdir + "/" + log)
            else:   
                train_dataset.add_log(args.logdir + "/" + log)

        print("Window size : ", args.window_size)
        print(f"Training dataset size : {len(train_dataset)}")
        print(f"Testing dataset size : {len(test_dataset)}")

        print(f"Shuffling datasets ...")
        train_dataset.shuffle()
        test_dataset.shuffle()

        train_dataset.save(args.datasetdir + f"/train_dataset_w{args.window_size}.npz")
        test_dataset.save(args.datasetdir + f"/test_dataset_w{args.window_size}.npz")
        print(f"Datasets saved in {args.datasetdir}! ")

    # Tests
    else:
        print("Testing FrictionDataset class :")
        dataset = FrictionDataset(3)

        for log in os.listdir("../data_106_network"):
            with open("../data_106_network/" + log, 'r') as file:
                data = json.load(file)

            dataset.add_log("../data_106_network/" + log, offset=0)
            print("Loading a log file ...")
            break

        print("First value (window_size = 3): ", dataset[0])
        volts = [entry["volts"] for entry in data["entries"]]
        acceleration = [entry["acceleration"] for entry in data["entries"]]
        print("Volts : ", volts[:3])
        print("Acceleration : ", acceleration[:3])

        print("Dataset size : ", len(dataset))

        ratio = 0.8
        train_dataset, test_dataset = dataset.split(ratio)
        print(f"Splitting with ratio {ratio} : ", len(train_dataset), len(test_dataset))

        print("Saving/Loading :")
        print("First value before saving : ", dataset[0])
        dataset.save("datasets/106/test.npz")
        loaded_dataset = FrictionDataset.load("datasets/106/test.npz")
        print("First value after loading : ", loaded_dataset[0])

        os.remove("datasets/106/test.npz")

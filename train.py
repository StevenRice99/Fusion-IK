import argparse
import os

import pandas as pd
import torch
from torch import nn, optim
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm


class InverseKinematicsDataset(Dataset):
    """
    The datasets built from the CSV data.
    """

    def __init__(self, df):
        """
        Create the dataset.
        """
        # Loop through all inputs.
        self.inputs = []
        i = 0
        while f"I{i + 1}" in df.columns:
            self.inputs.append(f"I{i + 1}")
            i += 1
        # Loop through all outputs.
        self.outputs = []
        i = 0
        while f"O{i + 1}" in df.columns:
            self.outputs.append(f"O{i + 1}")
            i += 1
        # Convert for PyTorch.
        self.inputs = torch.tensor(df[self.inputs].to_numpy(), dtype=torch.float32)
        self.outputs = torch.tensor(df[self.outputs].to_numpy(), dtype=torch.float32)

    def __len__(self):
        """
        Get the length of the dataset.
        :return: The length of the dataset.
        """
        return len(self.inputs)

    def __getitem__(self, index):
        """
        Get a data pair.
        :param index: The index to get.
        :return: The inputs and outputs.
        """
        return self.inputs[index], self.outputs[index]


class JointNetwork(nn.Module):
    """
    The neural network to train.
    """

    def __init__(self, joints: int, minimal: bool, large: bool):
        """
        Create the neural network.
        :param joints: The number of joints.
        :param minimal: If this is a minimal network.
        :param minimal: If a large structure of hidden layers should be used.
        """
        super().__init__()
        # Define the network.
        inputs = 6 if minimal else joints + 6
        if large:
            hidden = 128
            self.neurons = nn.Sequential(
                nn.Linear(inputs, hidden),
                nn.ReLU(),
                nn.Dropout(),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.Dropout(),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.Dropout(),
                nn.Linear(hidden, joints),
                nn.ReLU(),
            )
        else:
            self.neurons = nn.Linear(inputs, joints)
        self.loss = nn.MSELoss()
        self.optimizer = optim.Adam(self.parameters())
        # Run on GPU if available.
        self.to(get_processing_device())

    def forward(self, inputs):
        """
        Feed forward inputs into the neural network.
        :param inputs: The inputs for the network.
        :return: The final output layer from the network.
        """
        return self.neurons(inputs)

    def predict(self, inputs):
        """
        Get the network's prediction for joint values.
        :param inputs: The inputs for the network.
        :return: The final output layer from the network.
        """
        with torch.no_grad():
            return self.forward(inputs)

    def optimize(self, inputs, outputs):
        """
        Optimize the neural network.
        :param inputs: The inputs for the network.
        :param outputs: The outputs for the network.
        :return: The network's loss on this prediction.
        """
        self.optimizer.zero_grad()
        loss = self.loss(self.forward(inputs), outputs)
        loss.backward()
        self.optimizer.step()
        return loss.item()

    def calculate_score(self, inputs, outputs):
        """
        Check the score of the model.
        :param inputs: The inputs for the network.
        :param outputs: The outputs for the network.
        :return: The network's score.
        """
        predicted = self.predict(inputs).tolist()
        outputs = outputs.tolist()
        accuracies = [0] * len(outputs[0])
        total = 0
        # Loop through every element in the batch.
        for i in range(len(outputs)):
            joints_total = 0
            # Loop through every output in that batch element, which in this case will just be one anyway.
            for j in range(len(outputs[i])):
                # Get the absolute difference.
                joint = max(outputs[i][j], predicted[i][j]) - min(outputs[i][j], predicted[i][j])
                accuracies[j] += joint
                joints_total += joint
            # Get the average of this element, which in this case again is just of size one to begin with anyway.
            joints_total /= len(outputs[i])
            total += joints_total
        # Average out over the batch size.
        for i in range(len(accuracies)):
            accuracies[i] /= len(outputs)
        return total / len(outputs), accuracies


def get_processing_device():
    """
    Get the device to use for training, so we can use the GPU if CUDA is available.
    :return: The device to use for training being a CUDA GPU if available, otherwise the CPU.
    """
    return torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")


def to_tensor(tensor, device=get_processing_device()):
    """
    Convert an image to a tensor to run on the given device.
    :param tensor: The data to convert to a tensor.
    :param device: The device to use for training being a CUDA GPU if available, otherwise the CPU.
    :return: The data ready to be used.
    """
    return tensor.to(device)


def test(net, dataset):
    """
    Test a neural network.
    :param net: The network.
    :param dataset: The dataset to test.
    :return: The model's accuracy.
    """
    net.eval()
    accuracy = 0
    accuracies = None
    for inputs, outputs in dataset:
        batch_accuracy, batch_accuracies = net.calculate_score(to_tensor(inputs), to_tensor(outputs))
        accuracy += batch_accuracy
        if accuracies is None:
            accuracies = batch_accuracies
        else:
            for i in range(len(accuracies)):
                accuracies[i] += batch_accuracies[i]
    # Convert into human-readable accuracy.
    for i in range(len(accuracies)):
        accuracies[i] = (1 - accuracies[i] / len(dataset)) * 100
    return (1 - accuracy / len(dataset)) * 100, accuracies


def train(epochs: int):
    """
    Train robot networks.
    :param epochs: Number of epochs to train for.
    :return: Nothing.
    """
    # Ensure values are valid.
    if epochs < 1:
        epochs = 1
    print(f"Fusion IK training running on GPU with CUDA {torch.version.cuda}." if torch.cuda.is_available() else "Fusion IK training running on CPU.")
    # Check if there is data to train on.
    if not os.path.exists(os.path.join(os.getcwd(), "Training")):
        return
    robots = os.listdir(os.path.join(os.getcwd(), "Training"))
    for robot in robots:
        if not os.path.isdir(os.path.join(os.getcwd(), "Training", robot)):
            continue
        results = {}
        joints = 0
        for minimal in [False, True]:
            mode = "Minimal.csv" if minimal else "Standard.csv"
            if not os.path.exists(os.path.join(os.getcwd(), "Training", robot, mode)):
                continue
            df = pd.read_csv(os.path.join(os.getcwd(), "Training", robot, mode))
            mode = mode.replace(".csv", "")
            # If there are no joints meaning the data is invalid, exit.
            joints = 0
            while f"O{joints + 1}" in df.columns:
                joints += 1
            if joints == 0:
                print(f"{robot} | {mode} | No joint values.")
                continue
            # Ensure folder to save models exists.
            if not os.path.exists(os.path.join(os.getcwd(), "Networks")):
                os.mkdir(os.path.join(os.getcwd(), "Networks"))
            if not os.path.exists(os.path.join(os.getcwd(), "Networks", robot)):
                os.mkdir(os.path.join(os.getcwd(), "Networks", robot))
            # Setup datasets.
            total = len(df)
            training_size = int(total * 0.8)
            testing_size = total - training_size
            if testing_size == 0:
                training_size -= 1
                testing_size = 1
            training = DataLoader(InverseKinematicsDataset(df.head(training_size)), batch_size=1, shuffle=True)
            testing = DataLoader(InverseKinematicsDataset(df.tail(testing_size)), batch_size=testing_size, shuffle=False)
            for large in [False, True]:
                # Define the network.
                net = JointNetwork(joints, minimal, large)
                name = robot + (" Minimal" if minimal else " Standard") + (" Large" if large else "")
                # Check if an existing net exists for this joint, load it.
                if os.path.exists(os.path.join(os.getcwd(), "Networks", robot, f"{name}.pt")):
                    try:
                        saved = torch.load(os.path.join(os.getcwd(), "Networks", robot, f"{name}.pt"))
                        epoch = saved["Epoch"]
                        accuracy = saved["Accuracy"]
                        accuracies = saved["Accuracies"]
                        best = saved["Best"]
                        net.load_state_dict(saved["Training"])
                        net.optimizer.load_state_dict(saved["Optimizer"])
                    except:
                        print(f"{name} | Unable to load existing data.")
                        continue
                # Otherwise, start a new training.
                else:
                    epoch = 0
                    best = net.state_dict()
                    accuracy, accuracies = test(net, testing)
                # Train for set epochs.
                parameters = sum(p.numel() for p in net.parameters() if p.requires_grad)
                core = f"{name} | {parameters} Parameters | {training_size} Training | {testing_size} Testing | No improvement for "
                while True:
                    # Save the data.
                    torch.save({
                        "Best": best,
                        "Training": net.state_dict(),
                        "Optimizer": net.optimizer.state_dict(),
                        "Epoch": epoch,
                        "Accuracy": accuracy,
                        "Accuracies": accuracies
                    }, os.path.join(os.getcwd(), "Networks", robot, f"{name}.pt"))
                    # Store the current training state.
                    current = net.state_dict()
                    # Export the best state.
                    net.load_state_dict(best)
                    torch.onnx.export(
                        net,
                        to_tensor(torch.randn(1, 6 if minimal else joints + 6, dtype=torch.float32)),
                        os.path.join(os.getcwd(), "Networks", robot, f"{name}.onnx"),
                        export_params=True,
                        opset_version=9,
                        do_constant_folding=True,
                        input_names=["input"],
                        output_names=["output"]
                    )
                    # Restore the current training state.
                    net.load_state_dict(current)
                    # Exit once done.
                    msg = f"{core}{epoch} Epochs | Accuracy {accuracy}%"
                    if epoch >= epochs:
                        msg += f" | Joints {accuracies[0]}%"
                        for i in range(1, len(accuracies)):
                            msg += f", {accuracies[i]}%"
                        results[mode] = {"Average": accuracy, "Joints": accuracies, "Parameters": parameters}
                        print(msg)
                        break
                    # Train on the training dataset.
                    net.train()
                    for inputs, outputs in tqdm(training, msg):
                        net.optimize(to_tensor(inputs), to_tensor(outputs))
                    # Check how well the newest epoch performs.
                    temp_accuracy, temp_accuracies = test(net, testing)
                    # Check if this is the new best network.
                    if temp_accuracy > accuracy:
                        best = net.state_dict()
                        accuracy = temp_accuracy
                        accuracies = temp_accuracies
                        epoch = 0
                    else:
                        epoch += 1
        if not os.path.exists(os.path.join(os.getcwd(), "Results")):
            os.mkdir(os.path.join(os.getcwd(), "Results"))
        if not os.path.exists(os.path.join(os.getcwd(), "Results", robot)):
            os.mkdir(os.path.join(os.getcwd(), "Results", robot))
        s = "Network"
        for mode in results:
            s += f",{mode}"
        s += "\nParameters"
        for mode in results:
            s += f",{results[mode]['Parameters']}"
        for i in range(joints):
            s += f"\nJoint {i + 1} Accuracy"
            for mode in results:
                s += f",{results[mode]['Joints'][i]}%"
        s += f"\nAverage Accuracy"
        for mode in results:
            s += f",{results[mode]['Average']}%"
        f = open(os.path.join(os.getcwd(), "Results", robot, "Training.csv"), "w")
        f.write(s)
        f.close()


if __name__ == '__main__':
    try:
        desc = "Fusion IK Training"
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=desc)
        parser.add_argument("epoch", nargs='?', type=int, help="Number of epochs to stop training after no improvement.", default=10)
        a = vars(parser.parse_args())
        train(a["epoch"])
    except KeyboardInterrupt:
        print("Training stopped.")
    except torch.cuda.OutOfMemoryError:
        print("CUDA out of memory. Try running with a smaller batch size.")
    except ValueError as error:
        print(error)

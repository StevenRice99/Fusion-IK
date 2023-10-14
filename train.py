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

    def __init__(self, joints: int, minimal: bool):
        """
        Create the neural network.
        :param joints: The number of joints.
        :param minimal: If this is a minimal network.
        """
        super().__init__()
        # Define the network.
        self.neurons = nn.Linear(6 if minimal else joints + 6, joints)
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
        total_acc = 0
        # Loop through every element in the batch.
        for i in range(len(outputs)):
            acc = 0
            # Loop through every output in that batch element, which in this case will just be one anyway.
            for j in range(len(outputs[i])):
                # Get the absolute difference.
                acc += max(outputs[i][j], predicted[i][j]) - min(outputs[i][j], predicted[i][j])
            # Get the average of this element, which in this case again is just of size one to begin with anyway.
            acc /= len(outputs[i])
            total_acc += acc
        # Average out over the batch size.
        return total_acc / len(outputs)


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


def test(net, dataloader):
    """
    Test a neural network.
    :param net: The network.
    :param dataloader: The dataloader to test.
    :return: The model's accuracy.
    """
    net.eval()
    accuracy = 0
    for inputs, outputs in dataloader:
        accuracy += net.calculate_score(to_tensor(inputs), to_tensor(outputs))
    # Convert into human-readable accuracy.
    return (1 - accuracy / len(dataloader)) * 100


def save(robot: str, minimal: bool, net, best, epoch: int, score: float, joints: int):
    """
    Save the model and ONNX export.
    :param robot: The robot the network is for.
    :param minimal: If this is a minimal network.
    :param net: The network.
    :param best: The best network model.
    :param epoch: The current epoch.
    :param score: The score.
    :param joints: The number of joints.
    :return: Nothing.
    """
    torch.save({
        'Best': best,
        'Training': net.state_dict(),
        'Optimizer': net.optimizer.state_dict(),
        'Epoch': epoch,
        'Score': score
    }, os.path.join(os.getcwd(), "Networks", robot, f"{robot} Minimal.pt" if minimal else f"{robot} Standard.pt"))
    # Store the current training state.
    old = net.state_dict()
    # Export the best state.
    net.load_state_dict(best)
    torch.onnx.export(
        net,
        to_tensor(torch.randn(1, 6 if minimal else joints + 6, dtype=torch.float32)),
        os.path.join(os.getcwd(), "Networks", robot, f"{robot} Minimal.onnx" if minimal else f"{robot} Standard.onnx"),
        export_params=True,
        opset_version=9,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output']
    )
    # Restore the current training state.
    net.load_state_dict(old)


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
            # Define the network.
            net = JointNetwork(joints, minimal)
            # Check if an existing net exists for this joint, load it.
            if os.path.exists(os.path.join(os.getcwd(), "Networks", robot, f"{robot} Minimal.pt" if minimal else f"{robot} Standard.pt")):
                try:
                    saved = torch.load(os.path.join(os.getcwd(), "Networks", robot, f"{robot} Minimal.pt" if minimal else f"{robot} Standard.pt"))
                    epoch = saved['Epoch']
                    score = saved['Score']
                    best = saved['Best']
                    net.load_state_dict(saved['Training'])
                    net.optimizer.load_state_dict(saved['Optimizer'])
                except:
                    print(f"{robot} | {mode} | Unable to load existing data.")
                    continue
            # Otherwise, start a new training.
            else:
                epoch = 0
                best = net.state_dict()
                score = test(net, testing)
                save(robot, minimal, net, best, epoch, score, joints)
            # Train for set epochs.
            core = f"{robot} | {mode} | {sum(p.numel() for p in net.parameters() if p.requires_grad)} Parameters | {training_size} Training | {testing_size} Testing | No improvement for "
            while True:
                # Exit once done.
                msg = f"{core}{epoch} Epochs | {score}%"
                if epoch >= epochs:
                    print(msg)
                    break
                # Train on the training dataset.
                net.train()
                for inputs, outputs in tqdm(training, msg):
                    net.optimize(to_tensor(inputs), to_tensor(outputs))
                # Check how well the newest epoch performs.
                temp = test(net, testing)
                # Check if this is the new best network.
                if temp > score:
                    best = net.state_dict()
                    score = temp
                    epoch = 0
                else:
                    epoch += 1
                # Save data.
                save(robot, minimal, net, best, epoch, score, joints)


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

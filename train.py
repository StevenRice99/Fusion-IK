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

    def __init__(self, df, joints: int):
        """
        Create the dataset.
        :param joints: The total number of joints.
        """
        # Loop through all inputs which is the number of joints plus three position and four rotation values.
        self.inputs = []
        for i in range(joints + 7):
            self.inputs.append(f"I{i + 1}")
        self.outputs = []
        # Loop through all outputs which is the number of joints.
        for i in range(joints):
            self.outputs.append(f"O{i + 1}")
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

    def __init__(self, joints: int, lr: float = 0.001, b1: float = 0.9, b2: float = 0.999, eps: float = 1e-08, decay: float = 0, amsgrad: bool = False):
        """
        Create the neural network.
        :param joints: The number of joints.
        """
        # Define the size of each joint network.
        hidden_layers = 2
        hidden_size = 128
        super().__init__()
        # Take in all joint, position, and rotation values.
        self.layers = nn.Sequential(
            nn.Linear(joints + 7, hidden_size),
            nn.ReLU()
        )
        # Apply batch normalization and dropout to the hidden layers.
        for i in range(hidden_layers):
            self.layers.append(nn.Linear(hidden_size, hidden_size))
            self.layers.append(nn.ReLU())
        # Outputs for joints.
        self.layers.append(nn.Linear(hidden_size, joints))
        self.layers.append(nn.ReLU())
        self.loss = nn.MSELoss()
        self.optimizer = optim.Adam(self.parameters(), lr, (b1, b2), eps, decay, amsgrad)
        # Run on GPU if available.
        self.to(get_processing_device())

    def forward(self, inputs):
        """
        Feed forward inputs into the neural network.
        :param inputs: The inputs for the network.
        :return: The final output layer from the network.
        """
        return self.layers(inputs)

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


def test(model, dataloader):
    """
    Test a neural network.
    :param model: The neural network.
    :param dataloader: The dataloader to test.
    :return: The model's accuracy.
    """
    model.eval()
    accuracy = 0
    for inputs, outputs in dataloader:
        accuracy += model.calculate_score(to_tensor(inputs), to_tensor(outputs))
    # Convert into human-readable accuracy.
    return (1 - accuracy / len(dataloader)) * 100


def save(robot: str, model, best, epoch: int, score: float, joints: int):
    """
    Save the model and ONNX export.
    :param robot: The robot the network is for.
    :param model: The network model.
    :param best: The best network model.
    :param epoch: The current epoch.
    :param score: The score.
    :param joints: The number of joints.
    :return: Nothing.
    """
    torch.save({
        'Best': best,
        'Training': model.state_dict(),
        'Optimizer': model.optimizer.state_dict(),
        'Epoch': epoch,
        'Score': score
    }, os.path.join(os.getcwd(), "Networks", f"{robot}.pt"))
    # Store the current training state.
    old = model.state_dict()
    # Export the best state.
    model.load_state_dict(best)
    torch.onnx.export(
        model,
        to_tensor(torch.randn(1, joints + 7, dtype=torch.float32)),
        os.path.join(os.getcwd(), "Networks", f"{robot}.onnx"),
        export_params=True,
        opset_version=9,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output']
    )
    # Restore the current training state.
    model.load_state_dict(old)


def train(epochs: int, batch: int):
    """
    Train robot joint networks.
    :param epochs: Number of epochs to train for.
    :param batch: Batch size.
    :return: Nothing.
    """
    # Ensure values are valid.
    if batch < 1:
        batch = 1
    if epochs < 1:
        epochs = 1
    print(f"Fusion-IK Training")
    print(f"Running on GPU with CUDA {torch.version.cuda}." if torch.cuda.is_available() else "Running on CPU.")
    # Check if there is data to train on.
    if not os.path.exists(os.path.join(os.getcwd(), "Training")):
        print("No data to train on.")
        return
    robots = os.listdir(os.path.join(os.getcwd(), "Training"))
    for robot in robots:
        if not os.path.isfile(os.path.join(os.getcwd(), "Training", robot)):
            continue
        df = pd.read_csv(os.path.join(os.getcwd(), "Training", robot))
        # We don't need the .csv anymore.
        robot = robot.replace(".csv", "")
        # If there are no joints meaning the data is invalid, exit.
        joints = 0
        while f"O{joints + 1}" in df.columns:
            joints += 1
        if joints == 0:
            print(f"{robot} | No joint values.")
            continue
        print(f"{robot} | {len(df)} records")
        # Ensure folder to save models exists.
        if not os.path.exists(os.path.join(os.getcwd(), "Networks")):
            os.mkdir(os.path.join(os.getcwd(), "Networks"))
        # Setup datasets.
        dataset = DataLoader(InverseKinematicsDataset(df, joints), batch_size=batch, shuffle=False)
        # Define the model.
        model = JointNetwork(joints)
        best = model.state_dict()
        best_score = 0
        # Check if an existing model exists for this joint, load it.
        if os.path.exists(os.path.join(os.getcwd(), "Networks", f"{robot}.pt")):
            try:
                saved = torch.load(os.path.join(os.getcwd(), "Networks", f"{robot}.pt"))
                epoch = saved['Epoch']
                best_score = saved['Score']
                # If already done training this joint, skip to the next.
                if epoch >= epochs:
                    print(f"{robot} | {best_score}%")
                    continue
                best = saved['Best']
                model.load_state_dict(saved['Training'])
                model.optimizer.load_state_dict(saved['Optimizer'])
                print(f"{robot} | Continuing training from epoch {epoch} with batch size {batch} for {epochs} epochs.")
            except:
                print(f"{robot} | Unable to load training data, exiting.")
                continue
        # Otherwise, start a new training.
        else:
            epoch = 1
            print(f"{robot} | Starting training with batch size {batch} for {epochs} epochs.")
        # If new training, write initial files.
        if epoch == 1:
            best_score = test(model, dataset)
            save(robot, model, best, epoch, best_score, joints)
        # Train for set epochs.
        while True:
            # Exit once done.
            if epoch > epochs:
                print(f"{robot} | {best_score}%")
                break
            msg = f"{robot} | Epoch {epoch}/{epochs} | {best_score:.4}%"
            # Train on the training dataset.
            model.train()
            for inputs, outputs in tqdm(dataset, msg):
                model.optimize(to_tensor(inputs), to_tensor(outputs))
            # Check how well the newest epoch performs.
            score = test(model, dataset)
            # Check if this is the new best model.
            if score > best_score:
                best = model.state_dict()
                best_score = score
            # Save data.
            epoch += 1
            save(robot, model, best, epoch, best_score, joints)


if __name__ == '__main__':
    try:
        desc = "Fusion-IK Training"
        parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=desc)
        parser.add_argument("-e", "--epoch", type=int, help="Number of epochs to train for.", default=100)
        parser.add_argument("-b", "--batch", type=int, help="Batch size.", default=64)
        a = vars(parser.parse_args())
        train(a["epoch"], a["batch"])
    except KeyboardInterrupt:
        print("Training stopped.")
    except torch.cuda.OutOfMemoryError:
        print("CUDA out of memory. Try running with a smaller batch size.")
    except ValueError as error:
        print(error)

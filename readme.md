# Fusion IK

- [Overview](#overview "Overview")
- [Setup](#setup "Setup")
- [Robot Modeling](#robot-modeling "Robot Modeling")
- [Training](#training "Training")
- [Testing](#testing "Testing")
- [Visualizing](#visualizing "Visualizing")
- [Bio IK](#bio-ik "Bio IK")

# Overview

Fusion IK: Solving Inverse Kinematics with Evolution and Deep Learning.

# Setup

1. Install [Unity](https://unity.com "Unity").
2. Clone or download and extract this repository.
3. Open the root of this repository with [Unity](https://unity.com "Unity").
4. Install [Python](https://www.python.org "Python") and required Python packages. If unfamiliar with [Python](https://www.python.org "Python"), "pip" comes with standard Python installations, and you can run "pip *package*" to install a package. The [Python](https://www.python.org "Python") scripts for this project is located in the "IK-Trainer" directory. It is recommended you create a [virtual environment](https://docs.python.org/3/library/venv.html "Python Virtual Environment") for your packages.
   1. [PyTorch](https://pytorch.org "PyTorch").
      1. It is recommended you visit [PyTorch's get started page](https://pytorch.org/get-started/locally "PyTorch Get Started") which will allow you to select to install CUDA support if you have an Nvidia GPU. This will give you a command you can copy and run to install [PyTorch](https://pytorch.org "PyTorch"), TorchVision, and TorchAudio, but feel free to remove TorchVision and TorchAudio from the command as they are not needed. Check which version of CUDA your Nvidia GPU supports. You can check your CUDA version by running "nvidia-smi" from the command line.
      2. When running the training script, a message will be output to the console if it is using CUDA.
   2. The remaining packages can be installed via "pip *package*" in the console:
      1. [pandas](https://pandas.pydata.org "pandas")
      2. [tqdm](https://github.com/tqdm/tqdm "tqdm")

# Robot Modeling

An ABB IRB 7600 robot has been prepared already. To create a new robot, attach a "Robot" component to the root of the chain. Create and setup a "Robot Properties" component and assign it to the "Robot" component. Joints are setup by attaching an "Articulation Body" and a "Robot Joint" to every joint. Every joint must have limits defined. Only single kinematic chains are supported, no robots with multiple arms. Although initial work to set them up has been done, and single-axis "Revolute" joints have been tested, "Prismatic" and "Spherical" joints were not used in testing. Save the robot as a prefab for use in later steps.

# Training

The scene "Generate" in the "ABB IRB 7600 > Scenes" folder. To do this with a new robot, a "Generator" component must exist in the scene and assign your previously created robot prefab. Set the maximum milliseconds the Bio IK attempts for generating will run for. Run the scene and training data will be generated. Visuals will be disabled in the scene as they are not needed. Once complete, run "train.py". Once complete, inside the "Networks" folder, copy the ONNX files for your robot's joints into your Unity project. Assign these ONNX models in order to the "Network Models" field of your "Robot Properties".

# Testing

The scene "Test" in the "ABB IRB 7600 > Scenes" folder is setup to do this. To do this with a new robot, a "Tester" component must exist in the scene and assign your previously created robot prefab. Run the scene and testing data will be generated. Visuals will be disabled in the scene as they are not needed. Once complete, run "evaluate.py". Once complete, you can view results in the "Results" folder.

# Visualizing

The scene "Visualize" in the "ABB IRB 7600 > Scenes" folder is setup to do this. To do this with a new robot, a "Visualizer" component must exist in the scene and assign your previously created robot prefab. Run the scene and you will have a GUI as well as keyboard and mouse controls to visualize the various inverse kinematics models.

# [Bio IK](https://d-nb.info/1221720910/34 "Bio IK: A Memetic Evolutionary Algorithm for Generic Multi-Objective Inverse Kinematics")

This repository utilizes a version of [Bio IK](https://d-nb.info/1221720910/34 "Bio IK: A Memetic Evolutionary Algorithm for Generic Multi-Objective Inverse Kinematics") designed specifically for single-chain robot arms, not being fully capable of other multi-objective solutions as described in the original [Bio IK paper](https://d-nb.info/1221720910/34 "Bio IK: A Memetic Evolutionary Algorithm for Generic Multi-Objective Inverse Kinematics"). This version of [Bio IK](https://d-nb.info/1221720910/34 "Bio IK: A Memetic Evolutionary Algorithm for Generic Multi-Objective Inverse Kinematics") was intended for use with this research only, and is not designed for general-purpose game development. For a full version of the original [Bio IK](https://d-nb.info/1221720910/34 "Bio IK: A Memetic Evolutionary Algorithm for Generic Multi-Objective Inverse Kinematics") with all features, see the official [Bio IK Unity asset](https://assetstore.unity.com/packages/tools/animation/bio-ik-67819 "Bio IK Unity Asset").
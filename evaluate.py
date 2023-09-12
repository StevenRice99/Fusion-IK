import argparse
import math
import os.path

import pandas as pd


def read_file(path: str):
    # Load the data.
    df = pd.read_csv(path)
    rows = df.shape[0]
    if rows == 0:
        return None
    success = 0
    time = 0
    fitness = 0
    # Sum the data.
    for index, data in df.iterrows():
        # Only add the time when the move was successful.
        if data["Success"] is True:
            success += 1
            time += float(data["Time"])
        # Only add the fitness when the move was unsuccessful.
        else:
            fitness += float(data["Fitness"])
    # Calculate the averages.
    time = math.inf if success == 0 else time / success
    fitness = 0 if success == rows else fitness / (rows - success)
    success = success / rows * 100
    return success, time, fitness


def evaluate():
    """
    Evaluate results.
    :return: Nothing.
    """
    print("Fusion-IK Evaluation")
    # Check that the data folder exists.
    testing = os.path.join(os.getcwd(), "Testing")
    if not os.path.exists(testing):
        return
    if not os.path.exists(os.path.join(os.getcwd(), "Results")):
        os.mkdir(os.path.join(os.getcwd(), "Results"))
    robots = os.listdir(testing)
    # Loop all robots.
    for robot in robots:
        if not os.path.exists(os.path.join(os.getcwd(), "Results", robot)):
            os.mkdir(os.path.join(os.getcwd(), "Results", robot))
        modes = os.listdir(os.path.join(testing, robot))
        results = {}
        # Loop all solving modes.
        for mode in modes:
            path = os.path.join(testing, robot, mode)
            # The network inference is in its own file.
            if os.path.isfile(path):
                if mode != "Network.csv":
                    continue
                success, time, fitness = read_file(path)
                f = open(os.path.join(os.getcwd(), "Results", robot, "Network.csv"), "w")
                if time == math.inf:
                    f.write(f"Success Rate (%),Fitness Score\n{success}%,{fitness}")
                else:
                    f.write(f"Success Rate (%),Move Time (s),Fitness Score\n{success}%,{time},{fitness}")
                f.close()
                continue
            # Loop through every timeout stored for Bio IK and Fusion IK.
            timeouts = os.listdir(path)
            for timeout in timeouts:
                file = os.path.join(path, timeout)
                if os.path.isdir(file):
                    continue
                timeout = int(timeout.replace(".csv", ""))
                if timeout not in results:
                    results[timeout] = {}
                success, time, fitness = read_file(file)
                if mode == "Bio IK":
                    results[timeout]["Bio IK"] = {"Success": success, "Time": time, "Fitness": fitness}
                elif mode == "Fusion IK":
                    results[timeout]["Fusion IK"] = {"Success": success, "Time": time, "Fitness": fitness}
        # Write the results to CSV.
        success = "Timeout (ms),Bio IK,Fusion IK"
        time = "Timeout (ms),Bio IK,Fusion IK"
        fitness = "Timeout (ms),Bio IK,Fusion IK"
        results = dict(sorted(results.items()))
        for timeout in results:
            success += f"\n{timeout},{results[timeout]['Bio IK']['Success']}%,{results[timeout]['Fusion IK']['Success']}%"
            time += f"\n{timeout},{results[timeout]['Bio IK']['Time']},{results[timeout]['Fusion IK']['Time']}"
            fitness += f"\n{timeout},{results[timeout]['Bio IK']['Fitness']},{results[timeout]['Fusion IK']['Fitness']}"
        f = open(os.path.join(os.getcwd(), "Results", robot, "Success Rate (%).csv"), "w")
        f.write(success)
        f.close()
        f = open(os.path.join(os.getcwd(), "Results", robot, "Move Time (s).csv"), "w")
        f.write(time)
        f.close()
        f = open(os.path.join(os.getcwd(), "Results", robot, "Fitness Score.csv"), "w")
        f.write(fitness)
        f.close()


if __name__ == '__main__':
    desc = "Fusion-IK Evaluation"
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=desc)
    evaluate()
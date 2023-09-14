import math
import os.path

import pandas as pd


def read_file(path: str):
    """
    Load data from a file.
    :param path: The CSV path.
    :return: The success rate (%), move time (s), and fitness score calculated from the file.
    """
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
                    print(f"Network | Success Rate (%) = {success}% | Fitness Score = {fitness}")
                    f.write(f"Success Rate (%),Fitness Score\n{success}%,{fitness}")
                else:
                    print(f"Network | Success Rate (%) = {success}% | Move Time (s) = {time} s | Fitness Score = {fitness}")
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
                results[timeout][mode] = {"Success": success, "Time": time, "Fitness": fitness}
        # Write the results to CSV.
        success = "Timeout (ms),Bio IK,Fusion IK,Improvement"
        time = "Timeout (ms),Bio IK,Fusion IK,Improvement (%)"
        fitness = "Timeout (ms),Bio IK,Fusion IK,Improvement"
        results = dict(sorted(results.items()))
        for timeout in results:
            b_success = results[timeout]["Bio IK"]["Success"]
            f_success = results[timeout]["Fusion IK"]["Success"]
            b_time = results[timeout]["Bio IK"]["Time"]
            f_time = results[timeout]["Fusion IK"]["Time"]
            b_fitness = results[timeout]["Bio IK"]["Fitness"]
            f_fitness = results[timeout]["Fusion IK"]["Fitness"]
            i_success = f_success - b_success
            i_time = (b_time - f_time) / f_time * 100
            i_fitness = (b_fitness - f_fitness) / f_fitness * 100
            print(f"Bio IK      | Timeout (ms) = {timeout} ms\t| Success Rate (%) = {b_success}%\t| Move Time (s) = {b_time} s\t| Fitness Score = {b_fitness}\n"
                  f"Fusion IK   | Timeout (ms) = {timeout} ms\t| Success Rate (%) = {f_success}%\t| Move Time (s) = {f_time} s\t| Fitness Score = {f_fitness}\n"
                  f"Improvement | Timeout (ms) = {timeout} ms\t| Success Rate (%) = {i_success}%\t| Move Time (%) = {i_time}%\t| Fitness Score (%) = {i_fitness}%")
            success += f"\n{timeout},{b_success}%,{f_success}%,{i_success}%"
            time += f"\n{timeout},{b_time},{f_time},{i_time}%"
            fitness += f"\n{timeout},{b_fitness},{f_fitness},{i_fitness}%"
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
    try:
        evaluate()
    except KeyboardInterrupt:
        print("Evaluation stopped.")
    except ValueError as error:
        print(error)

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
    successes = {}
    # Sum the data.
    for index, data in df.iterrows():
        # Only add the time when the move was successful.
        if data["Success"] is True:
            success += 1
            successes[index] = float(data["Time"])
            time += successes[index]
        # Only add the fitness when the move was unsuccessful.
        else:
            fitness += float(data["Fitness"])
    # Calculate the averages.
    time = "" if success == 0 else time / success
    fitness = "" if success == rows else fitness / (rows - success)
    success = success / rows * 100
    print(f"Processed {path}")
    return success, time, fitness, successes


def evaluate():
    """
    Evaluate results.
    :return: Nothing.
    """
    print("Fusion IK Evaluation")
    # Check that the data folder exists.
    testing = os.path.join(os.getcwd(), "Testing")
    if not os.path.exists(testing):
        return
    if not os.path.exists(os.path.join(os.getcwd(), "Results")):
        os.mkdir(os.path.join(os.getcwd(), "Results"))
    robots = os.listdir(testing)
    modes = []
    successes = {}
    # Loop all robots.
    for robot in robots:
        if not os.path.exists(os.path.join(os.getcwd(), "Results", robot)):
            os.mkdir(os.path.join(os.getcwd(), "Results", robot))
        data = os.listdir(os.path.join(testing, robot))
        results = {}
        networks = {}
        # Loop all solving modes.
        for d in data:
            path = os.path.join(testing, robot, d)
            # The network inference is in its own file.
            if os.path.isfile(path):
                success, time, fitness, _ = read_file(path)
                networks[d.replace(".csv", "")] = {"Success": success, "Time": time, "Fitness": fitness}
                continue
            # Loop through every timeout stored for Bio IK and Fusion IK.
            timeouts = os.listdir(path)
            if d not in modes:
                modes.append(d)
            for timeout in timeouts:
                file = os.path.join(path, timeout)
                if os.path.isdir(file):
                    continue
                timeout = int(timeout.replace(".csv", ""))
                if timeout not in results:
                    results[timeout] = {}
                success, time, fitness, temp = read_file(file)
                if d not in successes:
                    successes[d] = {}
                    for iteration in temp:
                        successes[d][iteration] = {"Timeout": timeout, "Time": temp[iteration]}
                else:
                    for iteration in temp:
                        if iteration not in successes[d] or timeout < successes[d][iteration]["Timeout"]:
                            successes[d][iteration] = {"Timeout": timeout, "Time": temp[iteration]}
                results[timeout][d] = {"Success": success, "Time": time, "Fitness": fitness}
        # Write network results to CSV.
        success = False
        for mode in networks:
            networks[mode]["Success"] = f"{networks[mode]['Success']}%"
            if networks[mode]["Time"] != "":
                success = True
        s = "Name,Success Rate (%),Move Time (s),Fitness Score" if success else "Name,Success Rate (%),Fitness Score"
        for mode in networks:
            if success:
                s += f"\n{mode},{networks[mode]['Success']},{networks[mode]['Time']},{networks[mode]['Fitness']}"
            else:
                s += f"\n{mode},{networks[mode]['Success']},{networks[mode]['Fitness']}"
        f = open(os.path.join(os.getcwd(), "Results", robot, "Networks.csv"), "w")
        f.write(s)
        f.close()
        # Write other results to CSV.
        s = "Timeout (ms)"
        for mode in modes:
            s += f",{mode}"
        for file in ["Success Rate (%).csv", "Move Time (s).csv", "Fitness Score.csv"]:
            f = open(os.path.join(os.getcwd(), "Results", robot, file), "w")
            f.write(s)
            f.close()
        results = dict(sorted(results.items()))
        for timeout in results:
            success = time = fitness = f"\n{timeout}"
            for mode in modes:
                if mode not in results[timeout]:
                    success += ","
                    time += ","
                    fitness += ","
                    continue
                success += f",{results[timeout][mode]['Success']}%"
                time += f",{results[timeout][mode]['Time']}"
                fitness += f",{results[timeout][mode]['Fitness']}"
            f = open(os.path.join(os.getcwd(), "Results", robot, "Success Rate (%).csv"), "a")
            f.write(success)
            f.close()
            f = open(os.path.join(os.getcwd(), "Results", robot, "Move Time (s).csv"), "a")
            f.write(time)
            f.close()
            f = open(os.path.join(os.getcwd(), "Results", robot, "Fitness Score.csv"), "a")
            f.write(fitness)
            f.close()
        # Calculate first convergences.
        averages = {}
        for mode in successes:
            count = 0
            total_timeout = 0
            total_time = 0
            for result in successes[mode]:
                count += 1
                total_timeout += successes[mode][result]["Timeout"]
                total_time += successes[mode][result]["Time"]
            averages[mode] = {"Timeout": total_timeout / count, "Time": total_time / count}
        s = "Mode,Average First Solution Time (ms),Average First Move Time (s)"
        for mode in averages:
            s += f"\n{mode},{averages[mode]['Timeout']},{averages[mode]['Time']}"
        f = open(os.path.join(os.getcwd(), "Results", robot, "Average Firsts.csv"), "w")
        f.write(s)
        f.close()


if __name__ == '__main__':
    try:
        evaluate()
    except KeyboardInterrupt:
        print("Evaluation stopped.")
    except ValueError as error:
        print(error)

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
    time = "" if success == 0 else time / success
    fitness = "" if success == rows else fitness / (rows - success)
    success = success / rows * 100
    print(f"Read {path}")
    return success, time, fitness


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
                success, time, fitness = read_file(path)
                f = open(os.path.join(os.getcwd(), "Results", robot, mode), "w")
                mode = mode.replace(".csv", "")
                if time == "":
                    print(f"{mode} | Success Rate (%) = {success}% | Fitness Score = {fitness}")
                    f.write(f"Success Rate (%),Fitness Score\n{success}%,{fitness}")
                else:
                    print(f"{mode} | Success Rate (%) = {success}% | Move Time (s) = {time} s | Fitness Score = {fitness}")
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
        modes = ["Bio IK", "Fusion IK Large", "Fusion IK Small", "Fusion IK Minimal", "Iterative Fusion IK Large", "Iterative Fusion IK Small", "Exhaustive Fusion IK Large", "Exhaustive Fusion IK Small"]
        longest = 0
        s = "Timeout (ms)"
        for mode in modes:
            s += f",{mode}"
            length = len(mode)
            if length > longest:
                longest = length
        for file in ["Success Rate (%).csv", "Move Time (s).csv", "Fitness Score.csv"]:
            f = open(os.path.join(os.getcwd(), "Results", robot, file), "w")
            f.write(s)
            f.close()
        results = dict(sorted(results.items()))
        for timeout in results:
            c_timeout = f"Timeout (ms) = {timeout} ms"
            padding = max(len(c_timeout), longest)
            c_timeout = c_timeout.ljust(padding)
            c_padding = "".ljust(padding, "-")
            success = {}
            time = {}
            fitness = {}
            titles = {"h": f"Timeout (ms) = {timeout} ms".ljust(padding)}
            for mode in modes:
                if results[timeout][mode] is None:
                    success[mode] = ""
                    time[mode] = ""
                    fitness[mode] = ""
                    continue
                success[mode] = f"{results[timeout][mode]['Success']}%"
                time[mode] = results[timeout][mode]["Time"]
                fitness[mode] = f"{results[timeout][mode]['Fitness']}"
                titles[mode] = mode.ljust(padding)
            s = f"\n{timeout}"
            padding = len("Success Rate (%)")
            for mode in success:
                s += f",{success[mode]}"
                length = len(success[mode])
                if length > padding:
                    padding = length
            f = open(os.path.join(os.getcwd(), "Results", robot, "Success Rate (%).csv"), "a")
            f.write(s)
            f.close()
            h_success = "Success Rate (%)".ljust(padding)
            p_success = "".ljust(padding, "-")
            for mode in success:
                success[mode] = success[mode].ljust(padding)
            s = f"\n{timeout}"
            padding = len("Move Time (s)")
            for mode in time:
                s += f",{time[mode]}"
                time[mode] = "-" if time[mode] == "" else f"{time[mode]} s"
                length = len(time[mode])
                if length > padding:
                    padding = length
            f = open(os.path.join(os.getcwd(), "Results", robot, "Move Time (s).csv"), "a")
            f.write(s)
            f.close()
            h_time = "Move Time (s)".ljust(padding)
            p_time = "".ljust(padding, "-")
            for mode in time:
                time[mode] = time[mode].ljust(padding)
            s = f"\n{timeout}"
            padding = len("Fitness Score")
            for mode in fitness:
                s += f",{fitness[mode]}"
                length = len(fitness[mode])
                if length > padding:
                    padding = length
            f = open(os.path.join(os.getcwd(), "Results", robot, "Fitness Score.csv"), "a")
            f.write(s)
            f.close()
            h_fitness = "Fitness Score".ljust(padding)
            p_fitness = "".ljust(padding, "-")
            for mode in fitness:
                fitness[mode] = fitness[mode].ljust(padding)
            s = (f"\n"
                 f"{c_timeout} | {h_success} | {h_time} | {h_fitness}\n"
                 f"{c_padding}-|-{p_success}-|-{p_time}-|-{p_fitness}-")
            for mode in success:
                s += f"\n{titles[mode]} | {success[mode]} | {time[mode]} | {fitness[mode]}"
            print(s)


if __name__ == '__main__':
    try:
        evaluate()
    except KeyboardInterrupt:
        print("Evaluation stopped.")
    except ValueError as error:
        print(error)

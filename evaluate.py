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
    fitness = 0 if success == rows else fitness / (rows - success)
    success = success / rows * 100
    print(f"Read {path}")
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
                if time == "":
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
        for file in ["Success Rate (%).csv", "Move Time (s).csv", "Fitness Score.csv"]:
            f = open(os.path.join(os.getcwd(), "Results", robot, file), "w")
            f.write("Timeout (ms),Bio IK,Fusion IK,Fusion IK Iterative")
            f.close()
        results = dict(sorted(results.items()))
        for timeout in results:
            b_success = results[timeout]["Bio IK"]["Success"]
            f_success = results[timeout]["Fusion IK"]["Success"]
            i_success = results[timeout]["Fusion IK Iterative"]["Success"]
            b_time = results[timeout]["Bio IK"]["Time"]
            f_time = results[timeout]["Fusion IK"]["Time"]
            i_time = results[timeout]["Fusion IK Iterative"]["Time"]
            b_fitness = results[timeout]["Bio IK"]["Fitness"]
            f_fitness = results[timeout]["Fusion IK"]["Fitness"]
            i_fitness = results[timeout]["Fusion IK Iterative"]["Fitness"]
            f = open(os.path.join(os.getcwd(), "Results", robot, "Success Rate (%).csv"), "a")
            f.write(f"\n{timeout},{b_success}%,{f_success}%,{i_success}%")
            f.close()
            f = open(os.path.join(os.getcwd(), "Results", robot, "Move Time (s).csv"), "a")
            f.write(f"\n{timeout},{b_time},{f_time},{i_time}")
            f.close()
            f = open(os.path.join(os.getcwd(), "Results", robot, "Fitness Score.csv"), "a")
            f.write(f"\n{timeout},{b_fitness},{f_fitness},{i_fitness}")
            f.close()
            b_success = f"{str(b_success)}%"
            f_success = f"{str(f_success)}%"
            i_success = f"{str(i_success)}%"
            b_time = "-" if b_time == "" else f"{str(b_time)} s"
            f_time = "-" if f_time == "" else f"{str(f_time)} s"
            i_time = "-" if i_time == "" else f"{str(i_time)} s"
            b_fitness = str(b_fitness)
            f_fitness = str(f_fitness)
            i_fitness = str(i_fitness)
            c = max(len(f"Timeout (ms) = {timeout} ms"), len("Fusion IK Iterative"))
            s = max(len("Success Rate (%)"), len(b_success), len(f_success), len(i_success))
            t = max(len("Move Time (s)"), len(b_time), len(f_time), len(i_time))
            f = max(len("Fitness Score"), len(b_fitness), len(f_fitness), len(i_fitness))
            c_h = f"Timeout (ms) = {timeout} ms".ljust(c)
            c_b = "Bio IK".ljust(c)
            c_f = "Fusion IK".ljust(c)
            c_i = "Fusion IK Iterative".ljust(c)
            c_p = "".ljust(c, "-")
            b_success = b_success.ljust(s)
            f_success = f_success.ljust(s)
            i_success = i_success.ljust(s)
            h_success = "Success Rate (%)".ljust(s)
            p_success = "".ljust(s, "-")
            b_time = b_time.ljust(t)
            f_time = f_time.ljust(t)
            i_time = i_time.ljust(t)
            h_time = "Move Time (s)".ljust(t)
            p_time = "".ljust(t, "-")
            b_fitness = b_fitness.ljust(f)
            f_fitness = f_fitness.ljust(f)
            i_fitness = i_fitness.ljust(f)
            h_fitness = "Fitness Score".ljust(f)
            p_fitness = "".ljust(f, "-")
            print(f"\n"
                  f"{c_h} | {h_success} | {h_time} | {h_fitness}\n"
                  f"{c_p}-|-{p_success}-|-{p_time}-|-{p_fitness}-\n"
                  f"{c_b} | {b_success} | {b_time} | {b_fitness}\n"
                  f"{c_f} | {f_success} | {f_time} | {f_fitness}\n"
                  f"{c_i} | {i_success} | {i_time} | {i_fitness}")


if __name__ == '__main__':
    try:
        evaluate()
    except KeyboardInterrupt:
        print("Evaluation stopped.")
    except ValueError as error:
        print(error)

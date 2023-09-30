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
                f = open(os.path.join(os.getcwd(), "Results", robot, "Network Testing.csv"), "w")
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
        success = "Timeout (ms),Bio IK,Fusion IK,Fusion IK Iterative,Fusion IK Improvement (%), Fusion IK Iterative Improvement (%)"
        time = "Timeout (ms),Bio IK,Fusion IK,Fusion IK Iterative,Fusion IK Improvement (%), Fusion IK Iterative Improvement (%)"
        fitness = "Timeout (ms),Bio IK,Fusion IK,Fusion IK Iterative,Fusion IK Improvement (%), Fusion IK Iterative Improvement (%)"
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
            x_success = f_success - b_success
            y_success = i_success - b_success
            x_time = (b_time - f_time) / f_time * 100
            y_time = (b_time - i_time) / i_time * 100
            x_fitness = (b_fitness - f_fitness) / f_fitness * 100
            y_fitness = (b_fitness - i_fitness) / i_fitness * 100
            success += f"\n{timeout},{b_success}%,{f_success}%,{x_success}%,{y_success}%"
            time += f"\n{timeout},{b_time},{f_time},{x_time}%,{y_time}%"
            fitness += f"\n{timeout},{b_fitness},{f_fitness},{x_fitness}%,{y_fitness}%"
            c = max(len(f"Timeout (ms) = {timeout} ms"), len("Fusion IK Iterative"))
            s = max(len("Success Rate (%)"), len(str(b_success)), len(str(f_success)), len(str(i_success)), len(str(x_success)), len(str(y_success)))
            t = max(len("Move Time (s)"), len(str(b_time)), len(str(f_time)), len(str(i_time)), len(str(x_time)), len(str(y_time)))
            f = max(len("Fitness Score"), len(str(b_fitness)), len(str(f_fitness)), len(str(i_fitness)), len(str(x_fitness)), len(str(y_fitness)))
            c_h = f"Timeout (ms) = {timeout} ms".ljust(c)
            c_b = "Bio IK".ljust(c)
            c_f = "Fusion IK".ljust(c)
            c_i = "Fusion IK Iterative".ljust(c)
            c_p = "".ljust(c, "-")
            c_s = "".ljust(c)
            b_success = str(b_success).ljust(s)
            f_success = str(f_success).ljust(s)
            i_success = str(i_success).ljust(s)
            x_success = str(x_success).ljust(s)
            y_success = str(y_success).ljust(s)
            h_success = "Success Rate (%)".ljust(s)
            p_success = "".ljust(s, "-")
            b_time = str(b_time).ljust(t)
            f_time = str(f_time).ljust(t)
            i_time = str(i_time).ljust(t)
            x_time = str(x_time).ljust(t)
            y_time = str(y_time).ljust(t)
            h_time = "Move Time (s)".ljust(t)
            p_time = "".ljust(t, "-")
            b_fitness = str(b_fitness).ljust(f)
            f_fitness = str(f_fitness).ljust(f)
            i_fitness = str(i_fitness).ljust(f)
            x_fitness = str(x_fitness).ljust(f)
            y_fitness = str(y_fitness).ljust(f)
            h_fitness = "Move Time (s)".ljust(f)
            p_fitness = "".ljust(f, "-")
            print(f"\n"
                  f"{c_h} | {h_success}  | {h_time}   | {h_fitness}\n"
                  f"{c_p}-|-{p_success}--|-{p_time}---|-{p_fitness}-\n"
                  f"{c_b} | {b_success}% | {b_time} s | {b_fitness}\n"
                  f"{c_f} | {f_success}% | {f_time} s | {f_fitness}\n"
                  f"{c_s} | {x_success}% | {x_time}%  | {x_fitness}%\n"
                  f"{c_i} | {i_success}% | {i_time} s | {i_fitness}\n"
                  f"{c_s} | {y_success}% | {y_time}%  | {y_fitness}%")
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

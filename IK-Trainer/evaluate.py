import argparse
import os.path

import pandas as pd


def evaluate(name: str):
    """
    Evaluate results.
    :param name: Name of the robot.
    :return: Nothing.
    """
    results = {}
    # Check that the data folder exists.
    root = os.path.join(os.getcwd(), "Evaluation")
    if not os.path.exists(root):
        return
    files = os.listdir(root)
    outputs = []
    # Loop all files.
    for file in files:
        # Only look for files with the desired name.
        if name not in file:
            continue
        # Load the data.
        df = pd.read_csv(os.path.join(root, file))
        # If there is no data, continue to the next file.
        rows = df.shape[0]
        if rows == 0:
            continue
        success = 0
        time = 0
        generations = 0
        distance = 0
        angle = 0
        # Sum the data.
        for index, data in df.iterrows():
            generations += data["Generations"]
            # Only add the time when the move was successful.
            if data["Success"] is True:
                success += 1
                time += data["Time"]
            # Only add the distance and angle when the move was unsuccessful.
            else:
                distance += data["Distance"]
                angle += data["Angle"]
        # Calculate the averages.
        time = "-" if success == 0 else time / success
        generations /= rows
        distance = "-" if success == rows else distance / (rows - success)
        angle = "-" if success == rows else angle / (rows - success)
        success = success / rows * 100
        # Determine the mode.
        strings = file.split()
        mode = strings[len(strings) - 2].replace("-", " ")
        max_generations = int(strings[len(strings) - 1].replace(".csv", ""))
        # Output to console.
        output = f"{name} | {max_generations} | {mode} | {success}%"
        if time != "-":
            output += f" | {time} s"
        output += f" | {generations}"
        if distance != "-":
            output += f" | {distance} m"
        if angle != "-":
            output += f" | {angle} °"
        outputs.append(output)
        # Append for file output.
        if max_generations not in results:
            results[max_generations] = {}
        results[max_generations][mode] = {"Success": success, "Time": time, "Generations": generations, "Distance": distance, "Angle": angle}
    # If there was no data to be written, exit.
    if len(results) == 0:
        return
    # Create the result folder.
    root = os.path.join(os.getcwd(), "Results")
    if not os.path.exists(root):
        os.mkdir(root)
    # Write the data to file.
    for max_generations in results:
        data = "Mode,Success Rate,Time,Generations,Distance,Angle"
        temp = results[max_generations]
        for mode in temp:
            result = temp[mode]
            data += f"\n{mode.replace('-', ' ')},{result['Success']}%"
            if result["Time"] == "-":
                data += ",-"
            else:
                data += f",{result['Time']} s"
            if result["Generations"] == "-":
                data += ",-"
            else:
                data += f",{result['Generations']}"
            if result["Distance"] == "-":
                data += ",-"
            else:
                data += f",{result['Distance']} m"
            if result["Angle"] == "-":
                data += ",-"
            else:
                data += f",{result['Angle']} °"
        f = open(os.path.join(root, f"{name} {max_generations}.csv"), "w")
        f.write(data)
        f.close()
    outputs.sort()
    output = outputs[0]
    for i in range(1, len(outputs)):
        output += f"\n{outputs[i]}"
    print(output)


if __name__ == '__main__':
    desc = "Inverse Kinematics Evaluator"
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=desc)
    parser.add_argument("name", type=str, help="Name of the robot.")
    a = vars(parser.parse_args())
    evaluate(a["name"])

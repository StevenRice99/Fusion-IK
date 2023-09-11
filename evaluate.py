import argparse
import os.path

import pandas as pd


def evaluate():
    """
    Evaluate results.
    :return: Nothing.
    """
    print("Fusion-IK Evaluation")
    # Check that the data folder exists.
    root = os.path.join(os.getcwd(), "Testing")
    if not os.path.exists(root):
        return
    robots = os.listdir(root)
    # Loop all files.
    for robot in robots:
        results = {}
        outputs = []
        files = os.listdir(os.path.join(root, robot))
        for name in files:
            # Load the data.
            df = pd.read_csv(os.path.join(root, robot, name))
            # If there is no data, continue to the next file.
            rows = df.shape[0]
            if rows == 0:
                continue
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
            time = "-" if success == 0 else time / success
            fitness = "-" if success == rows else fitness / (rows - success)
            success = success / rows * 100
            # Determine the mode.
            strings = name.split()
            mode = strings[len(strings) - 1].replace(".csv", "").replace("-", " ")
            generations = int(strings[len(strings) - 2])
            # Output to console.
            output = f"{robot} | {generations} | {mode} | {success}%"
            if time != "-":
                output += f" | {time} s"
            if fitness != "-":
                output += f" | {fitness}"
            outputs.append(output)
            # Append for file output.
            if generations not in results:
                results[generations] = {}
            results[generations][mode] = {"Success": success, "Time": time, "Fitness": fitness}
        # If there was no data to be written, exit.
        if len(results) == 0:
            continue
        # Create the result folder.
        root = os.path.join(os.getcwd(), "Results")
        if not os.path.exists(root):
            os.mkdir(root)
        root = os.path.join(root, robot)
        if not os.path.exists(root):
            os.mkdir(root)
        # Write the data to file.
        for generations in results:
            data = "Mode,Success Rate (%),Time (s),Fitness"
            temp = results[generations]
            for mode in temp:
                result = temp[mode]
                data += f"\n{mode.replace('-', ' ')},{result['Success']}%"
                if result["Time"] == "-":
                    data += ",-"
                else:
                    data += f",{result['Time']}"
                if result["Fitness"] == "-":
                    data += ",-"
                else:
                    data += f",{result['Fitness']}"
            f = open(os.path.join(root, f"{robot} {generations}.csv"), "w")
            f.write(data)
            f.close()
        output = outputs[0]
        for i in range(1, len(outputs)):
            output += f"\n{outputs[i]}"
        print(output)


if __name__ == '__main__':
    desc = "Fusion-IK Evaluation"
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=desc)
    evaluate()
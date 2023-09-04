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
            distance = 0
            angle = 0
            # Sum the data.
            for index, data in df.iterrows():
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
            distance = "-" if success == rows else distance / (rows - success)
            angle = "-" if success == rows else angle / (rows - success)
            success = success / rows * 100
            # Determine the mode.
            strings = name.split()
            mode = strings[len(strings) - 2].replace("-", " ")
            generations = int(strings[len(strings) - 1].replace(".csv", ""))
            # Output to console.
            output = f"{robot} | {name} | {generations} | {mode} | {success}%"
            if time != "-":
                output += f" | {time} s"
            if distance != "-":
                output += f" | {distance} m"
            if angle != "-":
                output += f" | {angle} °"
            outputs.append(output)
            # Append for file output.
            if generations not in results:
                results[generations] = {}
            results[generations][mode] = {"Success": success, "Time": time, "Distance": distance, "Angle": angle}
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
            data = "Mode,Success Rate,Time,Distance,Angle"
            temp = results[generations]
            for mode in temp:
                result = temp[mode]
                data += f"\n{mode.replace('-', ' ')},{result['Success']}%"
                if result["Time"] == "-":
                    data += ",-"
                else:
                    data += f",{result['Time']} s"
                if result["Distance"] == "-":
                    data += ",-"
                else:
                    data += f",{result['Distance']} m"
                if result["Angle"] == "-":
                    data += ",-"
                else:
                    data += f",{result['Angle']} °"
            f = open(os.path.join(root, f"{robot} {generations}.csv"), "w")
            f.write(data)
            f.close()
        outputs.sort()
        output = outputs[0]
        for i in range(1, len(outputs)):
            output += f"\n{outputs[i]}"
        print(output)


if __name__ == '__main__':
    desc = "Fusion-IK Evaluator"
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter, description=desc)
    evaluate()

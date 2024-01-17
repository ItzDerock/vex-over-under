import sys

import matplotlib.pyplot as plt


# Function to parse a dataset
def parse_dataset(data_lines):
    dataset = {}
    for line in data_lines:
        try:
            key, value = line.strip().split(":")
            # match first number in value
            value = value.split()[0]

            if key in dataset:
                dataset[key].append(float(value))
            else:
                dataset[key] = [float(value)]
        except:
            print("failed to parse " + line)

    if "output" in dataset:
        print("Total of {} outputs in dataset".format(len(dataset["output"])))

    return dataset


# Function to plot a dataset
def plot_dataset(dataset):
    # Plot distance and angular errors
    fig, (error, error2, output, output2) = plt.subplots(4)
    # plt.plot(dataset["distanceError"], label="Distance Error")
    # plt.plot(dataset["angularError"], label="Angular Error")
    # plt.legend()
    # plt.xlabel("Time")
    # plt.ylabel("Error")
    # plt.show()

    # Plot distance and angular errors
    if "distanceError" in dataset:
        error.plot(dataset["distanceError"], label="Distance Error")
        error.plot(dataset["angularError"], label="Angular Error")
    else:
        error.plot(dataset["distance error"], label="Distance Error")
        error2.plot(dataset["current error is"], label="Angular Error")
        error2.legend()
        output2.plot(dataset["turn out"], label="Output")
        output2.legend()

    error.legend()
    error.set_xlabel("Time")
    error.set_ylabel("Error")

    # Plot output
    # plt.plot(dataset["output"])
    # plt.xlabel("Time")
    # plt.ylabel("Output")
    # plt.show()

    # Plot output
    output.plot(dataset["output"])
    output.set_xlabel("Time")
    output.set_ylabel("Output")

    # Show the plot
    plt.show()


# Read the input file
with open(sys.argv[1], "r") as f:
    data_lines = []
    datasets = []
    for line in f:
        if line.startswith("targetX") or line.startswith("theta"):
            if data_lines:
                datasets.append(parse_dataset(data_lines))
            data_lines = []
        data_lines.append(line)

    # Check if there is a last dataset without a closing line
    if data_lines:
        datasets.append(parse_dataset(data_lines))

# Initialize variables
current_dataset = 0

# Main loop
while True:
    # Plot current dataset
    plot_dataset(datasets[current_dataset])

    # Get user input
    choice = input(
        "Enter 'n' for next dataset, 'p' for previous dataset, or 'q' to quit: "
    )

    # Switch datasets based on user input
    if choice == "n":
        current_dataset += 1
        if current_dataset >= len(datasets):
            current_dataset = 0
    elif choice == "p":
        current_dataset -= 1
        if current_dataset < 0:
            current_dataset = len(datasets) - 1
    elif choice == "q":
        break

    # Clear the plot for the next dataset
    plt.clf()

from common.config import config
from common.experiments import execute_experiment
from common.plot import *
from common.utils import save_to_file


def extract_number(string):
    return int(re.search(r"lan_(\d+)", string).group(1))


def remove_empty(data):
    return [x for x in data if x and x != "\n"]


def load_data():
    with open(config.get_default_results_file()) as f:
        lines = f.readlines()
        labels = remove_empty(lines[0].split(","))
        labels = [extract_number(label) for label in labels]

        data = lines[1:]
        data = data[0].split(",")
        data = remove_empty(data)
        data = [(labels[i], float(data[i])) for i in range(len(data))]
    return data


def main():
    config.init_config("alt_landmarks_number")

    execute_experiment(f"-q 4096 -r 3 -x 6")

    save_to_file(
        create_figure(create_axis(
            "Number of landmarks", "Cumulative time (ms)",
            create_plot(load_data(), "altcolor")
        ))
    )


main()

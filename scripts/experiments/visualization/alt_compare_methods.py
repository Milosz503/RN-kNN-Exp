from common.config import config
from common.csv_utils import read_csv_columns
from common.experiments import execute_experiment
from common.plot import *
from common.utils import save_to_file, add_marks
import pandas as pd

def load_data(experiment, network):
    data = read_csv_columns(config.get_results_file(experiment, network))

    headers = []
    plots = []
    for column in data:
        num_of_queries = 1
        name = (column[0].split("_")[5])
        headers.append(name)
        build_time = float(column[1])
        values = column[2:]
        plot = []
        for value in values:
            plot.append((num_of_queries, float(value) + build_time))
            num_of_queries *= 2
        plots.append(plot)

    return plots, headers

def load_plot(experiment, network, index):
    data, headers = load_data(experiment, network)
    return data[index], headers[index]

def load_experiments(experiments, network):
    plots = []
    headers = []
    for experiment, index, name in experiments:
        plot, header = load_plot(experiment, network, index)
        plots.append(plot)
        headers.append(f"{name}({header})")
    return plots, headers

experiments = [
    ("alt_dist_threshold_query_vs_time", 1, "DistS"),
    ("alt_hops_threshold_query_vs_time", 1, "HopsS"),
    ("alt_farthest", 1, "Far"),
    ("alt_avoid", 1, "Avoid"),
]
def main():
    plots, headers = load_experiments(experiments, config.network)
    plots = [add_marks(plot, mark) for plot, mark in zip(plots, scatter_classes)]

    save_to_file(
        create_figure(
            content=[
                create_axis(
                    "Number of queries",
                    "Cumulative time (ms)",
                    log_axis=True,
                    params="legend columns=2, legend pos=north west,",
                    content=[create_plot(plot, color, legend=header) for plot, header, color in
                             zip(plots, headers, color_classes)],
                ),
            ]
        )
    )

main()
from common.utils import save_to_file, add_marks
from common.config import config
from common.csv_utils import read_csv_columns
from common.experiments import execute, execute_experiment
from common.plot import *
import os
import pandas as pd

def load_data(experiment, network, clustered):
    data = pd.read_csv(config.get_results_file(experiment, network, clustered), header=0, delimiter=",")
    data = data.drop(columns=[col for col in data.columns if col.startswith("Unnamed")])

    if clustered:
        values_per_run = 16
    else:
        values_per_run = 14
    means = {}
    for col in data.columns:
        means[col] = []
        num_runs = len(data[col]) // values_per_run
        for i in range(values_per_run):
            values = []
            for j in range(num_runs):
                values.append(data[col][j * values_per_run + i])
            mean = sum(values) / len(values)
            means[col].append(mean)
    data = pd.DataFrame(means)

    headers = []
    plots = []
    for column in data.columns:
        num_of_queries = 1
        name = column
        headers.append(name)
        build_time = float(data[column][0])
        values = data[column][1:]
        plot = []
        for value in values:
            plot.append((num_of_queries, float(value) + build_time))
            num_of_queries *= 2
        plots.append(plot)

    return plots, headers

    # data = read_csv_columns(config.get_results_file(experiment, network, clustered))
    #
    # headers = []
    # plots = []
    # for column in data:
    #     num_of_queries = 1
    #     name = column[0]
    #     headers.append(name)
    #     build_time = float(column[1])
    #     values = column[2:]
    #     plot = []
    #     for value in values:
    #         plot.append((num_of_queries, float(value) + build_time))
    #         num_of_queries *= 2
    #     plots.append(plot)
    #
    # return plots, headers


def load_plot(experiment, network, index, clustered):
    data, headers = load_data(experiment, network, clustered)
    return data[index], headers[index]


def load_experiments(experiments, network, clustered=False):
    plots = []
    headers = []
    for experiment, index, name, header_index in experiments:
        plot, header = load_plot(experiment, network, index, clustered)
        print(header)
        info = header.split("_")[header_index]
        plots.append(plot)

        if name:
            headers.append(f"{name}-{info}")
        else:
            headers.append(f"{info}")
    return plots, headers

def create_comparison_plot(name, experiments, network, clustered=False,params=""):
    plots, headers = load_experiments(experiments, network, clustered)
    plots = [add_marks(plot, mark) for plot, mark in zip(plots, scatter_classes)]

    suffix=name + "_" + network
    if clustered:
        suffix = suffix + "_clustered"
    save_to_file(
        suffix=suffix,
        content=create_figure(
            content=[
                create_axis(
                    "Number of queries",
                    "Cumulative time (ms)",
                    log_axis=True,
                    params=f"legend columns=2, legend pos=north west,{params}",
                    content=[create_plot(plot, color, legend=header) for plot, header, color in
                             zip(plots, headers, color_classes)],
                ),
            ]
        )
    )

default_header_index = 5
all_experiments = [
    # experiment, column index, name, header index
    ("alt_compare_mixed", 0, "Mixed", default_header_index),
    ("alt_compare_mixed", 1, "MixedDecay", default_header_index),
    # ("alt_compare_mixed", 2, "HopsDecay", default_header_index),
    # ("alt_compare_mixed", 3, "MixedDecay", default_header_index),
    # ("alt_compare_mixed", 4, "Hops", default_header_index),
    # ("alt_compare_mixed", 5, "Hops", default_header_index),
]




def main():
    # execute_experiment(f"-q {config.default_query_number} -r {config.repeats} -x 26", executable=config.executable_alt_estimate)
    # execute_experiment(f"-q {config.clustered_query_number} -r {config.repeats} -x 26 -w 1")

    # create_comparison_plot("best", all_experiments, "W") #, params="xmin=10,xmax=2100")
    # create_comparison_plot("best", all_experiments, config.network, clustered=True) #, params="xmin=10,xmax=2100")

    create_comparison_plot("all", all_experiments, config.network, clustered=False,)
    # create_comparison_plot("best", all_experiments, "W", clustered=True)

main()

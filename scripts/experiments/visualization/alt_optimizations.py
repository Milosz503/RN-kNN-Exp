from common.config import config
from common.csv_utils import read_csv_columns
from common.experiments import execute_experiment
from common.plot import *
from common.utils import save_to_file, add_marks

def load_data(experiment, network, clustered):
    data = read_csv_columns(config.get_results_file(experiment, network, clustered))

    headers = []
    plots = []
    for column in data:
        num_of_queries = 1
        name = column[0]
        headers.append(name)
        build_time = float(column[1])
        values = column[2:]
        plot = []
        for value in values:
            plot.append((num_of_queries, float(value) + build_time))
            num_of_queries *= 2
        plots.append(plot)

    return plots, headers


def load_plot(experiment, network, index, clustered):
    data, headers = load_data(experiment, network, clustered)
    return data[index], headers[index]


def load_experiments(experiments, network, clustered=False):
    plots = []
    headers = []
    for experiment, index, name, header_index in experiments:
        plot, header = load_plot(experiment, network, index, clustered)
        plots.append(plot)

        if header_index < 0:
            headers.append(f"{name}")
        else:
            info = header.split("_")[header_index]
            if name:
                headers.append(f"{name}-{info}")
            else:
                headers.append(f"{info}")
    return plots, headers


default_header_index = -1
experiments_8 = [
    # experiment, column index, name, header index
    ("alt_optimizations", 0, "Default", default_header_index),
    ("alt_optimizations_dynamic", 0, "Dynamic", default_header_index),
    ("alt_optimizations_euclidean", 0, "Euclidean", default_header_index),
]

experiments_20 = [
    # experiment, column index, name, header index
    ("alt_optimizations", 1, "Default", default_header_index),
    ("alt_optimizations_dynamic", 1, "Dynamic", default_header_index),
    ("alt_optimizations_euclidean", 1, "Euclidean", default_header_index),
]


def create_comparison_plot(name, experiments, network, clustered=False):
    plots, headers = load_experiments(experiments, network, clustered)
    plots = [add_marks(plot, mark) for plot, mark in zip(plots, scatter_classes)]

    suffix=name + "_" + network
    if clustered:
        suffix = network + "_clustered"
    save_to_file(
        suffix=suffix,
        content=create_figure(
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
def main():
    execute_experiment(f"-q {config.default_query_number} -r {config.repeats} -x 24", executable=config.executable_alt_dynamic, suffix="_dynamic")
    execute_experiment(f"-q {config.default_query_number} -r {config.repeats} -x 24", executable=config.executable_alt_euclidean, suffix="_euclidean")
    execute_experiment(f"-q {config.default_query_number} -r {config.repeats} -x 24")
    # plots, headers = load_data()
    # plots = [add_marks(plot, mark) for plot, mark in zip(plots, scatter_classes)]
    #
    # save_to_file(
    #     create_figure(
    #         content=[
    #             create_axis(
    #                 "Number of queries",
    #                 "Cumulative time (ms)",
    #                 log_axis=True,
    #                 params="legend columns=2, legend pos=north west,",
    #                 content=[create_plot(plot, color, legend=header) for plot, header, color in
    #                          zip(plots, headers, color_classes)],
    #             ),
    #         ]
    #     )
    # )
    create_comparison_plot("8", experiments_8, config.network)
    create_comparison_plot("20", experiments_20, config.network)

main()

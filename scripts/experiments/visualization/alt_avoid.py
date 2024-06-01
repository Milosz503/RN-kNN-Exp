from common.utils import save_to_file, add_marks
from common.config import config
from common.csv_utils import read_csv_columns
from common.experiments import execute, execute_experiment
from common.plot import *
import os

def load_data():
    data = read_csv_columns(config.get_default_results_file())

    headers = []
    plots = []
    for column in data:
        num_of_queries = 1
        name = (column[0].split("_")[4])
        headers.append(name)
        build_time = float(column[1])
        values = column[2:]
        plot = []
        for value in values:
            plot.append((num_of_queries, float(value) + build_time))
            num_of_queries *= 2
        plots.append(plot)


    return plots, headers

def main():

    execute_experiment(f"-q {config.default_query_number} -r {config.repeats} -x 12")
    plots, headers = load_data()
    plots = [add_marks(plot, mark) for plot, mark in zip(plots, scatter_classes)]

    save_to_file(
        create_figure(
            content=[
                create_axis(
                    "Number of queries",
                    "Cumulative time (ms)",
                    log_axis=True,
                    params="legend columns=2, legend pos=north west,",
                    content=[create_plot(plot, color, legend=header) for plot, header, color in zip(plots, headers, color_classes)],
                ),
            ]
        )
    )

main()
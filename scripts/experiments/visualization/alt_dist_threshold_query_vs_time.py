from common.utils import save_to_file
from common.config import config
from common.csv_utils import read_csv_columns
from common.experiments import execute, execute_experiment
from common.plot import *
import os

def load_data():
    data = read_csv_columns(config.get_default_results_file())

    plots = []
    for column in data:
        num_of_queries = 1
        build_time = float(column[1])
        values = column[2:]
        plot = []
        for value in values:
            plot.append((num_of_queries, float(value) + build_time))
            num_of_queries *= 2
        plots.append(plot)
    return plots

def main():

    execute_experiment(f"-q {config.default_query_number} -r {config.repeats} -x 8")

    save_to_file(
        create_figure(
            content=[
                create_axis(
                    "Number of queries",
                    "Cumulative time (ms)",
                    log_axis=True,
                    content=[create_plot(plot, color) for plot, color in zip(load_data(), color_classes)],
                ),
            ]
        )
    )

main()
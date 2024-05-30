from common.config import config
from common.csv_utils import read_csv_columns
from common.experiments import execute_experiment
from common.plot import *
from common.utils import save_to_file

number_of_queries = config.default_query_number
def load_data():
    max_x = number_of_queries
    data = read_csv_columns(config.get_default_results_file())

    plots = []
    for column in data:
        values = column[1:]
        plot = []
        for i in range(len(values)):
            plot.append((float(values[i]), i + 1))
        plot.append((max_x, len(values)))
        plots.append(plot)
    return plots


def generate_plot():
    save_to_file(
        create_figure(
            create_axis(
                "Number of queries",
                "Number of landmarks",
                [create_plot(plot, color) for plot, color in zip(load_data(), color_classes)]
            ))
    )



def main():
    config.init_config("alt_min_dist_threshold_landmarks")
    execute_experiment(f"-q {number_of_queries} -r 1 -x 7")

    generate_plot()


main()
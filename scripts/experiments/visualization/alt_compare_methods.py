from common.config import config
from common.csv_utils import read_csv_columns
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
        info = header.split("_")[header_index]
        plots.append(plot)

        if name:
            headers.append(f"{name}-{info}")
        else:
            headers.append(f"{info}")
    return plots, headers


default_header_index = 5
all_experiments = [
    # experiment, column index, name, header index
    ("alt_dist_threshold_query_vs_time", 1, "DistS", default_header_index),
    ("alt_hops_threshold_query_vs_time", 1, "HopsS", default_header_index),
    ("alt_farthest", 1, "Far", default_header_index),

    # ("alt_avoid", 1, "Avoid", default_header_index),
    ("adaptive_dist_threshold_query_vs_time", 1, "ADistS", default_header_index),
    ("adaptive_hops_threshold_query_vs_time", 1, "AHopsS", default_header_index),
    ("alt_est_threshold_query_vs_time", 1, "AEstQ", default_header_index),
    ("other_methods", 0, None, 0),
    ("other_methods", 1, None, 0),
    ("other_methods", 2, None, 0),
]

alt_experiments = [
    ("alt_avoid", 1, "Avoid", default_header_index),
    ("alt_dist_threshold_query_vs_time", 1, "DistS", default_header_index),
    ("alt_dist_threshold_query_vs_time", 2, "DistS", default_header_index),
    ("alt_hops_threshold_query_vs_time", 1, "HopsS", default_header_index),
    ("alt_hops_threshold_query_vs_time", 2, "HopsS", default_header_index),
    ("alt_farthest", 1, "Far", default_header_index),
    ("alt_farthest", 2, "Far", default_header_index),
    ("alt_farthest", 3, "Far", default_header_index),
]

adaptive_experiments = [
    ("adaptive_dist_threshold_query_vs_time", 0, "ADistS", default_header_index),
    ("adaptive_dist_threshold_query_vs_time", 2, "ADistS", default_header_index),
    ("adaptive_hops_threshold_query_vs_time", 0, "AHopsS", default_header_index),
    ("adaptive_hops_threshold_query_vs_time", 2, "AHopsS", default_header_index),
    ("alt_est_threshold_query_vs_time", 1, "AEstQ", default_header_index),
]

best_methods = [
    # experiment, column index, name, header index
    ("alt_hops_threshold_query_vs_time", 1, "HopsS", default_header_index),
    ("alt_hops_threshold_query_vs_time", 2, "HopsS", default_header_index),
    ("alt_farthest", 2, "Far", default_header_index),
    ("alt_farthest", 3, "Far", default_header_index),
    ("adaptive_hops_threshold_query_vs_time", 0, "AHopsS", default_header_index),
    ("adaptive_hops_threshold_query_vs_time", 2, "AHopsS", default_header_index),
    ("other_methods", 3, None, 0),
    ("other_methods", 4, None, 0),
]

def create_comparison_plot(name, experiments, network, clustered=False,params=""):
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
                    params=f"legend columns=2, legend pos=north west,{params}",
                    content=[create_plot(plot, color, legend=header) for plot, header, color in
                             zip(plots, headers, color_classes)],
                ),
            ]
        )
    )
def main():
    create_comparison_plot("alt", alt_experiments, config.network, params="xmin=10,xmax=2100")
    create_comparison_plot("adaptive", adaptive_experiments, config.network)
    create_comparison_plot("best", best_methods, config.network)
    create_comparison_plot("best_zoom", best_methods, config.network, params="xmin=10,xmax=2100,ymin=500,ymax=40000,")



main()

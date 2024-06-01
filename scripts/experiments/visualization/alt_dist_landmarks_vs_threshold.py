from common.config import config
from common.experiments import execute_experiment
from common.plot import *
from common.utils import save_to_file
import pandas as pd


def load_data(network):
    data = pd.read_csv(f'{config.results_dir}/{network}_results_output.csv', header=0, delimiter=",")
    data = data.drop(columns=[col for col in data.columns if col.startswith("Unnamed")])
    plots = []
    plot = []
    y_values = []
    x_values = []
    for column in data.columns:
        threshold = float(column.split("_")[3])
        x_values.append(threshold)

    for i in range(len(x_values)):
        column = data.columns[i]
        y_values.append(data[column].mean())

        threshold = float(column.split("_")[3])
        for value in data[column]:
            plot.append((threshold, value))
    plots.append(plot)

    window = 1
    trend_line = pd.DataFrame({'x': x_values, 'y': y_values}).rolling(window=window, center=True).mean().dropna()
    trend_line = list(zip(trend_line['x'], trend_line['y']))

    return plots, trend_line


def main():
    for network in config.networks:
        execute_experiment(f"-q {config.default_query_number} -r 5 -x 15", network=network)
    networks_plots = []
    sampled_plots = []
    networks = config.networks
    for network in config.networks:
        points, trend_line = load_data(network)
        networks_plots.append(trend_line)
        sampled_plots.append(trend_line[::2])


    save_to_file(
        suffix="networks",
        content=create_figure(
            content=create_axis(
                "Threshold", "Number of landmarks",
                [create_plot(plot, color) for plot, color in zip(networks_plots, color_classes)] +
                [create_scatter(plot, marks, network) for plot, marks, network in zip(sampled_plots, scatter_classes, networks)]
            )

        )
    )

    plots, trend_line = load_data(config.network)
    save_to_file(
        suffix=config.network,
        content=create_figure(create_axis(
            "Threshold", "Number of landmarks",
            [create_plot(trend_line, "black", config.network)] +
            [create_scatter(plot, "blackdotsalpha") for plot in plots],
        ))
    )

main()

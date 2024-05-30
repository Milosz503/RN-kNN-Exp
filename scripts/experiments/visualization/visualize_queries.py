
from common.config import config
from common.experiments import execute_experiment
from graphs import graph_tools
import pandas as pd


network = config.network
def draw_queries(type):
    data = pd.read_csv(f'{config.results_dir}/{type}_query_data.csv', header=0, delimiter=",")
    graph_tools.visualize_queries(network, data, type)

execute_experiment(f"-q {config.default_query_number} -x 14", network=network)

draw_queries("random")
draw_queries("clustered")
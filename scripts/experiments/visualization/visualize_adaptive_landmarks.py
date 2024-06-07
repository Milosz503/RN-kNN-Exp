
from common.config import config
from common.experiments import execute_experiment
from graphs import graph_tools
import pandas as pd

network = config.network
execute_experiment(f"-q {config.default_query_number} -x 23", network=network, include_clustered=True)

data = pd.read_csv(f'{config.results_dir}/{network}_landmark_data.csv', header=0, delimiter=",")
data.columns = [header.split("_")[1] for header in data.columns]
queries_data = pd.read_csv(f'{config.results_dir}/{network}_query_data.csv', header=0, delimiter=",")

data_clustered = pd.read_csv(f'{config.results_dir}/clustered_{network}_landmark_data.csv', header=0, delimiter=",")
data_clustered.columns = [header.split("_")[1] for header in data_clustered.columns]
queries_data_clustered = pd.read_csv(f'{config.results_dir}/clustered_{network}_query_data.csv', header=0, delimiter=",")

for column in data.columns:
    graph_tools.visualize_landmarks(network, data[column].dropna(), column, queries_data=queries_data)
    graph_tools.visualize_landmarks(network, data_clustered[column].dropna(), column, queries_data=queries_data_clustered, clustered=True)
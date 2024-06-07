
from common.config import config
from common.experiments import execute_experiment
from graphs import graph_tools
import pandas as pd

network = config.network
execute_experiment(f"-q {config.default_query_number} -x 13", network=network, include_clustered=False)

data = pd.read_csv(f'{config.results_dir}/{network}_landmark_data.csv', header=0, delimiter=",")
data.columns = [header.split("_")[1] for header in data.columns]

for column in data.columns:
    graph_tools.visualize_landmarks(network, data[column].dropna(), column)
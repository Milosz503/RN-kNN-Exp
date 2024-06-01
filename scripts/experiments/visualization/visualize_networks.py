from common.config import config
from graphs import graph_tools


for network in config.networks:
    graph_tools.visualize_graph(network)

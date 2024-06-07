import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import pandas as pd
import os

from matplotlib.colors import ListedColormap

from common.config import config

matplotlib.rc('figure', figsize=(10, 10))

base_bins = 400

def draw_nodes_histogram(graph_file, ax):
    with open(graph_file, 'r') as file:
        data = pd.read_csv(file, header=0, delimiter=" ")
        data.columns = ['nodeId', 'x', 'y']
        x_min = data['x'].min()
        y_min = data['y'].min()

        x_max = data['x'].max()
        y_max = data['y'].max()

        cm = matplotlib.colormaps['Greys'].resampled(256)
        # cm = matplotlib.colormaps['YlOrRd'].resampled(256)
        # cm = matplotlib.colormaps['PuBu'].resampled(256)
        newcolors = cm(np.linspace(0.1, 1, 256))
        white = np.array([1, 1, 1, 1])
        newcolors[:1, :] = white
        newcmp = ListedColormap(newcolors)

        ax.set_xlim(0, data['x'].max())
        ax.set_ylim(0, data['y'].max())

        width = x_max - x_min
        height = y_max - y_min

        ax.set_aspect('equal', adjustable='box')
        ax.hist2d(data['x'], data['y'], bins=base_bins, cmap=newcmp)

        ax.set_xlim(x_min - 0.02*width, x_max + 0.02*width)
        ax.set_ylim(y_min - 0.02*height, y_max + 0.02*height)

        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['left'].set_visible(False)


def draw_landmarks(landmarks_data, ax):
    landmarks_x = []
    landmarks_y = []
    for landmark in landmarks_data:
        x = int(landmark.split("_")[0])
        y = int(landmark.split("_")[1])
        landmarks_x.append(x)
        landmarks_y.append(y)

    ax.scatter(
        landmarks_x,
        landmarks_y,
        c='black',
        s=200,
        alpha=1,
        edgecolor='white', linewidths=1,
        marker='^'
    )

    # for i in range(len(landmarks_x)):
    #     plt.text(landmarks_x[i], landmarks_y[i], str(i), c='red')


def draw_queries(queries_data, ax, color='black'):
    ax.scatter(
        queries_data['x'],
        queries_data['y'],
        c=color,
        s=1,
        alpha=0.3,
    )

def visualize_landmarks(network, landmarks_data, method, queries_data=None, clustered=False):
    graph_file = os.path.join(config.data_dir, f'{network}-t.coordinates')
    fig, ax = plt.subplots()
    draw_nodes_histogram(graph_file, ax)
    if queries_data is not None:
        draw_queries(queries_data, ax, color="#4287f5")
    draw_landmarks(landmarks_data, ax)
    # ax.set_title(method)
    prefix=""
    if clustered:
        prefix="clustered_"
    plt.savefig(f'{config.visualization_dir}/{prefix}{config.experiment_name}_{network}_{method}.png', bbox_inches='tight', pad_inches=0)
    # plt.show()

def visualize_queries(network, queries_data, method):
    graph_file = os.path.join(config.data_dir, f'{network}-t.coordinates')
    fig, ax = plt.subplots()
    draw_nodes_histogram(graph_file, ax)
    draw_queries(queries_data, ax)

    # ax.set_title(method)

    plt.savefig(f'{config.visualization_dir}/queries_{network}_{method}.png', bbox_inches='tight', pad_inches=0)
    plt.show()


def visualize_graph(network):
    graph_file = os.path.join(config.data_dir, f'{network}-t.coordinates')
    fig, ax = plt.subplots()
    draw_nodes_histogram(graph_file, ax)

    plt.savefig(f'{config.visualization_dir}/graph_{network}.png')
    # plt.show()








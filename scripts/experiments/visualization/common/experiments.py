import os

from common.config import config


def execute_experiment(params, network=None, include_clustered=False):
    if network is None:
        network = config.network

    if include_clustered is None:
        include_clustered = config.include_clustered

    if config.rerun_experiments:
        if config.include_random:
            execute(f"{config.executable} -c distance -g {config.graphs_dir}/{network}.bin"
                    f" -k 1 {params} {config.validate} -f {config.results_dir}")
        if include_clustered:
            execute(f"{config.executable} -c distance -g {config.graphs_dir}/{network}.bin"
                    f" -k 1 {params} {config.validate} -f {config.results_dir} -w 1")
    else:
        print("Skipping experiment execution")
def execute(command):
    print("Executing: " + command, flush=True)
    result = os.system(command)
    print("Result: " + str(result))
import os

from common.config import config


def execute_experiment(params, network=None):
    if network is None:
        network = config.network

    if config.rerun_experiments:
        execute(f"{config.executable} -c distance -g {config.graphs_dir}/{network}.bin"
                f" -k 1 {params} {config.validate} -f {config.results_dir}")
    else:
        print("Skipping experiment execution")
def execute(command):
    print("Executing: " + command, flush=True)
    result = os.system(command)
    print("Result: " + str(result))
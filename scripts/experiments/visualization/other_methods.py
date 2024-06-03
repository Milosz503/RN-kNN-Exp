from common.config import config
from common.experiments import execute_experiment

def main():
    execute_experiment(f"-q 2048 -r {config.repeats} -x 19")

main()

import os
import sys
class Config:
    def __init__(self):
        self.default_query_number=4096
        self.clustered_query_number=16384*2
        self.rerun_experiments = True
        self.validate = "-v 0"
        self.network = "NW"
        self.networks = ["ME", "NW", "W", "USA"]
        self.repeats = "10"
        self.include_random = True
        self.include_clustered = True

        self.results_dir = ""
        self.visualization_dir = ""
        self.graphs_dir = ""
        self.data_dir = ""
        self.experiment_name = ""
        self.executable = ""
        self.executable_alt_dynamic = ""
        self.executable_alt_euclidean = ""
        self.executable_alt_estimate = ""

    def init_config(self):
        experiment_name = os.path.splitext(os.path.basename(sys.argv[0]))[0]
        print("Experiment name: ", experiment_name)
        self.experiment_name = experiment_name
        self.results_dir = self._get_result_path()
        self.visualization_dir = self._get_visualization_path()
        self.graphs_dir = self._get_graphs_dir()
        self.data_dir = self._get_data_dir()
        self.executable = self._get_executable_dir() + "/nd_knn"
        self.executable_alt_dynamic = self._get_executable_dir() + "/nd_knn_alt_dynamic"
        self.executable_alt_euclidean = self._get_executable_dir() + "/nd_knn_alt_euclidean"
        self.executable_alt_estimate = self._get_executable_dir() + "/nd_knn_alt_estimate"

        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)

        if not os.path.exists(self.visualization_dir):
            os.makedirs(self.visualization_dir)

    def get_results_file(self, experiment_name, network, clustered=False):
        prefix = ""
        if clustered:
            prefix = "clustered_"
        return sys.argv[1] + "/results/" + experiment_name + f"/{prefix}{network}_results_output.csv"

    def get_default_results_file(self):
        return self.results_dir + f"/{config.network}_results_output.csv"


    def _get_result_path(self):
        if len(sys.argv) < 2:
            return "../results/" + self.experiment_name
        return sys.argv[1] + "/results/" + self.experiment_name

    def _get_visualization_path(self):
        if len(sys.argv) < 2:
            return "../visualization"
        return sys.argv[1] + "/visualization"

    def _get_graphs_dir(self):
        if len(sys.argv) < 3:
            raise ValueError("Graphs directory not provided")
        return sys.argv[2]

    def _get_data_dir(self):
        if len(sys.argv) < 5:
            raise ValueError("Data directory not provided")
        return sys.argv[4]

    def _get_executable_dir(self):
        if len(sys.argv) < 4:
            raise ValueError("Executable not provided")
        return sys.argv[3]


config = Config()
config.init_config()

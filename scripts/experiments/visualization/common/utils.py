from common.config import config

def tuples_to_string(tuples_list):
    # Convert each tuple to a string in the format (x, y)
    tuples_str = [f"({x}, {y})" for x, y in tuples_list]
    # Join the list of strings with newline characters
    result = "\n".join(tuples_str)
    return result

def save_to_file(content):
    output_file = config.visualization_dir + "/" + config.experiment_name + ".tex"
    with open(output_file, "w") as file:
        file.write(content)
from common.config import config

def tuples_to_string(tuples_list):
    # Convert each tuple to a string in the format (x, y)
    tuples_str = []
    for point in tuples_list:
        if len(point) == 2:
            tuples_str.append(f"({point[0]}, {point[1]})")
        else:
            tuples_str.append(f"({point[0]}, {point[1]}) [{point[2]}]")
    # tuples_str = [f"({x}, {y}, {mark})" for x, y in tuples_list]
    # Join the list of strings with newline characters
    result = "\n".join(tuples_str)
    return result

def tuples_to_string_scatter(tuples_list, class_name):
    # Convert each tuple to a string in the format (x, y)
    tuples_str = [f"({x}, {y}) [{class_name}]" for x, y in tuples_list]
    # Join the list of strings with newline characters
    result = "\n".join(tuples_str)
    return result

def save_to_file(content, suffix=""):
    if suffix:
        suffix = "_" + suffix
    output_file = config.visualization_dir + "/" + config.experiment_name + suffix + ".tex"
    with open(output_file, "w") as file:
        file.write(content)
    print(f"Saved to {output_file}")


def add_marks(tuples_list, mark, sampling_rate=1):
    tuples_str = []
    for i, point in enumerate(tuples_list):
        if i % sampling_rate == 0:
            tuples_str.append((point[0], point[1], mark))
        else:
            tuples_str.append((point[0], point[1]))
    return tuples_str
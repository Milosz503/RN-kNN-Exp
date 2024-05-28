from common import *

def extract_number(string):
    return int(re.search(r"lan_(\d+)", string).group(1))

def remove_empty(data):
    return [x for x in data if x and x != "\n"]

def load_data():
    with open(sys.argv[1]) as f:
        lines = f.readlines()
        labels = remove_empty(lines[0].split(","))
        labels = [extract_number(label) for label in labels]

        data = lines[1:]
        data = data[0].split(",")
        data = remove_empty(data)
        data = [(labels[i], float(data[i])) for i in range(len(data))]
    return data

save_to_file(
    "alt_landmarks_number",
    create_figure(create_axis(
        "Number of landmarks", "Cumulative time (ms)",
        create_plot(load_data(), "altcolor")
    ))
)

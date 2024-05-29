import csv
from itertools import zip_longest

def read_csv_columns(file_path):
    columns = []

    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        # Transpose the rows to columns
        rows = list(reader)
        columns = list(zip_longest(*rows, fillvalue=None))

    # Convert columns from tuples to lists and remove None values
    columns = [list(filter(None, column)) for column in columns]
    columns = [column for column in columns if column]

    return columns
import re
import sys

colors = '''
\definecolor{altcolor}{HTML}{1f78b4}
\definecolor{aaltcolor}{HTML}{33a02c}
\definecolor{gtree}{HTML}{4d9221}
'''


def create_figure(content):
    figure = ""
    figure += colors
    figure += r"\begin{tikzpicture}" + "\n"
    figure += content
    figure += r"\end{tikzpicture}" + "\n"
    return figure


def create_axis(x_label, y_label, content, log_axis=False):
    axis = "\n"
    axis += format(r'''
    \begin{axis}[
    xlabel={{{xlabel}}},
    ylabel={{{ylabel}}},
    % legend pos=north east,''',
                   {"xlabel": x_label, "ylabel": y_label})

    if log_axis:
        axis += r'''
        xmode=log,
        ymode=log,
        '''

    axis += r'''
    % xmin=1, xmax=1000,
    % mark size= 2.5pt,
    % line width=2pt,
    label style={font=\normalsize}
]
    '''

    axis += content
    axis += r'''
    \end{axis}
    '''
    return axis


def create_plot(values, color):
    values_str = tuples_to_string(values)
    plot = "\n" + format(
        r'''
    \addplot[mark=none, color={{color}}, line width=0.8pt] 
    coordinates{
    {{values}}
    };
''',
        {"values": values_str, "color": color}
    )
    return plot


def format(string, variables):
    # Define a regular expression pattern to match variables in the format {{var_name}}
    pattern = r"\{\{(\w+)\}\}"

    # Define a function to replace each match with its corresponding value from the variables dictionary
    def replace(match):
        var_name = match.group(1)
        return variables.get(var_name, match.group(0))

    # Use re.sub() to replace all occurrences of variables in the string
    return re.sub(pattern, replace, string)


def tuples_to_string(tuples_list):
    # Convert each tuple to a string in the format (x, y)
    tuples_str = [f"({x}, {y})" for x, y in tuples_list]
    # Join the list of strings with newline characters
    result = "\n".join(tuples_str)
    return result

def save_to_file(filename, content):
    if len(sys.argv) < 3:
        path = "."
    else:
        path = sys.argv[2]
    output_file = path + "/" + filename + ".tex"
    with open(output_file, "w") as file:
        file.write(content)
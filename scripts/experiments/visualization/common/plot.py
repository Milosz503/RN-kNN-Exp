from common.utils import tuples_to_string, tuples_to_string_scatter
import re

colors_definitions = '''
\definecolor{altcolor}{HTML}{1f78b4}
\definecolor{aaltcolor}{HTML}{33a02c}
\definecolor{gtree}{HTML}{4d9221}


\definecolor{class1color}{HTML}{e41a1c}
\definecolor{class2color}{HTML}{377eb8}
\definecolor{class3color}{HTML}{4daf4a}
\definecolor{class4color}{HTML}{984ea3}
\definecolor{class5color}{HTML}{ff7f00}
\definecolor{class6color}{HTML}{ffff33}
\definecolor{class7color}{HTML}{a65628}
\definecolor{class8color}{HTML}{f781bf}
\definecolor{class9color}{HTML}{999999}
'''


color_classes = [
    "class1color",
    "class2color",
    "class3color",
    "class4color",
    "class5color",
    "class6color",
    "class7color",
    "class8color",
    "class9color",
] * 3

scatter_classes=r'''
   scatter/classes={
            blackdotsalpha={mark=*, black, mark size=1.5pt, opacity=0.2, draw opacity=0.2,},
            airlin={mark=diamond, airlincolor},
            air={mark=diamond, airlincolor},
            rstar={mark=star, rstarcolor},
            rquad={mark=pentagon, rquadcolor},
            cs={mark=triangle, cscolor},
            csr={mark=star, csrcolor},
            gq={mark=x, gqcolor},
            gqr={mark=o, gqrcolor},
            gs={mark=oplus, gscolor},
            gsr={mark=square, gsrcolor},
            cq={mark=otimes, cqcolor},
            cqr={mark=halfcircle, cqrcolor},
            gsrc={mark=Mercedes star, gsrccolor},
            gsrp={mark=+, gsrpcolor},
	    quadtree={mark=o, quadtreecolor}
	    },'''

def append_content(current_content, new_content):
    if isinstance(new_content, list):
        for item in new_content:
            current_content += item
            current_content += "\n"
    else:
        current_content += new_content
    return current_content

def create_figure(content):
    figure = ""
    figure += colors_definitions
    figure += r"\begin{tikzpicture}" + "\n"
    figure = append_content(figure, content)
    figure += r"\end{tikzpicture}" + "\n"
    return figure


def create_axis(x_label, y_label, content, log_axis=False):
    axis = "\n"
    axis += format(r'''
    \begin{axis}[
    width=\textwidth,
    xlabel={{{xlabel}}},
    ylabel={{{ylabel}}},
    % legend pos=north east,''',
                   {"xlabel": x_label, "ylabel": y_label})

    if log_axis:
        axis += r'''
        xmode=log,
        ymode=log,'''

    axis += scatter_classes

    axis += r'''
    % xmin=1, xmax=1000,
    % mark size= 2.5pt,
    % line width=2pt,
    label style={font=\normalsize}
]
    '''
    axis = append_content(axis, content)
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


def create_scatter(values, class_name):
    values_str = tuples_to_string_scatter(values, class_name)
    plot = "\n" + format(
        r'''
    \addplot[
        scatter,only marks,
        scatter src=explicit symbolic] 
    coordinates{
    {{values}}
    };
''',
        {"values": values_str}
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

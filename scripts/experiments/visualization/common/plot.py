from common.utils import tuples_to_string, tuples_to_string_scatter
import re

colors_definitions = r'''
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

scatter_definitions=r'''
   scatter/classes={
            mark1={mark=diamond, class1color},
            mark2={mark=diamond, class2color},
            mark3={mark=star, class3color},
            mark4={mark=pentagon, class4color},
            mark5={mark=triangle, class5color},
            mark6={mark=star, class6color},
            mark7={mark=x, class7color},
            mark8={mark=o, class8color},
            mark9={mark=oplus, class9color},
            blackdotsalpha={mark=*, black, mark size=1pt, opacity=0.2, draw opacity=0.2},
            none={mark=none}
	    },'''

scatter_classes = [
    "mark1",
    "mark2",
    "mark3",
    "mark4",
    "mark5",
    "mark6",
    "mark7",
    "mark8",
    "mark9",
]

def append_content(current_content, new_content):
    if isinstance(new_content, list):
        for item in new_content:
            current_content += item
            current_content += "\n"
    else:
        current_content += new_content
    return current_content

def create_figure(content):
    figure = r'''
    \ifhideplots
        \emptyplot
    \else
    '''
    figure += colors_definitions
    figure += r"\begin{tikzpicture}" + "\n"
    figure = append_content(figure, content)
    figure += r"\end{tikzpicture}" + "\n"
    figure += r"\fi" + "\n"
    return figure


def create_axis(x_label, y_label, content, log_axis=False, params=None):
    if params and not params.endswith(","):
        params += ","
    axis = "\n"
    axis += format(r'''
    \begin{axis}[
    % width=\textwidth,
    xlabel={{{xlabel}}},
    ylabel={{{ylabel}}},{{params}}
    % legend pos=north east,''',
                   {
                       "params": params,
                       "xlabel": x_label,
                       "ylabel": y_label
                   })

    if log_axis:
        axis += r'''
        xmode=log,
        ymode=log,'''

    axis += scatter_definitions

    axis += r'''
    % xmin=1, xmax=1000,
    % mark size= 2.5pt,
    % line width=2pt,
    legend style={font=\tiny},
    tick label style={font=\small},
    label style={font=\normalsize}
]
    '''
    axis = append_content(axis, content)
    axis += r'''
    \end{axis}
    '''
    return axis


def create_plot(values, color, legend=None, line_width="0.8pt"):
    values_str = tuples_to_string(values)

    forget = None
    if not legend:
        forget = ", forget plot"

    scatter = None
    if any(len(t) == 3 for t in values):
        scatter = "scatter,"
    plot = "\n" + r'''
    \addplot[{{scatter}}mark=none, color={{color}}, line width={{line_width}}, scatter src=explicit symbolic{{forget}}] 
    coordinates{
    {{values}}
    };
'''
    if legend:
        plot += r'''
            \addlegendentry{{{legend}}}
            '''
    plot = format(plot, {
        "scatter": scatter,
        "line_width": line_width,
        "values": values_str,
        "color": color,
        "legend": legend,
        "forget": forget
    })
    return plot


def create_scatter(values, class_name, legend=None):
    values_str = tuples_to_string_scatter(values, class_name)

    forget = None
    if not legend:
        forget = ", forget plot"
    plot = "\n" + r'''
    \addplot[
        scatter,only marks,
        scatter src=explicit symbolic{{forget}}] 
    coordinates{
    {{values}}
    };
'''
    if legend:
        plot += r'''
            \addlegendentry{{{legend}}}
            '''
    plot = format(plot, {
        "values": values_str,
        "legend": legend,
        "forget": forget
    })
    return plot


def format(string, variables):
    variables = {k: v for k, v in variables.items() if v is not None}
    # Define a regular expression pattern to match variables in the format {{var_name}}
    pattern = r"\{\{(\w+)\}\}"

    # Define a function to replace each match with its corresponding value from the variables dictionary
    def replace(match):
        var_name = match.group(1)
        value = variables.get(var_name, None)
        if value is None:
            return ""
        return value

    # Use re.sub() to replace all occurrences of variables in the string
    return re.sub(pattern, replace, string)

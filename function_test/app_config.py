import ttkbootstrap as ttk

dark_theme = True
dark_theme_use = "darkly"
light_theme_use = "litera"

if dark_theme:
    inherit_style = "dark"
else:
    inherit_style = "light"

free_style = str.format(f"free.{inherit_style}.TButton")
obs_style = str.format(f"obs.{inherit_style}.TButton")
transition_style = "tran.dark.TButton"

start_style = str.format(f"start.{inherit_style}.TButton")
goal_style = str.format(f"goal.{inherit_style}.TButton")

pipe_style = "pipe.success.TButton"
att_style = "att.success.TButton"
fit_style = "fit.success.TButton"


fit_deviated_style = "fitdev.warning.TButton"
att_deviated_style = "attdev.warning.TButton"
pipe_deviated_style = "pipedev.warning.TButton"
fit_misplaced_style = "fitmis.warning.TButton"
att_misplaced_style = "attmis.warning.TButton"
pipe_misplaced_style = "pipemis.warning.TButton"
highlight_next_rec_action_style = "nextrecact.primary.TButton"
treeview_style = "treestyle.Treeview"



obs_color = "orange"
obs_active_color = "orange2"

transition_color = "black"
transition_active_color = "grey20"

start_color = "green"
start_active_color = "green2"
goal_color = "red2"
goal_active_color = "red3"

pipe_color = "blue"
pipe_active_color = "blue2"
fit_color = "saddle brown"
fit_active_color = "sandy brown"
att_color = "magenta"
att_active_color = "magenta2"

deviated_color = "yellow"

deviated_color2 = "gold3"

button_width = 6  #: Width of the grid buttons.
button_height = 10  #: Height of the grid buttons.
font = ('Tahoma', 6)  #: Tuple containing :obj:`str` of font art and :obj:`int` of font size.


def configure_style(style: ttk.Style):
    """Sets the style for the process visualization and interface. If ttkbootstrap theme is not used
    the look of the buttons will depend on the version and kind of operating system used!

    Args:
        style(:obj:`tkinter.ttk.Style`): Tkinter Style object.
    """

    style.configure(free_style,  width=button_width, height=button_height,  font=font)
    style.configure(pipe_style, background=pipe_color, width=button_width, height=button_height, font=font)
    style.configure(obs_style, background=obs_color, width=button_width, height=button_height, font=font)
    style.configure(fit_style, background=fit_color, width=button_width, height=button_height, font=font)
    style.configure(start_style, background=start_color, width=button_width, height=button_height, font=font)
    style.configure(goal_style, background=goal_color, width=button_width, height=button_height, font=font)
    style.configure(transition_style, background=transition_color, width=button_width, height=button_height, font=font)
    style.configure(att_style, background=att_color, width=button_width, height=button_height, font=font)
    style.configure(fit_deviated_style, background=fit_color,  width=button_width,
                    height=button_height,
                    font=font)
    style.configure(att_deviated_style, background=att_color,  width=button_width,
                    height=button_height,
                    font=font)
    style.configure(pipe_deviated_style, background=pipe_color,  width=button_width,
                    height=button_height,
                    font=font)
    style.configure(fit_misplaced_style, background=fit_color,  width=button_width,
                    height=button_height,
                    font=font)
    style.configure(att_misplaced_style, background=att_color,  width=button_width,
                    height=button_height,
                    font=font)
    style.configure(pipe_misplaced_style, background=pipe_color,  width=button_width,
                    height=button_height,
                    font=font)

    style.map(pipe_style, background=[("active", pipe_active_color)])
    style.map(obs_style, background=[("active", obs_active_color)])
    style.map(fit_style, background=[("active", fit_active_color)])
    style.map(start_style, background=[("active", start_active_color)])
    style.map(goal_style, background=[("active", goal_active_color)])
    style.map(transition_style, background=[("active", transition_active_color)])
    style.map(att_style, background=[("active", att_active_color)])
    style.map(fit_deviated_style, background=[("active", fit_active_color)])
    style.map(att_deviated_style, background=[("active", att_active_color)])
    style.map(pipe_deviated_style, background=[("active", pipe_active_color)])
    style.map(fit_misplaced_style, background=[("active", fit_active_color)])

    style.map(att_misplaced_style, background=[("active", "")])
    style.map(pipe_misplaced_style, background=[("active", "")])


    style.configure(treeview_style)
    style.layout(treeview_style, [(treeview_style + '.treearea', {'sticky': 'nswe'})])
    style.configure("TRadiobutton", anchor="W")

    if dark_theme:
        pass
        style.configure("TFrame", background="grey12")
        style.configure("Treeview", foreground="grey51", background="grey12")
        style.configure("TLabelframe", foreground="white", background="grey12")
        style.configure("TLabelframe.Label", foreground="grey51", background="grey12")
        style.configure("TRadiobutton", background="grey12", foreground="grey51")


class GridButton(ttk.Button):
    def __init__(self, frame, text, style):
        super(GridButton, self).__init__(frame, text=text, style=style)



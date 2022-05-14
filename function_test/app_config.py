import tkinter.ttk
from tkinter import ttk

free_style = "FREE.TButton"
pipe_style = "PIPE.TButton"
obs_style = "OBSTACLE.TButton"
att_style = "ATTACHMENT.TButton"
fit_style = "CORNER.TButton"
start_style = "START.TButton"
goal_style = "GOAL.TButton"
transition_style = "TRANSITION.TButton"
fit_deviated_style = "FITDEV.TButton"
att_deviated_style = "ATTDEV.TButton"
pipe_deviated_style = "PIPEDEV.TButton"
fit_misplaced_style = "FITMIS.TButton"
att_misplaced_style = "ATTMIS.TButton"
pipe_misplaced_style = "PIPEMIS.TButton"
highlight_next_rec_action_style = "NEXTRECACT.TButton"
treeview_style = "TREESTYLE.Treeview"

dark_theme = False


def configure_visualization_style(button_height:int, button_width:int, font:tuple[str, int], style: tkinter.ttk.Style):
    """Sets the style for the process visualization.

    Args:
        button_width(:obj:`int`): Width of the grid buttons.
        button_height(:obj:`int`): Height of the grid buttons.
        style(:obj:`tkinter.ttk.Style`): Tkinter Style object.
        font(:obj:`str`): Tuple containing :obj:`str` of font art and :obj:`int` of font size.


    """
    style.configure(free_style, background="white", width=button_width, height=button_height, font=font)
    style.configure(pipe_style, background="blue", width=button_width, height=button_height, font=font)
    style.configure(obs_style, background="orange", width=button_width, height=button_height, font=font)
    style.configure(fit_style, background="cyan", width=button_width, height=button_height, font=font)
    style.configure(start_style, background="green", width=button_width, height=button_height, font=font)
    style.configure(goal_style, background="red", width=button_width, height=button_height, font=font)
    style.configure(transition_style, background="black", width=button_width, height=button_height, font=font)
    style.configure(att_style, background="magenta", width=button_width, height=button_height, font=font)
    style.configure(fit_deviated_style, background="cyan", foreground="yellow", width=button_width,
                    height=button_height,
                    font=font)
    style.configure(att_deviated_style, background="magenta", foreground="yellow", width=button_width,
                    height=button_height,
                    font=font)
    style.configure(pipe_deviated_style, background="blue", foreground="yellow", width=button_width,
                    height=button_height,
                    font=font)
    style.configure(fit_misplaced_style, background="cyan", foreground="yellow", width=button_width,
                    height=button_height,
                    font=font)
    style.configure(att_misplaced_style, background="magenta", foreground="yellow", width=button_width,
                    height=button_height,
                    font=font)
    style.configure(pipe_misplaced_style, background="blue", foreground="yellow", width=button_width,
                    height=button_height,
                    font=font)
    style.configure(treeview_style)
    style.layout(treeview_style, [(treeview_style + '.treearea', {'sticky': 'nswe'})])
    style.configure("TRadiobutton", anchor="W")

    if dark_theme:
        style.configure("TFrame", background="grey12")
        style.configure("TButton", background="black", foreground="black")
        style.configure(free_style, background="black", foreground="black")
        style.configure(transition_style, background="tan4", foreground="black")
        style.configure("Treeview", foreground="grey51", background="grey12")
        style.configure("TLabelframe", foreground="white", background="grey12")
        style.configure("TLabelframe.Label", foreground="grey51", background="grey12")
        style.configure("TRadiobutton", background="grey12", foreground="grey51")

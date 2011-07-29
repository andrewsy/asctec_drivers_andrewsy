import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

def _equal_axes(ax):
    ax.set_aspect('equal')
    ax.set_aspect('equal', adjustable='datalim')
    
def newfig(title="", xlabel="", ylabel="", equal_axes=False, grid=True, hold=True):
    fig = plt.figure()
    plt.plot(0,0,'r')
    ax = plt.gca()
    ax.set_title(title)
    if equal_axes:
        _equal_axes(ax)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    plt.grid(grid)
    plt.hold(hold)
    fmwin = plt.get_current_fig_manager().window
    fignum_txt = fmwin.title()
    fmwin.title(fignum_txt + ': ' + title)
    return ax

def plot_traj(x,y, *args, **kwargs):
    plt.plot(x, y, *args, **kwargs)
    plt.plot(x[0], y[0], 'go')
    plt.plot(x[-1], y[-1], 'rx')
    
def show():
    plt.show()

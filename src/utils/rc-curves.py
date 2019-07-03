#!/usr/bin/python

import matplotlib.pyplot as plt
import numpy as np
import sys

EXPO = [0, 0.5, 1]
MID = [0.5, 0, 1]

YLABEL = 'Output'
XLABEL = 'Input'

def thr(expo, mid):
    def curve(x):
        if x < 0:
            return -curve(-x)

        tmp = x - mid

        y = 1
        if tmp > 0:
            y = 1 - mid
        elif tmp < 0:
            y = mid

        return mid + tmp * (1 - expo + expo * tmp**2 / y**2)
    return curve

def rpy(expo):
    def curve(x):
        return (1 + expo * (x * x - 1)) * x
    return curve


def input_range():
    return np.arange(-1, 1, 0.01)

def plot_rpy(expo_values):
    plotted = []
    legend = []

    x = input_range()
    for expo in expo_values:
        y = map(rpy(expo), x)
        plotted.append(x)
        plotted.append(y)
        legend.append('expo = %s' % (expo))

    plt.plot(*plotted)
    plt.title('Roll/Pitch/Yaw')
    plt.legend(legend, fontsize='xx-small', loc=2)
    plt.grid()


def plot_thr(expo_values, mid_values):
    plotted = []
    legend = []

    x = input_range()
    for expo in expo_values:
        for mid in mid_values:
            y = map(thr(expo, mid), x)
            plotted.append(x)
            plotted.append(y)
            legend.append('expo = %s, mid = %s' % (expo, mid))
            # Plot only one mid value with expo = 0, since
            # mid does nothing in that case
            if expo == 0:
                break

    plt.plot(*plotted)
    plt.title('Throttle')
    plt.legend(legend, fontsize='xx-small', loc=2)
    plt.grid()

def plot_all():
    plt.subplot(2, 1, 1)
    plot_rpy(EXPO)
    plt.ylabel(YLABEL)
    plt.subplot(2, 1, 2)
    plot_thr(EXPO, MID)
    plt.xlabel(XLABEL)
    plt.ylabel(YLABEL)

def main():
    if len(sys.argv) > 1:
        if sys.argv[1] == 'rpy':
            expo = EXPO
            if len(sys.argv) > 2:
                expo = [float(sys.argv[2])]
            plot_rpy(expo)
            plt.xlabel(XLABEL)
            plt.ylabel(YLABEL)
        elif sys.argv[1] == 'thr':
            expo = EXPO
            mid = MID
            if len(sys.argv) > 2:
                expo = [float(sys.argv[2])]
            if len(sys.argv) > 3:
                mid = [float(sys.argv[3])]
            plot_thr(expo, mid)
            plt.xlabel(XLABEL)
            plt.ylabel(YLABEL)
        elif sys.argv[1] == 'all':
            plot_all()
        else:
            print('Invalid plot type. Valid ones are rpy, thr and all')
            sys.exit(1)
    else:
        plot_all()
    plt.show()

if __name__ == '__main__':
    main()

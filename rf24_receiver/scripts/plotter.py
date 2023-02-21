#!/usr/bin/env python3

import serial
import sys
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# disable toolbar
plt.rcParams['toolbar'] = 'None'

class Plotter:
    def __init__(self, serial, ax):
        self.serial = serial
        self.ax = ax
        self.maxt = 250
        self.tdata = [0]
        self.ydata = [3.3/2]
        self.line = Line2D(self.tdata, self.ydata)

        self.ax.add_line(self.line)
        self.ax.set_ylim(0, 3.3)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt - self.tdata[0] >= self.maxt:  # drop old frames
            self.tdata = self.tdata[1:]
            self.ydata = self.ydata[1:]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)

        t = lastt + 1
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,

    def serial_getter(self):
        # note sometimes UART drops chars so we try a max of 5 times
        # to get proper data
        while True:
            for i in range(5):
                line = self.serial.readline()
                try:
                    line = float(line)
                except ValueError:
                    continue
                break
            yield line

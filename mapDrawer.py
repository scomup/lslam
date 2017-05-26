#!/usr/bin/env python
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import Queue
from threading import Thread
import time
from numpy.random import *
import signal
import sys
hexByName = {"black" 	    : "#000000",
             "mediumgreen" 	: "#47b73b",
             "lightgreen" 	: "#7CCF6F",
             "darkblue" 	: "#5D4EFF",
             "lightblue" 	: "#8072FF",
             "darkred" 	    : "#B66247",
             "cyan"         : "#5DC8ED",
             "mediumred" 	: "#D76B48",
             "lightred" 	: "#FB8F6C",
             "darkyellow" 	: "#C3CD41",
             "lightyellow" 	: "#D3DA76",
             "darkgreen" 	: "#3E9F2F",
             "magenta" 	    : "#B664C7",
             "gray"         : "#cccccc",
             "white" 	    : "#ffffff"}

colorAssoc = {
             0: "white",
             1: "lightgreen",
             2: "darkred",
             3: "darkgreen",
    }
lut = np.zeros((4, 3), dtype=np.ubyte)
rgbByName = {key: np.array((int(value[1:3], 16), int(value[3:5], 16), int(value[5:7], 16)))
             for key, value in hexByName.items()}

for key, val in colorAssoc.items():
    lut[key, :] = rgbByName[val]

class mapDrawer:
    
    def __init__(self, windowTitle="mapDrawer"):
        self.ports = []
        self.timer = pg.QtCore.QTimer()
        self.win = pg.GraphicsWindow(windowTitle)
        self.plotItem = pg.PlotItem()
        self.plotItem.setXRange(0,5)
        self.plotItem.setYRange(0,5)
        self.plotItem.setAspectLocked(True)
        self.scatterPlotItem = pg.ScatterPlotItem()
        
        self.plotItem.addItem(self.scatterPlotItem)
        self.imageItem = pg.ImageItem(autoRange=False,autoLevels=False)
        self.imageItem.setLevels([0.,1000.])
        #self.imageItem.setLookupTable(lut)
        #self.imageItem.setZValue(-100)
        self.plotItem.addItem(self.imageItem)
        self.win.addItem(self.plotItem)
        self.timer.timeout.connect(self.update)
        self.timer.start(0)
        self.q = Queue.Queue()

    def getPort(self):
        return self.q

    def update(self):
        try:
            data,path = self.q.get(block=False)
            self.imageItem.setImage(data, autoLevels=True)
            #self.scatterPlotItem.addPoints(x = [path[ 0 ]], y = [path[ 1 ]])
        except Queue.Empty:
            pass

    def run(self):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        app = QtGui.QApplication.instance()
        #sys.exit()
        app.exec_()




if __name__ == "__main__":
    p = mapDrawer()
    q = p.getPort()
    q.put( (np.random.rand(5,5)*0.1,(10,10)))
    p.run()
    pass
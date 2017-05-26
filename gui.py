# -*- coding: utf-8 -*-


from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import signal
import Queue
import sys

class mapDrawer:
    def __init__(self, windowTitle="viewer"):
        ## Open a windows
        app = QtGui.QApplication([])
        w = pg.GraphicsView()
        w.show()
        w.resize(800,800)
        w.setWindowTitle(windowTitle)
        view = pg.ViewBox()
        w.setCentralItem(view)

        ## lock the aspect ratio
        view.setAspectLocked(True)

        ## Create image item
        self.img = pg.ImageItem(np.zeros((400,400)))
        view.addItem(self.img)
        

        ## Set initial view bounds
        view.setRange(QtCore.QRectF(0, 0, 400, 400))
        
        self.img.setLevels([0, 1])

        #Set timer
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(30)
        
        #Data queue
        self.q = Queue.Queue()

        ## Start Qt event loop unless running in interactive mode or using pyside.
        if __name__ == '__main__':
            import sys
            if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
                QtGui.QApplication.instance().exec_()
              
    def update(self):
        try:
            #self.img.setImage(np.random.rand(400,400))
            data = self.q.get(block=False)
            self.imageItem.setImage(data, autoLevels=True)
            #self.scatterPlotItem.addPoints(x = [path[ 0 ]], y = [path[ 1 ]])
        except Queue.Empty:
            pass

    def setdata(self, data):
        self.q.put( data )

gui = mapDrawer()

for i in range(10):
    

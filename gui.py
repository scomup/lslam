#!/usr/bin/python
# coding: UTF-8



from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import signal
import Queue
import sys
import time
import threading

class LSLAMGUI(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        
    def run(self):
        app = QtGui.QApplication([])
        w = pg.GraphicsWindow()
        w.show()
        w.resize(800,800)
        w.setWindowTitle("viewer")
        vb = w.addViewBox()
        self.img = pg.ImageItem(np.zeros((400,400)))
        self.img.setImage(np.random.rand(400,400))
        ## lock the aspect ratio
        vb.setAspectLocked(True)
        self.plt = pg.PlotItem()

        self.plt.plot(np.random.normal(size=100), pen=(255,0,0), name="Red curve")

        vb.addItem(self.img)
        #self.plt.setParentItem(self.img)
        #self.plt.setZValue(10)

        #vb.addItem(self.plt)
        #self.img.setParentItem(self.plt)
        #self.img.setZValue(-10)

        ## make plot with a line drawn in
        #self.plt = pg.PlotItem()
        #view.addItem(self.plt)
        ### Create image item
        #self.img = pg.ImageItem(np.zeros((400,400)))
        #self.plt.addItem(self.img)
        #view.addItem(self.plt)
        #self.img.setImage(np.random.rand(400,400))
        

        ## Set initial view bounds
        vb.setRange(QtCore.QRectF(0, 0, 400, 400))
        
        self.img.setLevels([0, 1])

        #Set timer
        timer = pg.QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(30)
        
        #Data queue
        self.q = Queue.Queue()
        
        self.robotpose = pg.ArrowItem(angle=90,pos=(200,200))
        self.robotpose.setParentItem(self.img)

        ## Start Qt event loop unless running in interactive mode or using pyside.
        if __name__ == '__main__':
            import sys
            if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
                QtGui.QApplication.instance().exec_()
              
    def update(self):
        try:
            data, pose = self.q.get(block=False)
            self.img.setImage(data)
            self.robotpose.setRotation(pose[2])
            self.robotpose.setPos(pose[0],pose[1])
        except:
            pass

    def setdata(self, mapdata, robotpose):
        self.q.put( (mapdata,robotpose) )
        pass

gui = LSLAMGUI()
gui.start()

for i in range(1000):
    time.sleep(0.1)
    gui.setdata(np.random.rand(400,400), [i,i,i])

    

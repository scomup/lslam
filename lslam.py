#!/usr/bin/python
# coding: UTF-8

from readbag import * 
from gui import *
from costmap import *
import tf
import time

class LSLAM():
    def __init__(self, raw_data, cost_map, gui):
        self.raw_data = raw_data
        self.cost_map = cost_map
        self.gui = gui
        tmp = tf.transformations.euler_matrix(0.0, 0.0, 0.0)
        tmp[0,3] = 0.0
        self.scan_base = np.matrix(tmp)

    def run(self):
        for loopi in range(len(self.raw_data)):
            time.sleep(0.03)
            scan, odom = self.raw_data[loopi]
            # scan data in robot coordinations
            map_idx_int, map_idx = self.movescanbyodom(scan, odom)
            # get robot pose for gui
            pose = self.getrobotpose(odom)
            for i in range(map_idx_int.shape[0]):
                #Update the cost map
                map_idx_i = map_idx_int[i,:]
                costmap.updateCostMap(map_idx_i,0.1)
                costmap.updateLines(pose,map_idx_i,-0.1)
            # set gui
            gui.setdata(costmap.prob_data, pose, map_idx)

    def movescanbyodom(self, scan, odom):
        scan_size = scan.shape[0]
        scan = scan.transpose()
        tmp = np.zeros((1, scan_size))
        scan = np.vstack([scan, tmp])
        tmp.fill(1)
        scan = np.vstack([scan, tmp])
        scan = np.matrix(scan)
        scan = odom*self.scan_base*scan
        scan = np.array(scan[0:2,:])
        scan = scan.transpose()
        map_idx = costmap.world_map(scan)
        map_idx_int = map_idx.astype(int)
        return map_idx_int, map_idx

    def getrobotpose(self, odom):
        al, be, ga = tf.transformations.euler_from_matrix(odom[0:3,0:3])
        ga = 180.*(ga)/np.pi
        odompose = np.array([ [ odom[0,3], odom[1,3] ] ])
        map_point = costmap.world_map(odompose)
        return [map_point[0,0],map_point[0,1],ga]

bagreader = BagReader('h1.bag', 'scan', 'odom', 0, 800)
costmap = CostMap()        
gui = LSLAMGUI()
gui.start()
lslam = LSLAM(bagreader.data, costmap, gui)
start = time.time()
lslam.run()
elapsed_time = time.time() - start
print ("elapsed_time:{0}".format(elapsed_time)) + "[sec]"

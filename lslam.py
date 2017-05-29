#!/usr/bin/python
# coding: UTF-8

from readbag import * 
from gui import *
from costmap import *
import tf
import time

class LSLAM():
    def __init__(self, raw_data, costmap, gui):
        self.raw_data = raw_data
        self.costmap = costmap
        self.gui = gui
        tmp = tf.transformations.euler_matrix(0.0, 0.0, -0.63)
        tmp[0,3] = 0.15
        self.scan_base = np.matrix(tmp)
        #self.pose
        #self.last_odom


    def run(self):
        for loopi in range(len(self.raw_data)):
            time.sleep(0.03)
            scan, odom = self.raw_data[loopi]
            # scan data in robot coordinations
            map_idx_int, map_idx = self.movescanbyodom(scan, odom)

            #==================================
            #Motion estimation
            #==================================
            #odom_inv = np.linalg.inv(odom)
            #t=pose*odom_inv*last_odom
            pose = self.getrobotpose(odom)
            self.getCompleteHessianDerivs(pose, map_idx, scan)


            #==================================
            #Scan matching
            #==================================
            pass

            #==================================
            #Map Update
            #==================================
            self.mapupdate(pose, map_idx_int)
            #tmp = tf.transformations.euler_matrix(0.0, 0.0, self.a)
            #tmp[0,3] = 0.15

            # Set gui
            gui.setdata(self.costmap.prob_data, pose, map_idx)

    def getCompleteHessianDerivs(self, pose, map_idx, scan):
        size = map_idx.shape[0]
        sinRot = np.sin(pose[2])
        cosRot = np.cos(pose[2])
        H = np.zeros((3,3))
        dTr = np.zeros((3,1))
        for i in range(size):
            transformedPointData = self.costmap.getMapValueWithDerivatives(map_idx[i,:])
            curPoint = scan[i,:]
            funVal = 1.0 - transformedPointData[0]
            dTr[0] += transformedPointData[1] * funVal
            dTr[1] += transformedPointData[2] * funVal
            rotDeriv = ((-sinRot * curPoint[0] - cosRot * curPoint[1]) * transformedPointData[1] + (cosRot * curPoint[0] - sinRot * curPoint[1]) * transformedPointData[2])
            dTr[2] += rotDeriv * funVal
            H[0,0] += transformedPointData[1]*transformedPointData[1] 
            H[1,1] += transformedPointData[2]*transformedPointData[2]
            H[2,2] += rotDeriv*rotDeriv 
            H[0,1] += transformedPointData[1] * transformedPointData[2]
            H[0,2] += transformedPointData[1] * rotDeriv
            H[1,2] += transformedPointData[2] * rotDeriv
        H[1,0] = H[0,1]
        H[2,0] = H[0,2]
        H[2,1] = H[1,2]
        if H[0, 0] != 0.0 and H[1, 1] != 0.0 and H[2, 2] != 0.0:
            H_inv = np.linalg.inv(H)
            searchDir = np.matrix(H_inv) * np.matrix(dTr)
            print searchDir
          
        
    def mapupdate(self, robotpose, map_idx_int):
        for i in range(map_idx_int.shape[0]):
            map_idx_i = map_idx_int[i,:]
            costmap.updateCostMap(map_idx_i,0.1)
            costmap.updateLines(robotpose, map_idx_i,-0.1)

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
        odompose = np.array([ [ odom[0,3], odom[1,3] ] ])
        map_point = costmap.world_map(odompose)
        return [map_point[0,0],map_point[0,1],ga]

#bagreader = BagReader('/home/liu/bag/h1.bag', '/Rulo/laser_scan', '/Rulo/odom', 60, 800)
bagreader = BagReader('/home/liu/bag/h1.bag', '/Rulo/laser_scan', '/Rulo/odom', 80, 800)
costmap = CostMap()        
gui = LSLAMGUI()
gui.start()
lslam = LSLAM(bagreader.data, costmap, gui)
start = time.time()
lslam.run()
elapsed_time = time.time() - start
print ("elapsed_time:{0}".format(elapsed_time)) + "[sec]"

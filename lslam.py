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
        #self.last_odom 


    def run(self):
        for loopi in range(len(self.raw_data)):


            time.sleep(0.03)
            scan, odom = self.raw_data[loopi]
            scan = np.matrix(scan)
            odom = np.matrix(odom)
            try: 
                last_odom = self.last_odom
                self.last_odom = odom
            except AttributeError:
                self.last_odom = odom
                self.pose = self.getrobotpose(odom)
                continue

            #==================================
            #Motion estimation
            #==================================
            last_odom_inv = np.matrix(np.linalg.inv(last_odom))
            odom_delta = last_odom_inv * odom
            pose_matrix = tf.transformations.euler_matrix(0.0, 0.0, self.pose[2])
            pose_matrix[0,3] = (self.pose[0] - self.costmap.original_point[0])*self.costmap.resolution
            pose_matrix[1,3] = (self.pose[1] - self.costmap.original_point[1])*self.costmap.resolution

            new_pose_matrix = pose_matrix * odom_delta
            #new_pose_matrix = pose_matrix
            #t=pose*odom_inv*last_odom
            new_pose = self.getrobotpose(new_pose_matrix)
            # scan data in robot coordinations

            #==================================
            #Scan matching
            #==================================
            map_idx, scan_fix = self.movescanbyodom(scan, new_pose_matrix)
            scan_delta = self.getCompleteHessianDerivs(new_pose, map_idx, scan_fix)
            self.pose = new_pose + scan_delta
            #self.pose = new_pose
            #self.pose[0] = new_pose[0] + scan_delta[0]
            #self.pose[1] = new_pose[1] + scan_delta[1]
            #print scan_delta
            """"
            if np.isnan(scan_delta[0]):
                self.pose[0] = new_pose[0]
            else:
                self.pose[0] = new_pose[0] + scan_delta[0]
            if np.isnan(scan_delta[1]):
                self.pose[1] = new_pose[1]
            else:
                self.pose[1] = new_pose[1] + scan_delta[1]
            if np.isnan(scan_delta[2]):
                self.pose[2] = new_pose[2]
            else:
                self.pose[2] = new_pose[2] + scan_delta[2]
            """
            matching_pose_matrix = tf.transformations.euler_matrix(0.0, 0.0, self.pose[2])
            matching_pose_matrix[0,3] = (self.pose[0] - self.costmap.original_point[0])*self.costmap.resolution
            matching_pose_matrix[1,3] = (self.pose[1] - self.costmap.original_point[1])*self.costmap.resolution
            map_idx, scan_fix = self.movescanbyodom(scan, matching_pose_matrix)


            #==================================
            #Map Update
            #==================================
            self.mapupdate(self.pose, map_idx)
            #tmp = tf.transformations.euler_matrix(0.0, 0.0, self.a)
            #tmp[0,3] = 0.15

            # Set gui          
            gui.setdata(self.costmap.prob_data, self.pose, self.pose, map_idx)

    def getCompleteHessianDerivs(self, pose, map_idx, scan):
        size = map_idx.shape[0]
        sinRot = np.sin(pose[2])
        cosRot = np.cos(pose[2])
        H = np.zeros((3,3))
        dTr = np.zeros((3,1))
        for i in range(size):
            transformedPointData = self.costmap.getMapValueWithDerivatives(map_idx[i,:])
            #if np.isnan(map_idx[i,0]) or np.isnan(map_idx[i,1]):
            #    continue

            curPoint = scan[i,:] / self.costmap.resolution
            #curPoint = self.costmap.world_map(curPoint)
            funVal = 1.0 - transformedPointData[0]
            dTr[0] += transformedPointData[1] * funVal
            dTr[1] += transformedPointData[2] * funVal
            rotDeriv = ((-sinRot * curPoint[0,0] - cosRot * curPoint[0,1]) * transformedPointData[1] + (cosRot * curPoint[0,0] - sinRot * curPoint[0,1]) * transformedPointData[2])
            dTr[2] += rotDeriv * funVal
            H[0,0] += transformedPointData[1]*transformedPointData[1] 
            H[1,1] += transformedPointData[2]*transformedPointData[2]
            H[2,2] += rotDeriv*rotDeriv 
            H[0,1] += transformedPointData[1] * transformedPointData[2]
            H[0,2] += transformedPointData[1] * rotDeriv
            H[1,2] += transformedPointData[2] * rotDeriv
        H[0,0] += 1000 
        H[1,1] += 1000
        H[2,2] += 0
        H[1,0] = H[0,1] 
        H[2,0] = H[0,2]
        H[2,1] = H[1,2]
        if H[0, 0] != 0.0 and H[1, 1] != 0.0 and H[2, 2] != 0.0:
            H_inv = np.linalg.inv(H)
            searchDir = np.matrix(H_inv) * np.matrix(dTr)
            #print searchDir
            return np.array([searchDir[0,0],searchDir[1,0],searchDir[2,0]])
        else:
            return np.zeros( 3 )
          
        
    def mapupdate(self, robotpose, map_idx):
        #if int(map_idx_int[0,0]) > self.costmap.size[0] or int(map_idx_int[0,0]) < 0:
        #    return 
        #if int(map_idx_int[0,1]) > self.costmap.size[1] or int(map_idx_int[0,1]) < 0:
        #    return 

        for i in range(map_idx.shape[0]):
            #print robotpose
            #print map_idx_int
            map_idx_i = map_idx[i,:]
            map_idx_i_ = map_idx_i + 0.5
            map_idx_i_int = map_idx_i_.astype(int)
            costmap.updateCostMap(map_idx_i_int,0.2)
            costmap.updateLines(robotpose, map_idx_i,-0.1)

    def movescanbyodom(self, scan, odom):
        scan_size = scan.shape[0]
        scan_tmp = scan.transpose()
        tmp = np.zeros((1, scan_size))
        scan_tmp = np.vstack([scan_tmp, tmp])
        tmp.fill(1)
        scan_tmp = np.vstack([scan_tmp, tmp])
        scan_tmp = np.matrix(scan_tmp)
        scan_fix = self.scan_base*scan_tmp
        world_idx = odom*scan_fix
        world_idx = np.array(world_idx[0:2,:])
        world_idx = world_idx.transpose()
        map_idx = costmap.world_map(world_idx)

        scan_fix = scan_fix.transpose()
        scan_fix = scan_fix[:,0:2]
        return  map_idx, scan_fix

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



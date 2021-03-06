#!/usr/bin/env python
# coding:utf-8

import numpy as np

class CostMap():
    def __init__(self, size =(400,400), original_point = (100,100), resolution = 0.025):
        self.size = size
        self.map_data = np.zeros(size)
        self.prob_data = np.zeros(size)
        self.prob_data.fill(0.5)
        self.resolution = resolution
        self.original_point = np.array(original_point)
        self.ker = np.array([[1./16.,1./8.,1./16.],[1./8.,1./4.,1./8.],[1./16.,1./8.,1./16.]])

    def world_map(self, world_point):
        map_point = world_point/self.resolution + np.tile(self.original_point,(world_point.shape[0],1))
        return map_point

    def map_world(self, map_point):
        world_point = (map_point - np.tile(self.original_point,(map_point.shape[0],1)) ) * self.resolution
        return world_point

    def updateCostMap(self, map_point, val, use_gaussian):
        old_val = self.map_data[map_point[1], map_point[0]]
        if old_val > 50 or old_val < -50:
            return
        if not use_gaussian:
            new_val = old_val + val
            self.map_data[map_point[1], map_point[0]] = new_val
            odds = np.exp(new_val)
            new_prob = odds/(1+odds)
            self.prob_data[int(map_point[1]), int(map_point[0])] = new_prob
        else:
            self.map_data[map_point[1]-1 : map_point[1]+2, map_point[0]-1 : map_point[0]+2] += self.ker * val
            odds = np.exp(self.map_data[map_point[1]-1 : map_point[1]+2, map_point[0]-1 : map_point[0]+2])
            new_prob = odds/(1+odds)
            self.prob_data[map_point[1]-1 : map_point[1]+2, map_point[0]-1 : map_point[0]+2] = new_prob
        

    def getMapValueWithDerivatives(self, map_point_float):
        factors0 = map_point_float[0] - float(int(map_point_float[0]))
        factors1 = map_point_float[1] - float(int(map_point_float[1]))
        p0 = self.prob_data[int(map_point_float[1]),   int(map_point_float[0])]
        p1 = self.prob_data[int(map_point_float[1]),   int(map_point_float[0]+1)]
        p2 = self.prob_data[int(map_point_float[1]+1), int(map_point_float[0])]
        p3 = self.prob_data[int(map_point_float[1]+1), int(map_point_float[0]+1)]
        dx1 = p0 - p1
        dx2 = p2 - p3
        dy1 = p0 - p2
        dy2 = p1 - p3
        xFacInv = 1.0 - factors0
        yFacInv = 1.0 - factors1
        return [
            ((p0 * xFacInv + p1 * factors0) * (yFacInv)) +
            ((p2 * xFacInv + p3 * factors0) * (factors1)),
            -((dx1 * xFacInv) + (dx2 * factors0)),
            -((dy1 * yFacInv) + (dy2 * factors1)) ]


    def updateLines(self, start, end, val, use_gaussian):
        lines = self.get_line(start, end)
        for p in lines:
            self.updateCostMap(p, val, use_gaussian)

    def get_line(self, start, end):
        # Setup initial conditions
        x1 = start[0] + 0.5
        y1 = start[1] + 0.5
        x2 = end[0] + 0.5
        y2 = end[1] + 0.5
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)
        dx = x2 - x1
        dy = y2 - y1
    
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        if swapped:
            points.reverse() 
        points.pop()           
        return points

if __name__ == "__main__":
    costmap = CostMap()
    costmap.updateCostMap(np.array([240,200]),0.1)
    print costmap.getMapValueWithDerivatives(np.array([240,199]))

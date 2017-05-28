#!/usr/bin/env python
# coding:utf-8

import numpy as np
#import ConfigParser


class CostMap():
    def __init__(self, size =(400,400), original_point = (200,200), resolution = 0.05):
        self.map_data = np.zeros(size)
        self.prob_data = np.zeros(size)
        self.prob_data.fill(0.5)
        self.resolution = resolution
        self.original_point = np.array(original_point)

    def world_map(self, world_point):
        map_point = world_point/self.resolution + np.tile(self.original_point,(world_point.shape[0],1))
        return map_point

    def map_world(self, map_point):
        world_point = (map_point - np.tile(self.original_point,(map_point.shape[0],1)) ) * self.resolution
        return world_point


    #def setCostMap(self, map_point, cost):
    #    self.map_data[int(map_point[0]), int(map_point[1])] = cost

    def updateCostMap(self, map_point, val):
        self.map_data[int(map_point[0]), int(map_point[1])] += val
        new_val = self.map_data[int(map_point[0]), int(map_point[1])]
        odds = np.exp(new_val)
        new_prob = odds/(1+odds)
        self.prob_data[int(map_point[0]), int(map_point[1])] = new_prob

    def getCostMap(self, map_point):
        return self.map_data[map_point[0], map_point[1]]

    def updateLines(self, start, end, val):
        lines = self.get_line(start, end)
        for p in lines:
            self.updateCostMap(p, val)

    def get_line(self, start, end):
        # Setup initial conditions
        x1 = start[0]
        y1 = start[1]
        x2 = end[0]
        y2 = end[1]
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
    print costmap.world_map(np.array([2,0.2]))
    print costmap.map_world(np.array([240,200]))
    costmap.updateCostMap(np.array([240,200]),0.1)
    print costmap.getCostMap(np.array([240,200]))
    costmap.updateLines(np.array([240,210]),np.array([250,240]),-0.1)

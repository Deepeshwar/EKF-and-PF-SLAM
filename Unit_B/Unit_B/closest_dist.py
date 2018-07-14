# -*- coding: utf-8 -*-
"""
Created on Fri Jun  8 11:52:33 2018

@author: Deepeshwar
"""

def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Enter your code here.
    # Make a loop over all cylinders and reference_cylinders.
    # In the loop, if cylinders[i] is closest to reference_cylinders[j],
    # and their distance is below max_radius, then add the
    # tuple (i,j) to cylinder_pairs, i.e., cylinder_pairs.append( (i,j) ).
    
    for i in range(len(cylinders)):
        dist, last_dist, closest_dist = 0.0, 100000, 0.0
        for j in range(len(reference_cylinders)):
            dist = math.sqrt(pow((cylinders[i][0]-reference_cylinders[j][0]),2) + pow((cylinders[i][1]-reference_cylinders[j][1]),2))
            if dist < last_dist:
                closest_dist = dist
                last_dist = dist
        if closest_dist < max_radius:
            cylinder_pairs.append( (i,j) )

    return cylinder_pairs


if __name__ == '__main__':
    
    max_cylinder_distance = 500.0
    
    reference_cylinders = [(1291.0, 1881.0), (482.0, 682.0), (1191.0, 747.0), (1693.0, 1043.0), (383.0, 1458.0), (1805.0, 190.0)]
    world_cylinders = [(41.43341623992836, 1744.3273350978545), (46.15955912002397, 782.2448889103157), (752.9083273871819, 500.43720082612833)]
    
    cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)
    
    print(cylinder_pairs)
    
# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, use this transform to correct the pose.
# 04_d_apply_transform
# Claus Brenner, 14 NOV 2012
import math
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
     compute_scanner_cylinders, write_cylinders
from math import sqrt, atan2

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.
def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    # --->>> Enter your code here.
    # Make a loop over all cylinders and reference_cylinders.
    # In the loop, if cylinders[i] is closest to reference_cylinders[j],
    # and their distance is below max_radius, then add the
    # tuple (i,j) to cylinder_pairs, i.e., cylinder_pairs.append( (i,j) ).
    
    for i in range(len(cylinders)):
        dist, last_dist, closest_dist,r = 0.0, 100000, 0.0, 0
        for j in range(len(reference_cylinders)):
            dist = math.sqrt(pow((cylinders[i][0]-reference_cylinders[j][0]),2) + pow((cylinders[i][1]-reference_cylinders[j][1]),2))
            if dist < last_dist:
                closest_dist = dist
                last_dist = dist
                r = j
        if closest_dist < max_radius:
            cylinder_pairs.append( (i,r) )

    return cylinder_pairs


# Given a point list, return the center of mass.
def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (sx / len(point_list), sy / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.
def estimate_transform(left_list, right_list, fix_scale = False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)
    
    la, c, s, tx, ty = 0.0, 0.0, 0.0, 0.0, 0.0
    l_x, l_y, r_x, r_y, cs, ss, rr, ll = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    #l = (0.0, 0.0)
    #r = (0.0, 0.0)
    #cs, ss, rr, ll = 0.0, 0.0, 0.0, 0.0
    
    for i in range(len(left_list)):
        #l = left_list[i] - lc
        #r = right_list[i] - rc
        l_x = left_list[i][0] - lc[0]
        l_y = left_list[i][1] - lc[1]
        r_x = right_list[i][0] - rc[0]
        r_y = right_list[i][1] - rc[1]
        
        cs += (r_x * l_x) + (r_y * l_y)
        ss += (-r_x * l_y) + (r_y * l_x)
        
        #cs += (r[0] * l[0]) + (r[1] * l[1])
        #ss += (-r[0] * l[1]) + (r[1] * l[0])
        
        rr += (r_x * r_x) + (r_y * r_y)
        ll += (l_x * l_x) + (l_y * l_y)
        
        #rr += (r[0] * r[0]) + (r[1] * r[1])
        #ll += (l[0] * l[0]) + (l[1] * l[1])
    
    if ll == 0.0 and rr == 0.0:
        return 0
    
    else:
        
        if fix_scale == True:
            la = 1.0
        else:
            la = math.sqrt(rr/ll)
        
        if ss == 0.0 and cs == 0.0:
            return 0
        else:
                
            c = cs / (math.sqrt(pow(cs,2)) + pow(ss,2))
            s = ss / (math.sqrt(pow(cs,2)) + pow(ss,2))
            
            tx = rc[0] - la * (c * lc[0] - s * lc[1])
            ty = rc[1] - la * (s * lc[0] + c * lc[1])
            
            
            # --->>> Insert here your code to compute lambda, c, s and tx, ty.
            
            return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

# Correct the pose = (x, y, heading) of the robot using the given
# similarity transform. Note this changes the position as well as
# the heading.
def correct_pose(pose, trafo):
    
    # --->>> This is what you'll have to implement.
    pose[0], pose[1] = apply_transform(trafo, pose)
    alpha = atan2(trafo[2], trafo[1])
    pose[2] += alpha 
    return (pose[0], pose[1], pose[2])  # Replace this by the corrected pose.


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.    
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 400.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = file("apply_transform.txt", "w")
    for i in xrange(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)
        print(world_cylinders)
        print(cylinder_pairs)
        print(reference_cylinders)
        # Estimate a transformation using the cylinder pairs.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale = True)
        #print(trafo)
        # Transform the cylinders using the estimated transform.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]
        
        #print(transformed_world_cylinders)
        # Also apply the trafo to correct the position and heading.
        if trafo:
            pose = correct_pose(pose, trafo)
        #print(pose)
        # Write to file.
        # The pose.
        print >> out_file, "F %f %f %f" % pose
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders, transformed using the estimated trafo.
        write_cylinders(out_file, "W C", transformed_world_cylinders)

    out_file.close()

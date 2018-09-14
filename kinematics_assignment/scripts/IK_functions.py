#! /usr/bin/env python3

import math

"""
    # Rizwan Asif
    # rasif@kth.se
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    
    L0 = 0.07    
    L1 = 0.3
    L2 = 0.35

    #subtracting offset
    x -= L0
    
    #t2 calculation
    t2 = math.acos( (math.pow(x,2) + math.pow(y,2) - math.pow(L1,2) - math.pow(L2,2))/(2*L1*L2) )
    
    #t1 calculation
    num = y*(L2*math.cos(t2)+L1) - x*(L2*math.sin(t2))
    dnum = x*(L2*math.cos(t2)+L1) + y*(L2*math.sin(t2)) 
    t1 = math.atan2(num,dnum)
    
    q = [t1, t2, z]
    
    print("Returning {}".format(q))
    
    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q

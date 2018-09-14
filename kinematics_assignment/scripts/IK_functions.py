#! /usr/bin/env python3

import numpy as np
import math

"""
    # {student full name}
    # {student email}
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    Fill in your IK solution here and return the three joint values in q
    """

    return q


def get_transform(q, a, d, al):
    cal = math.cos(al)
    sal = math.sin(al)

    ct = math.cos(q)
    st = math.sin(q)
    
    return np.matrix([[ ct, -st, 0, a ],[ st*cal, ct*cal, -sal, -sal*d ],[ st*sal, ct*sal, cal, cal*d ],[ 0,0,0,1 ]])


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = np.array(joint_positions) #it must contain 7 elements

    for i in range(100):
    #while(True):

       A7 = get_transform(q[6], 0, 0, 0)
       A6 = get_transform(q[5], 0, 0, -math.pi/2.0)
       A5 = get_transform(q[4], 0, 0.39, math.pi/2.0)
       A4 = get_transform(q[3], 0, 0, 0)
       A3 = get_transform(q[2], 0, 0.4,-math.pi/2.0)
       A2 = get_transform(q[1], 0, 0, -math.pi/2.0)
       A1 = get_transform(q[0], 0, 0, math.pi/2.0)

       T1 = A1
       T2 = np.matrix(A1*A2)
       T3 = np.matrix(A1*A2*A3)
       T4 = np.matrix(A1*A2*A3*A4)
       T5 = np.matrix(A1*A2*A3*A4*A5)
       T6 = np.matrix(A1*A2*A3*A4*A5*A6)
       T7 = np.matrix(A1*A2*A3*A4*A5*A6*A7)


       z0 = np.matrix([[0],[0],[1]])
       z1 = T1[0:3,2]
       z2 = T2[0:3,2]
       z3 = T3[0:3,2]
       z4 = T4[0:3,2]
       z5 = T5[0:3,2]
       z6 = T6[0:3,2]

       p0 = np.matrix([[0],[0],[0]])
       p1 = T1[0:3,3]
       p2 = T2[0:3,3]
       p3 = T3[0:3,3]
       p4 = T4[0:3,3]
       p5 = T5[0:3,3]
       p6 = T6[0:3,3]

       P = np.dot(T7, np.transpose(np.matrix([0,0,0.078,1])))[0:3]
       P[2] += 0.311
       
       R2 = [ [T7[0,0], T7[0,1], T7[0,2]], [T7[1,0], T7[1,1], T7[1,2]], [T7[2,0], T7[2,1], T7[2,2]] ]
       X2 = [ T7[3,0], T7[3,1], T7[3,2]]
       #print(X2)
       #temp = np.array([0,0,0.078])
       #print(temp)
       #X2 = np.dot(X2,temp)
       #print("{}".format(X2))
       #X2[0][2] += 0.311
       
       
       #R error
       rh1 = np.array(R)
       rh2 = np.array(R2)

       ne, se, ae = rh1[:,0], rh1[:,1], rh1[:,2]
       nd, sd, ad = rh2[:,0], rh2[:,1], rh2[:,2]

       ne = np.transpose(ne)
       se = np.transpose(se)
       ae = np.transpose(ae)
       nd = np.transpose(nd)
       sd = np.transpose(sd)
       ad = np.transpose(ad)
       

       Re = 0.5 * (np.cross(ne,nd) + np.cross(se,sd) + np.cross(ae,ad))
       
       #X error
       Xe = X2 - point
       print("shape Xe {}".format(np.shape(Xe)))
       print("shape Re {}".format(np.shape(Re)))
       Xe = np.transpose(np.matrix(Xe))
       Re = np.transpose(np.matrix(Re))
       print(Xe)
       print(Re)

       error = np.vstack((Xe,Re))
       #error = np.transpose(error)
       #error = np.array(error)[0]
       #print(error)

       r00 = np.matrix(np.transpose(np.cross(np.transpose(z0), np.transpose(P-p0))))
       r01 = np.matrix(np.transpose(np.cross(np.transpose(z1), np.transpose(P-p1))))
       r02 = np.matrix(np.transpose(np.cross(np.transpose(z2), np.transpose(P-p2))))
       r03 = np.matrix(np.transpose(np.cross(np.transpose(z3), np.transpose(P-p3))))
       r04 = np.matrix(np.transpose(np.cross(np.transpose(z4), np.transpose(P-p4))))
       r05 = np.matrix(np.transpose(np.cross(np.transpose(z5), np.transpose(P-p5))))
       r06 = np.matrix(np.transpose(np.cross(np.transpose(z6), np.transpose(P-p6))))

       r0 = np.vstack((r00, z0))
       r1 = np.vstack((r01, z1))
       r2 = np.vstack((r02, z2))
       r3 = np.vstack((r03, z3))
       r4 = np.vstack((r04, z4))
       r5 = np.vstack((r05, z5))
       r6 = np.vstack((r06, z6))

       J = np.hstack((r0,r1,r2,r3,r4,r5,r6))
       #J = np.transpose(J)
       #J = np.array(J)[0]
       #error = np.array(error)[0]
       print("J shape {}".format(np.shape(J)))
       print("e shape {}".format(np.shape(error)))
       print("Jacob {}".format(J))
       print("Error {}".format(error))

       Qeps = np.dot(np.linalg.pinv(J), error)
       #Qyyeps = np.linalg.pinv(J)*error
       Qeps = np.transpose(Qeps)

       print("Max Error {}".format(np.max(np.abs(error))))
       #if np.max(np.abs(error)) < 0.01:
       if np.linalg.norm(Qeps) < 0.01:
           break  
       
       print("{} - {}".format(q,Qeps))
       q = q - Qeps
       q = np.array(q)[0]
       print(q)

    return q

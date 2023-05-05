import trianglesolver
from scipy.optimize import fsolve
import math

# points as tuples, angles as radians
def calc_ang_pos(pa,pb,pc,a1,a2):
    a3 = 2*math.pi - a1 - a2
    # a1 is btw pb pa, a2 btw pc pb, a3 btw pa pc
    # system of angles 

    big_triangle = trianglesolver.solve(a=d(pb,pc),b=d(pa,pc),c=d(pa,pb))

    # print(a3)
    # print(big_triangle)

    [a,b,c,A,B,C] = big_triangle
    
    def func(x):
        return [math.pi - a1 - x[0] - x[1],
                math.pi - a2 - x[2] - x[3],
                math.pi - a3 - x[4] - x[5],
                A - x[0] - x[5],# A - ab - ac
                B - x[1] - x[2],
                C - x[3] - x[4], ]
    # x = ab,ba,bc,cb,ca,ac
    root = fsolve(func,[A/2,B/2,B/2,C/2,C/2,A/2])

    [ab,ba,bc,cb,ca,ac]=root
    print(root)

    ra = trianglesolver.solve(A=ba,C=ab,b=d(pa,pb))[0]
    # rb = trianglesolver.solve(A=cb,C=bc,b=d(pc,pb))[0]
    # rc = trianglesolver.solve(A=ac,C=ca,b=d(pa,pc))[0]

    ang_ra = ang(pa,pb) - ab
    # print(ang_ra)
    # print(ab)
    x = ra*math.cos(ang_ra)+pa[0]
    y = ra*math.sin(ang_ra)+pa[1]
    return ([x,y])

def d2(A,B):
    return (A[0]-B[0])**2 + (A[1]-B[1])**2

def d(A,B):
    return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)

def ang(A,B):
    return math.atan2(B[1]-A[1],B[0]-A[0])

print(calc_ang_pos([0,4],[4,4],[2,0],math.pi*2/3,math.pi*2/3))

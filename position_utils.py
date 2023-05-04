import trianglesolver
from scipy.optimize import fsolve

# points as tuples, angles as radians
def calc_ang_pos(pa,pb,pc,a1,a2):
    a3 = 2*math.pi - a1 - a2
    # a1 is btw pb pa, a2 btw pc pb, a3 btw pa pc
    # system of angles 

    big_triangle = trianglesolver.solve(a=d(pb,pc),b=d(pa,pc),c=d(pa,pb))
    
    def func(x):
        return [math.pi - a1 - x[0] - x[1],
                math.pi - a2 - x[2] - x[3],
                math.pi - a3 - x[4] - x[5],
                big_triangle[3] - x[0] - x[5],# A - ab - ac
                big_triangle[4] - x[1] - x[2],
                big_triangle[5] - x[3] - x[4], ]
    # x = ab,ba,bc,cb,ca,ac
    root = fsolve(func,[1,1,1,1,1,1])

    [ab,ba,bc,cb,ca,ac]=root

    ra = trianglesolver.solve(A=ba,C=ab,b=d(pa,pb))[0]
    # rb = trianglesolver.solve(A=cb,C=bc,b=d(pc,pb))[0]
    # rc = trianglesolver.solve(A=ac,C=ca,b=d(pa,pc))[0]

    ang_ra = ang(pa,pb) - ab
    x = ra*math.cos(ang_ra)
    y = ra*math.sin(ang_ra)
    return ()

def d2(A,B):
    return (A[0]-B[0])**2 + (A[1]-B[1])**2

def d(A,B):
    return (A[0]-B[0])**2 + (A[1]-B[1])**2

def ang(A,B):
    return math.atan2(B[0]-A[0],B[1]-A[1])
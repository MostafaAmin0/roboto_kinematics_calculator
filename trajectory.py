from sympy import *
import numpy as np
import matplotlib.pyplot as plt

t = symbols("t")
c = symbols('c0:4')
t0 = 0
tf = np.pi/8
qlist = [[-0.356,1.47655],[-0.3, 1.5]]
jacobian=np.array([[-1.1,-1.8],
          [-2.74,0.87],
          [0,0],
          [0,0],
          [0,0],
          [1,1]
         
         ])

jacobianf=np.array([[-1.27,-1.86],
          [-2.63,0.71],
          [0,0],
          [0,0],
          [0,0],
          [1,1]
         ])


q=[]
dq=[]


def inv_jacobian(jacobian):
    jacobianTrans=np.transpose(jacobian)
    jacobianTrans+=0.
    temp = np.dot(jacobianTrans,jacobian)
    temp=np.transpose(temp)
    temp=np.around(temp,1)

    jacobianInv=np.dot(temp,jacobianTrans)
    jacobianInv=np.around(jacobianInv,1)
    

    return jacobianInv


def get_joint_velocities(inv_jacobian, dx,dy):
    
    meu = np.array([[dx],[dy],[0],[0],[0],[0]])
    dq = np.dot(inv_jacobian,meu)

    dq = dq*np.pi/180
    
    return dq.flatten().tolist()


def cubic_trajectory_planning(q,dq,t0,tf):
    
    trajEq = c[0] + c[1]*t + c[2] * t**2 + c[3] * t**3
    diffTrajEq = diff(trajEq,t)
    jointTrajectoryEqs = []
    
    for i in range(len(q)):
        
        jointsEq = []

        jointsEq.append(trajEq.subs([(t,t0)])-q[0][i])
        jointsEq.append(diffTrajEq.subs([(t,t0)])-dq[0][i])
        jointsEq.append(trajEq.subs([(t,tf)])-q[1][i])
        jointsEq.append(diffTrajEq.subs([(t,tf)])-dq[1][i])

        # Solve equations for q
        solution = solve(jointsEq, [c[0],c[1],c[2],c[3]])
        
        jointTrajectory = trajEq.subs([(c[0],solution[c[0]]), (c[1],solution[c[1]]), (c[2],solution[c[2]]), (c[3],solution[c[3]])])

        jointTrajectoryEqs.append(jointTrajectory)
        print("solution")
        pprint(solution)
    return jointTrajectoryEqs
    
#     return joints_trajectory

def trajectoryJoints(time=0, xEq=2 + (1/2)*cos(t) , yEq=1 + (1/2)*sin(t),jac=0,qlist=0): ## remove jac
    dxEq = diff(xEq,t)
    dyEq = diff(yEq,t)
    xt=xEq.subs([(t,time)])
    yt=yEq.subs([(t,time)])
    dxt=dxEq.subs([(t,time)])
    dyt=dyEq.subs([(t,time)])
    
    ## call inverse function
    # [theta1 inital , theta2 initial] ,[ theta1 final , theta 2 final] >> from inveser kinematics
    #   q = [[-0.356,1.47655],[-0.3, 1.5]]
    q.append(qlist)
    
    #call function for DH param 
        ##TODO
    
    # call jacobian 
#     jacobian=np.array([[-1.1,-1.8],
#           [-2.74,0.87],
#           [0,0],
#           [0,0],
#           [0,0],
#           [1,1]
         
#          ])

# jacobianf=np.array([[-1.27,-1.86],
#           [-2.63,0.71],
#           [0,0],
#           [0,0],
#           [0,0],
#           [1,1]
#          ])

    
    
    # call inverse jacobian 
    
#     invJ = inv_jacobian(jacobian)
    invJ = inv_jacobian(jac)
    dq.append(get_joint_velocities(invJ, dxt,dyt))

trajectoryJoints(time=t0,jac=jacobian,qlist=qlist[0])
trajectoryJoints(time=tf,jac=jacobianf,qlist=qlist[1])

print(np.matrix(q))
print(np.matrix(dq))
jointsEquations = cubic_trajectory_planning(q,dq,t0,tf)
print(jointsEquations)


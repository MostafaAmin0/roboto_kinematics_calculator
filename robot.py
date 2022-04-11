import numpy as np
from sympy import *
import re
import matplotlib.pyplot as plt
import copy
#-------------------------------------------------------
# global variables
# dof =0
joints=''
variables = ["a ", "Alpha ","d ","Theta "]
A = symbols('A0:7')
a = symbols('a0:7')
alpha = symbols('alpha0:7')
d = symbols('d0:7')
theta = symbols('theta0:7')
x, y, z = symbols("x y z")
roll, pitch, yaw = symbols("roll pitch yaw")
t = symbols("t")
c = symbols('c0:4')
t0 = 0
tf = np.pi/8
# q=[]
# dq=[]

#RPP robot
# list_A =[ [ [0,-1,0,0],[1,0,0,0],[0,0,1,1],[0,0,0,1]] ,
#           [ [1,0,0,0],[0,0,1,0],[0,-1,0,2],[0,0,0,1]] ,
#           [ [1,0,0,0],[0,1,0,0],[0,0,1,4],[0,0,0,1]] ]


# list_TF=[
#          [[0,-1,0,0],[1,0,0,0],[0,0,1,1],[0,0,0,1]],
#          [[0,0,-1,0],[1,0,0,0],[0,-1,0,3],[0,0,0,1]],
#          [[0,0,-1,-4],[1,0,0,0],[0,-1,0,3],[0,0,0,1]] ] 



#--------------------------------------------------------
#set the manipulator configurations

#DOF & type of joints
def setJoints ():
    joints=input('please enter joints type: ')
    while(not re.match("^[r|p|R|P]*$", joints)):
        print ("Error! Only letters r or p allowed!")
        joints=input('please enter joints type: ')
    joints=joints.lower()
    dof=len(joints)
    return joints ,dof

    

# DH Parameter
def setDHParameter(dh_matrix):
    dhInput=[]
    for i in range(len(dh_matrix)):          # A for loop for row entries
        dhRow=[]
        for j in range(4):      # A for loop for column entries
            inputData=dh_matrix[i][j]
            inputData =float ( inputData.toPlainText())
            if j==1 or j==3 :
                inputData=(inputData/180.0)*np.pi
            dhRow.append(inputData)
        dhInput.append(dhRow)
    return dhInput

# DH Parameter
def setDHInverse(dh_matrix):
    dhInput=[]
    for i in range(len(dh_matrix)):          # A for loop for row entries
        dhRow=[]
        for j in range(4):      # A for loop for column entries
            inputData=dh_matrix[i][j]
            if type(inputData)!=int:
                inputData =float ( inputData.toPlainText())
            dhRow.append(inputData)
        dhInput.append(dhRow)
    return dhInput

#------------------------------------------------------
#helper functions to return the A matrices
# A1 is 2d matrix 
def An_matrix(a,alpha,d,theta):

    An=[[np.cos(theta), -1*np.sin(theta)*np.cos(alpha),np.sin(theta)*np.sin(alpha),a*np.cos(theta)],
        [np.sin(theta),np.cos(theta)*np.cos(alpha),-1*np.cos(theta)*np.sin(alpha),a*np.sin(theta)],
        [0,np.sin(alpha),np.cos(alpha),d],
        [0,0,0,1]
       ]
    return An
    
# A1----An
def list_A_matrix(DH):
#     if type(DH) != np.array :
    DH=np.array(DH, dtype=float)
    list_A = []
    for i in range(len(DH)):
        mat=np.around(An_matrix(DH[i][0],DH[i][1],DH[i][2],DH[i][3]),1)
        mat+=0.
        list_A.append(mat.tolist())
        
    return list_A

            
#------------------------------------------------------
#helper function to return the Transformatiom matrices

# T1----Tn
def list_TF_matrix(list_A):
    
    list_TF=[]
    tf1 = list_A[0]
    list_TF.append(tf1)
    for i in range (0,len(list_A)-1):
        tf=np.dot( list_TF[i] , list_A[i+1] )
        list_TF.append(tf.tolist())
    return list_TF

#-----------------------------------------------------
#driver function to implement the forward kienmatics

def fK (DH):
    list_A = list_A_matrix(DH)
    list_TF = list_TF_matrix(list_A)
    tf_length=len(list_TF)-1  
    
    xVal = list_TF[tf_length][0][3]
    yVal = list_TF[tf_length][1][3]
    zVal = list_TF[tf_length][2][3]

    # roll >> phi >> about x
    # pitch >> theta >> about y
    # yaw >> psi >> about z

    if list_TF[tf_length][0][0] == 0:
        rollVal = np.pi/2
    else:
        rollVal = atan(list_TF[tf_length][1][0]/list_TF[tf_length][0][0])

    if sqrt((1-list_TF[tf_length][2][0]**2)) == 0:
        pitchVal = np.pi/2

    else:
        pitchVal = atan(-list_TF[tf_length][2][0]/sqrt((1-list_TF[tf_length][2][0]**2)))


    if list_TF[tf_length][2][2] == 0 :
        yawVal = np.pi/2
    else:
        yawVal = atan(list_TF[tf_length][2][1]/list_TF[tf_length][2][2])
    
    # return list_TF,x,y,z
    return list_TF , xVal, yVal, zVal, rollVal, pitchVal, yawVal

#-----------------------------------------------------
#driver function to implement jacobian matrix

def getJacobian(joints,list_TF):
    dof=len(list_TF)
    jacobian_matrix = []
    #get O0--->On
    #get z0-->Zn-1
    list_O = []
    list_Z= []
    
    #for O0
    list_O.append( [0,0,0])
    # for Z0
    list_Z.append( [0,0,1])

    for i in range(0,dof):
        tf=list_TF[i]
        o = [ tf[0][3] ,tf[1][3] ,tf[2][3]]
        list_O.append(o)
        if (i != dof-1):
            z = [ tf[0][2] ,tf[1][2] ,tf[2][2]]
            list_Z.append(z)

#     print(np.transpose(list_O))
#     print(np.transpose(list_Z))
    
    #Jv
    list_jv=[]
    for i in range (1,dof+1):
        if joints[i-1] =='R' or joints[i-1] =='r' :
            On =list_O[dof]
            Oi_1 =list_O[i-1]
            O_subtraction = np.subtract(On,Oi_1)
            z_i=list_Z[i-1]
            jv=np.cross(z_i,O_subtraction)
            list_jv.append(jv)
        else :
            z_i=list_Z[i-1]
            jv=[ z_i[0],z_i[1],z_i[2]]
            list_jv.append(jv)
    list_jv=np.transpose(list_jv)
#     print(list_jv)
    
    #jw
    list_jw=[]
    for i in range (1,dof+1):
        if joints[i-1] =='R' or joints[i-1] =='r':
            z_i=list_Z[i-1]
            jw=[z_i[0],z_i[1],z_i[2]]
            list_jw.append(jw)
        else :
            jw=[0,0,0]
            list_jw.append(jw)
    list_jw=np.transpose(list_jw)
#     print(list_jw)
    
    jacobian_matrix=np.concatenate((list_jv,list_jw),axis=0)
    
    return jacobian_matrix
        


#-----------------------------------------------------
#driver functions to implement the inverse kienmatics
def get_A_Symbolic (i):
    
    AnSymb = Matrix([
        [cos(theta[i]), -sin(theta[i])*cos(alpha[i]), sin(theta[i])*sin(alpha[i]), a[i]*cos(theta[i])],
        [sin(theta[i]), cos(theta[i])*cos(alpha[i]), -cos(theta[i])*sin(alpha[i]), a[i]*sin(theta[i]) ],
        [0,sin(alpha[i]), cos(alpha[i]), d[i]],
        [0, 0, 0, 1],
    ])
    
    return AnSymb

def get_T_Symbolic (dof, dhSymb):
    
    T_Matrix = get_A_Symbolic(1).subs([(a[1], dhSymb[0][0]), (alpha[1],dhSymb[0][1]),(d[1],dhSymb[0][2]), (theta[1], dhSymb[0][3])])
    
    for i in range (2, dof+1):
        A = get_A_Symbolic(i).subs([(a[i],dhSymb[i-1][0]), (alpha[i],dhSymb[i-1][1]),(d[i],dhSymb[i-1][2]), (theta[i], dhSymb[i-1][3])])
        T_Matrix = T_Matrix * A
    
    return T_Matrix

def get_q_list (joints):
    qList = []
    for i in range(len(joints)):
        if joints[i] == 'r' or joints[i] == 'R':
            qList.append(theta[i+1])
        else:
            qList.append(d[i+1])
            
    return qList

def get_dh_symbolic (DH, joints):
#     dhSymb = DH.copy().tolist()
    dhSymb = copy.deepcopy(DH)
    if type(dhSymb) != list:
        dhSymb=dhSymb.tolist()
        
    for i in range(len(joints)):
        dhSymb[i][1] = (float(dhSymb[i][1])/180.0)*pi
        dhSymb[i][3] = (float(dhSymb[i][3])/180.0)*pi

        if joints[i] == 'r' or joints[i] == 'R':
            dhSymb[i][3] = theta[i+1]
        else:
            dhSymb[i][2] = d[i+1]
            
    return dhSymb

def ik(DH,joints,x=0, y=0, z=0, roll=None, pitch=None, yaw=None):
    # Get the dh in symbolic form based on joints types
    dhSymb = get_dh_symbolic(DH, joints)
    print("Symbolic DH Matrix")
    pprint(dhSymb)
    print("--------------------------------\n\n")
    
    # Get the HTM in symbolic form and substitute with the constant dh values
    dof=len(joints)
    TSymb = get_T_Symbolic(dof,dhSymb)
    print("Symbolic HTM")
    pprint(TSymb)
    print("--------------------------------\n\n")
    
    # Get q variables list
    q = get_q_list(joints)
    
    print("joint variables")
    pprint(q)
    print("--------------------------------\n\n")
    
    equations = []
    eqList = []
    
    # roll >> phi >> about x
    rollEq = atan(TSymb[1,0]/TSymb[0,0])
    # pitch >> theta >> about y
    pitchEq = atan(-TSymb[2,0]/sqrt((1-TSymb[2,0]**2)))
    # yaw >> psi >> about z
    yawEq = atan(TSymb[2,1]/TSymb[2,2])
    
    
    # prevent AccumulationBounds problem
    if TSymb[0,0] == 0:
        rollEq = np.pi/2

    if sqrt((1-TSymb[2,0]**2)) == 0:
        pitchEq = np.pi/2

    if TSymb[2,2] == 0 :
        yawEq = np.pi/2
    
    
    # Define equations, rearranged so expressions equal 0

    # eq1 = T[0, 3] - x
    eqList.append(TSymb[0, 3] - x)
    # eq2 = T[1, 3] - y
    eqList.append(TSymb[1, 3] - y)
    # eq3 = T[2,3] - z
    eqList.append(TSymb[2,3] - z)
    
    if(roll):
        # eq4 = rollEq - roll
        eqList.append(rollEq - roll)
    if(pitch):
        # eq5 = pitchEq - pitch
        eqList.append(pitchEq - pitch)
    if(yaw):
        # eq6 = yawEq - yaw
        eqList.append(yawEq - yaw)
   
    # remove equations that have no symbols
    for eq in eqList:
        if(type(eq) != float and eq != 0):
            equations.append(eq)

#     print("equations")
#     pprint(equations)
#     print("--------------------------------\n\n")
    
    
    # Solve equations for q
    solution = solve(equations, q, dict=True)
    print("Inverse Kinematics Solution")
    return solution

#------------------------------------------------------

#driver functions to implement the path planning 

def inv_jacobian(jacobian):
    jacobianTrans=np.transpose(jacobian)
    jacobianTrans+=0.
    temp = np.dot(jacobianTrans,jacobian)
    temp=np.inv(temp)
    temp=np.around(temp,1)

    jacobianInv=np.dot(temp,jacobianTrans)
    jacobianInv=np.around(jacobianInv,1)
    

    return jacobianInv


def get_joint_velocities(inv_jacobian, dx,dy):
    
    meu = np.array([[dx],[dy],[0],[0],[0],[0]])
    dq = np.dot(inv_jacobian,meu)

    dq = dq*np.pi/180
    
    return dq.flatten().tolist()

    
def get_dh_modified(DH,joints,qList):
    dhMod = copy.deepcopy(DH)
    

    for i in range(len(joints)):
        d1=dhMod[i][1]
        print(type(d1),d1)
        d2=dhMod[i][3]
        print(type(d2),d2)
        dhMod[i][1] = (float(d1)/180.0)*np.pi
        dhMod[i][3] = (float(d2)/180.0)*np.pi

        if joints[i] == 'r' or joints[i] == 'R':
            dhMod[i][3] = qList[i]
        else:
            dhMod[i][2] = qList[i]
            
    return dhMod


def trajectoryJoints(DH,joints,time=0, xEq=2 + (1/2)*cos(t) , yEq=1 + (1/2)*sin(t)):
    
    dxEq = diff(xEq,t)
    dyEq = diff(yEq,t)
    
    xt=xEq.subs([(t,time)])
    yt=yEq.subs([(t,time)])
    
    dxt=dxEq.subs([(t,time)])
    dyt=dyEq.subs([(t,time)])
    print('DH bla 1',DH)
    ## call inverse function
    ikSolution = ik(DH,joints,xt,yt)
    
    print(ikSolution)
    qSymbols = get_q_list(joints)
    qList = [ ikSolution[0][qSymbols[0]], ikSolution[0][qSymbols[1]] ]
    # [theta1 inital , theta2 initial] ,[ theta1 final , theta 2 final] >> from inveser kinematics
    #   q = [[-0.356,1.47655],[-0.3, 1.5]]
#     q.append(qList)
    
    
    print('DH bla 2',DH)
    #call function for modified DH param 
    dhMod = get_dh_modified(DH, joints, qList)
    print(dhMod)
        
    
    # call jacobian 
    list_A = list_A_matrix(dhMod)
    # print (list_A)
    list_TF = list_TF_matrix(list_A)
    # print (list_TF)
    jacobian_matrix = getJacobian(joints,list_TF)


    
    # call inverse jacobian 
    invJ = inv_jacobian(jacobian_matrix)
#     dq.append(get_joint_velocities(invJ, dxt,dyt))
    dqList = get_joint_velocities(invJ, dxt,dyt)

    return qList, dqList

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
#         print("solution")
#         pprint(solution)
    return jointTrajectoryEqs

def plot_trajectory(jointsEquations, initalTime = 0, finalTime = 1, steps = 100):

    n = len(jointsEquations) # dof

    timesteps = np.linspace(initalTime, finalTime, num = steps)

    q = np.zeros((n, steps))
    dq = np.zeros((n, steps))
    ddq = np.zeros((n, steps))

    for i in range (n):

        eq =  jointsEquations[i]

        for j in range(len(timesteps)):

            timestep = timesteps[j]

            q[i, j] = eq.subs([(t, timestep)])
            dq[i, j] = diff(eq).subs([(t, timestep)])
            ddq[i, j] = diff(diff(eq)).subs([(t, timestep)])

        jointName = f"Joint {i + 1}"

        fig, axis = plt.subplots(3)
        fig = plt.gcf()
        fig.set_size_inches(16, 12)
        fig.suptitle(jointName)

        # Joint Position Plot
        axis[0].set_title("Position")
        axis[0].set(xlabel = "Time", ylabel = "Position")
        axis[0].plot(timesteps, q[i])


        axis[1].set_title("Velocity")
        axis[1].set(xlabel = "Time", ylabel = "Velocity")
        axis[1].plot(timesteps, dq[i])

        # Joint Acceleration Plot
        axis[2].set_title("Acceleration")
        axis[2].set(xlabel = "Time", ylabel = "Acceleration")
        axis[2].plot(timesteps, ddq[i])


        fig.tight_layout()
#         plt.show()

        fig.savefig(f"{jointName}.png", dpi=100, facecolor=('white'))

def get_trajectory (DH,joints,initialTime , finalTime):

    q=[]
    dq=[]
    #input t0, tf
    qList, dqList = trajectoryJoints(DH,joints,time=initialTime)
    q.append(qList)
    dq.append(dqList)

    qList, dqList = trajectoryJoints(DH,joints,time=finalTime)
    q.append(qList)
    dq.append(dqList)

#     print(q)
#     print(dq)

    jointsEquations = cubic_trajectory_planning(q,dq,initialTime,finalTime)
    plot_trajectory(jointsEquations, initalTime = initialTime, finalTime = finalTime)
    return jointsEquations

#-------------------------------------------------------
# start of main code
# call your functions here and take input from user 

if __name__ == "__main__":
    joints,dof=setJoints()
    choice = input('type f for forward :')
    dhInput=[]
    if choice == 'f' :
        #take input matrix 
        for i in range(dof):          # A for loop for row entries
            print('Variable for joint number ' + str(i+1))
            for j in range(4):      # A for loop for column entries
                print("please enter "+ variables[j])

                inputData = float(input())
                if variables[j] == "Alpha " or variables[j] == "Theta ":
                    inputData=(inputData/180.0)*np.pi

                dhInput.append(inputData)            
    else:
        #take input matrix 
        for i in range(dof):          # A for loop for row entries
            print('Variable for joint number ' + str(i+1))
            for j in range(4):      # A for loop for column entries
                if (joints[i] == 'r' and j==3 ) or (joints[i] == 'p' and j==2 ):
                    dhInput.append(0)
                    continue

                print("please enter "+ variables[j])

                inputData = float(input())
                dhInput.append(inputData)

    DH=np.array(dhInput)
    DH=DH.reshape(dof,4)
    # choice = input('type f for forward :')
    if choice =='f':
        list_TF,i,j,k, rollVal, pitchVal, yawVal=fK(DH)
        print("x y z roll pitch yaw",i,j,k, rollVal, pitchVal, yawVal)
        jacobian_matrix = getJacobian(joints,list_TF)
        print (jacobian_matrix)


    elif choice == 'i':
        # output :{θ₁: 0.529243889346318, θ₂: 0.776654323826081}
    #     ik(x=2.25, y=2.94, z=0, roll=1.3058982131724, pitch=0, yaw=0) rr


        print('please enter end effector pose : ')
        ikx = float(input('x = '))
        iky = float(input('y = '))
        ikz = float(input('z = '))
        rpy = input('do you want to enter RPY? y/n ')


        if rpy == 'y':
            ikRoll = float(input('roll = '))
            ikPitch = float(input('pitch = '))
            ikYaw = float(input('yaw = '))
            ik(DH,joints, x=ikx, y=iky, z=ikz, roll=ikRoll, pitch=ikPitch, yaw=ikYaw) # rpp

        else:
            ik(DH,joints, x=ikx, y=iky, z=ikz) # rpp


    else:
        #input t0, tf
        
#         jointsEquations = cubic_trajectory_planning(q,dq,t0,tf)
#         print(jointsEquations)
        jointsEquations=get_trajectory(DH,joints,initialTime = t0, finalTime = tf)
        print(jointsEquations)
    # dummy data to test inverse kinmatics
    # joints ='RR'
    # DH = [
    #     [2,0,0, 0.529],
    #     [2,0,0, 0.776],
    # ]




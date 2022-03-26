import numpy as np
import re

#-------------------------------------------------------
# global variables
dof =0
joints=''
variables = ["a ", "Alpha ","d ","Theta "]

#RPP robot
# list_A =[ [ [0,-1,0,0],[1,0,0,0],[0,0,1,1],[0,0,0,1]] ,
#           [ [1,0,0,0],[0,0,1,0],[0,-1,0,2],[0,0,0,1]] ,
#           [ [1,0,0,0],[0,1,0,0],[0,0,1,4],[0,0,0,1]] ]

list_A = []



# list_TF=[
#          [[0,-1,0,0],[1,0,0,0],[0,0,1,1],[0,0,0,1]],
#          [[0,0,-1,0],[1,0,0,0],[0,-1,0,3],[0,0,0,1]],
#          [[0,0,-1,-4],[1,0,0,0],[0,-1,0,3],[0,0,0,1]] ] 
list_TF=[]

jacobian_matrix=[]

#--------------------------------------------------------
#set the manipulator configurations

#DOF & type of joints
def setJoints ():
    pass

# DH Parameter
def setDHParameter():
    pass

#------------------------------------------------------
#helper functions to return the A matrices
# A1 is 2d matrix 
def An_matrix(a,alpha,d,theta):
    alpha=(float(alpha)/180.0)*np.pi
    theta=(float(theta)/180.0)*np.pi

    An=[[np.cos(theta), -1*np.sin(theta)*np.cos(alpha),np.sin(theta)*np.sin(alpha),a*np.cos(theta)],
        [np.sin(theta),np.cos(theta)*np.cos(alpha),-1*np.cos(theta)*np.sin(alpha),a*np.sin(theta)],
        [0,np.sin(alpha),np.cos(alpha),d],
        [0,0,0,1]
       ]
    return An
    
# A1----An
def list_A_matrix():
    for i in range(dof):
        mat=np.around(An_matrix(DH[i][0],DH[i][1],DH[i][2],DH[i][3]),1)
        mat+=0.
        list_A.append(mat.tolist())
    print(list_A)

            
#------------------------------------------------------
#helper function to return the Transformatiom matrices

# T1----Tn
def list_TF_matrix():
    
    tf1 = list_A[0]
    list_TF.append(tf1)
    for i in range (0,dof-1):
        tf=np.dot( list_TF[i] , list_A[i+1] )
        list_TF.append(tf.tolist())
    print(list_TF)

#-----------------------------------------------------
#driver function to implement the forward kienmatics

def fK ():
    pass

#-----------------------------------------------------
#driver function to implement jacobian matrix

def getJacobian():
    
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
    print(jacobian_matrix)
        


#-----------------------------------------------------
#driver function to implement the inverse kienmatics
def iK():
    pass

#------------------------------------------------------
#driver function to implement the path planning 

def path_planning():
    pass

#-------------------------------------------------------
# start of main code
# call your functions here and take input from user 

joints=input('please enter joints type: ')
while(not re.match("^[r|p|R|P]*$", joints)):
    print ("Error! Only letters r or p allowed!")
    joints=input('please enter joints type: ')

dof=len(joints)
a=[]
#take input matrix 
for i in range(dof):          # A for loop for row entries
    print('Variable for joint number ' + str(i+1))
    for j in range(4):      # A for loop for column entries
        print("please enter "+ variables[j])
        a.append(int(input()))
DH=np.array(a)
DH=DH.reshape(dof,4)
list_A_matrix()
list_TF_matrix()

getJacobian()
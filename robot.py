import numpy as np

#-------------------------------------------------------
# global variables
dof =3
joints='RPP'
#RPP robot
list_A =[ [ [0,-1,0,0],[1,0,0,0],[0,0,1,1],[0,0,0,1]] ,
          [ [1,0,0,0],[0,0,1,0],[0,-1,0,2],[0,0,0,1]] ,
          [ [1,0,0,0],[0,1,0,0],[0,0,1,4],[0,0,0,1]] ]

list_TF=[
         [[0,-1,0,0],[1,0,0,0],[0,0,1,1],[0,0,0,1]],
         [[0,0,-1,0],[1,0,0,0],[0,-1,0,3],[0,0,0,1]],
         [[0,0,-1,-4],[1,0,0,0],[0,-1,0,3],[0,0,0,1]] ] 

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
def matrix_A():
    pass

# A1----An
def list_A_matrix():
    pass

#------------------------------------------------------
#helper functions to return the Transformatiom matrices
#T0
def transformationMatrix ():
    pass

# T0----Tn
def list_TF_matrix():
    pass

#-----------------------------------------------------
#driver function to implement the forward kienmatics

def fK ():
    pass

#-----------------------------------------------------
#driver function to implement jacobian matrix

def getJacobian():
    global list_TF
    global dof
    global joints
    
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
        if joints[i-1] =='R':
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
    print(list_jv)
    
    #jw
    list_jw=[]
    for i in range (1,dof+1):
        if joints[i-1] =='R':
            z_i=list_Z[i-1]
            jw=[z_i[0],z_i[1],z_i[2]]
            list_jw.append(jw)
        else :
            jw=[0,0,0]
            list_jw.append(jw)
    list_jw=np.transpose(list_jw)
    print(list_jw)
    
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

print('alo')

getJacobian()
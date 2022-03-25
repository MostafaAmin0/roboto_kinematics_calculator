import numpy as np

#-------------------------------------------------------
# global variables

#  A1=[[1,1,1,1],
#       [1,1,1,1],
#       [1,1,1,1],
#       [0,0,0,1],
#     ]
# listA = [ A1 , A2 , ...]

# and so on for TF

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
    pass

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


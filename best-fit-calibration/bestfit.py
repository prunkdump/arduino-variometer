##########################################################################
# bestfit -- A best-fit library                                          #
#                                                                        #
# Copyright 2016-2019 Baptiste PELLEGRIN                                 #
#                                                                        #
# This file is part of GNUVario.                                         #
#                                                                        #
# GNUVario is free software: you can redistribute it and/or modify       #
# it under the terms of the GNU General Public License as published by   #
# the Free Software Foundation, either version 3 of the License, or      #
# (at your option) any later version.                                    #
#                                                                        #
# GNUVario is distributed in the hope that it will be useful,            #
# but WITHOUT ANY WARRANTY; without even the implied warranty of         #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          #
# GNU General Public License for more details.                           #
#                                                                        #
# You should have received a copy of the GNU General Public License      #
# along with this program.  If not, see <https://www.gnu.org/licenses/>. #
##########################################################################

from __future__ import print_function
from __future__ import division
import numpy as np
from math import *

#||||||||||||||||||||||||||||||||||||#
#                                    #
#       2D BEST-FIT ALGORITHMS       #
#                                    #
#||||||||||||||||||||||||||||||||||||#

###########################################
#             CIRCLE MODEL                #
# A circle is defined by a center z and   #
# a radius in a single vector.            #
###########################################
def applyCircleCorrection(circle, ep):

    m = ep.shape[0]
    z = circle[0:2]
    radius = circle[2]
    
    for vect in ep:
        for i in range(2):
            vect[i] = (vect[i] - z[i])/radius


######################################################
#               ELLIPSE MODEL                        #
# An ellipse is defined by the parametric form :     #
#  x = z0 + a0 * cos(phi)                            #
#  y = z1 + a1 * sin(phi)                            #
#                                                    #
# So the ellipse vector contain in order :           #
# -> The m phi values corresponding to the m points  #
# -> The ellipse center z       (ellipse[-4:-2])     #
# -> The ellipse coefficients a (ellipse[-2:])       #
######################################################
def applyEllipseCorrection(ellipse, ep):

    m = ep.shape[0]
    phi = ellipse[0:m]
    z = ellipse[-4: -2]
    a = ellipse[-2:]

    for vect in ep:
        for i in range(2):
            vect[i] = (vect[i] - z[i])/a[i]
    


#--------------------------#
#     BEST-FIT CIRCLE      #
#       minimizing         #
#    ALGEBRAIC distance    # 
#--------------------------#
def algebraicLeastSquaresCircle( ep ):

    m = ep.shape[0]

    # create linear matrix corresponding to circle equation #
    M = np.zeros( (m,4) )
    for i in range(m):
        M[i,0] = ep[i,0]**2 + ep[i,1]**2
        M[i,1] = ep[i,0]
        M[i,2] = ep[i,1]
        M[i,3] = 1.0

    # compute SVD decomposition #
    S,V,D = np.linalg.svd(M)

    # find smallest singular value #
    sp = 0
    s = V[0]
    for i in range(4):
        if( V[i] < s ):
            sp = i
            s = V[i]

    # get the corresponding least-squares vector #
    lsv = D[sp]
    err = np.linalg.norm( M.dot(lsv) )/m

    # get the sphere parameters #
    circle = np.array( [-lsv[1]/(2*lsv[0]),
                        -lsv[2]/(2*lsv[0]),
                        sqrt( (lsv[1]**2 + lsv[2]**2)/(4*(lsv[0]**2)) - (lsv[3]/lsv[0]) )] )

    # compute geometric error #
    gerr = 0.0
    for i in range(m):
        gerr +=  (np.linalg.norm(ep[i] - circle[0:2]) - circle[2])**2
    gerr = sqrt(gerr)/m
        

    return circle, err, gerr


def algebraicLeastSquaresCirclePrint(ep):

    circle, err, gerr = algebraicLeastSquaresCircle(ep)
    print("--------------------------------------")
    print("Algebraic least-squares circle results :")
    print("--------------------------------------")
    print("( algebraic error =", err, ")")
    print("geometric error =", gerr)
    print("center = (", sphere[0], ",", sphere[1],  ")") 
    print("radius =", sphere[2])
    print(" ")
    
    return circle, err, gerr
    

#--------------------------#
#     BEST-FIT CIRCLE      #
#       minimizing         #
#    GEOMETRIC distance    # 
#--------------------------#

##########################################
#     Newton-Gauss function vector       #
#  (need to be least-squares minimized)  #
##########################################
def computeCircleR(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    z = beta[0:2]
    radius = beta[2]

    # compute r #
    r = np.zeros(m)

    for i in range(m):
        vect = ep[i] - z
        r[i] = radius - np.linalg.norm(vect)
        
    return r


###################################
#  Newton-Gauss optimization step #
###################################
def NewtonGaussCircleStep(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    z = beta[0:2]
    radius = beta[2]

    # compute jacobian #
    J = np.zeros( (m, 3) )

    for i in range(m):
        for j in range(2):
            J[i,j] = (z[j]-ep[i,j])/np.linalg.norm( z - ep[i] )

        J[i, 2] = -1.0
    
    # compute function vector r #
    r = computeCircleR(beta, ep)

    # compute delta and apply #
    delta = np.linalg.lstsq(J,r)[0]
    beta = beta + delta
    
    return beta, np.linalg.norm(delta)
    

#################################
#  Newton-Gauss best-fit circle #
#################################
def NewtonGaussCircle(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):

    ndelta = deltaThreshold + 1.0
    NGSteps = 0
    while( NGSteps < minNGSteps or ( NGSteps < maxNGSteps and ndelta > deltaThreshold ) ):
        beta, ndelta = NewtonGaussCircleStep(beta, ep)
        NGSteps += 1
        
    # normalize radius #
    beta[2] = abs(beta[2])

    # compute error #
    m = ep.shape[0]
    r = computeCircleR(beta, ep)
    err = np.linalg.norm(r)/m

    return beta, ndelta, NGSteps, err

def NewtonGaussCirclePrint(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):

    beta, ndelta, NGSteps, err = NewtonGaussCircle(minNGSteps, maxNGSteps, deltaThreshold, beta, ep)
    print("----------------------------------------------------------------")
    print("Newton-Gauss least-squares geometrical distance circle results :")
    print("----------------------------------------------------------------")
    print("geometric error =", err)
    print("Newton-Gauss Steps =", NGSteps)
    print("center = (", beta[O], ",", beta[1], ")")
    print("radius =", beta[2],)
    print(" ")

    return beta, ndelta, NGSteps, err


#-------------------------#
#   BEST-FIT ELLIPSE      #
#     minimizing          #
#   GEOMETRIC distance    #
#-------------------------#

#############################################################
# Determine trivial ellipse Newton-Gauss vector from circle #
#############################################################
def circularEllipse(circle, ep):

    m = ep.shape[0]

    # get the ellipse center z from circle center #
    z = circle[0:2]

    # get the ellipse coefficients from circle radius #
    a = np.array( [circle[2], circle[2]] )

    # compute experimental points circular coordinates #
    phi = np.zeros(m)
    for i in range(m):
        c = (ep[i,0] - z[0]) + 1j * (ep[i,1] - z[1])
        phi[i] = np.angle(c)

    # concatenate to Newton-Gauss vector #
    return np.hstack( (phi, z, a) )


##########################################
#     Newton-Gauss function vector       #
#  (need to be least-squares minimized)  #
##########################################
def computeEllipseR(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    phi = beta[0:m]
    z = beta[m: m + 2]
    a = beta[m + 2: m + 4]

    # compute r #
    r = np.zeros( 2*m )
    for i in range(0, m):
        r[i] = z[0] + a[0]*cos(phi[i]) - ep[i,0]
        r[m+i] = z[1] + a[1]*sin(phi[i]) - ep[i,1]

    return r    

###################################
#  Newton-Gauss optimization step #
###################################
def NewtonGaussEllipseStep(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    phi = beta[0:m]
    z = beta[m: m + 2]
    a = beta[m + 2: m + 4]


    # compute jacobian #
    J = np.zeros( (2*m, m+4) )

    for i in range(0, m):
        J[i,i] = a[0]* sin(phi[i])
        J[m+i,i] = -a[1] * cos(phi[i])

        J[i, m+0] = -1.0
        J[i, m+1] = 0.0
        J[i, m+2] = - cos(phi[i])
        J[i, m+3] = 0.0

        J[m+i, m+0] = 0.0
        J[m+i, m+1] = -1.0
        J[m+i, m+2] = 0.0
        J[m+i, m+3] = - sin(phi[i])

  
    # compute function vector r #
    r = computeEllipseR(beta, ep)

    # compute delta and apply #
    delta = np.linalg.lstsq(J,r)[0]
    beta = beta + delta
    
    return beta, np.linalg.norm(delta)


##################################
#  Newton-Gauss best-fit ellipse #
##################################
def NewtonGaussEllipse(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):

    ndelta = deltaThreshold + 1.0
    NGSteps = 0
    while( NGSteps < minNGSteps or ( NGSteps < maxNGSteps and ndelta > deltaThreshold ) ):
        beta, ndelta = NewtonGaussEllipseStep(beta, ep)
        NGSteps += 1
    
    # normalize coeffs #
    for i in range(2):
        beta[-2+i] = abs(beta[-2+i])

    # compute error #
    m = ep.shape[0]
    r = computeEllipseR(beta, ep)
    err = np.linalg.norm(r)/m

    return beta, ndelta, NGSteps, err


def NewtonGaussEllipsePrint(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):
    
    beta, ndelta, NGSteps, err = NewtonGaussEllipse(minNGSteps, maxNGSteps, deltaThreshold, beta, ep)
    print("----------------------------------------------------------------")
    print("Newton-Gauss least-squares geometrical distance Ellipse results :")
    print("----------------------------------------------------------------")
    print("geometric error =", err)
    print("Newton-Gauss Steps =", NGSteps)
    print("center = (", beta[-4], ",", beta[-3], ")")
    print("coeffs = (", beta[-2], ",", beta[-1], ")")
    print(" ")

    return beta, ndelta, NGSteps, err
    

#||||||||||||||||||||||||||||||||||||#
#                                    #
#       3D BEST-FIT ALGORITHMS       #
#                                    #
#||||||||||||||||||||||||||||||||||||#


###########################################
#             SPHERE MODEL                #
# A sphere is defined by a center z and   #
# a radius in a single vector.            #
###########################################
def applySphereCorrection(sphere, ep):

    m = ep.shape[0]
    z = sphere[0:3]
    radius = sphere[3]
    
    for vect in ep:
        for i in range(3):
            vect[i] = (vect[i] - z[i])/radius


#######################################################
#             ELLIPSOID MODEL                         #
# An ellipsoid is defined by the parametric form :    #
# x = z0 + a0*cos(phi)*cos(theta)                     #  
# y = z1 + a1*cos(phi)*sin(theta)                     #
# z = z2 + a2*sin(phi)                                #
#                                                     #
# So the ellipsoid vector contain in order :          #
# -> The m phi values corresponding to the m points   #
# -> The m theta values corresponding to the m points #
# -> The ellipsoid center z       (ellipsoid[-6:-3])  #
# -> The ellipsoid coefficients a (ellipsoid[-3:])    #
#######################################################
def applyEllipsoidCorrection(ellipsoid, ep):

    m = ep.shape[0]
    phi = ellipsoid[0:m]
    z = ellipsoid[-6: -3]
    a = ellipsoid[-3:]

    for vect in ep:
        for i in range(3):
            vect[i] = (vect[i] - z[i])/a[i]


#--------------------------#
#     BEST-FIT SPHERE      #
#       minimizing         #
#    ALGEBRAIC distance    # 
#--------------------------#
def algebraicLeastSquaresSphere( ep ):

    m = ep.shape[0]

    # create linear matrix corresponding to sphere equation #
    M = np.zeros( (m,5) )
    for i in range(m):
        M[i,0] = ep[i,0]**2 + ep[i,1]**2 + ep[i,2]**2
        M[i,1] = ep[i,0]
        M[i,2] = ep[i,1]
        M[i,3] = ep[i,2]
        M[i,4] = 1.0

    # compute SVD decomposition #
    S,V,D = np.linalg.svd(M)

    # find smallest singular value #
    sp = 0
    s = V[0]
    for i in range(5):
        if( V[i] < s ):
            sp = i
            s = V[i]

    # get the corresponding least-squares vector #
    lsv = D[sp]
    err = np.linalg.norm( M.dot(lsv) )/m

    # get the sphere parameters #
    sphere = np.array( [-lsv[1]/(2*lsv[0]),
                        -lsv[2]/(2*lsv[0]),
                        -lsv[3]/(2*lsv[0]),
                        sqrt( (lsv[1]**2 + lsv[2]**2 + lsv[3]**2)/(4*(lsv[0]**2)) - (lsv[4]/lsv[0]) )])

    # compute geometric error #
    gerr = 0.0
    for i in range(m):
        gerr +=  (np.linalg.norm(ep[i] - sphere[0:3]) - sphere[3])**2
    gerr = sqrt(gerr)/m
        

    return sphere, err, gerr


def algebraicLeastSquaresSpherePrint(ep):

    sphere, err, gerr = algebraicLeastSquaresSphere(ep)
    print("--------------------------------------")
    print("Algebraic least-squares sphere results :")
    print("--------------------------------------")
    print("( algebraic error =", err, ")")
    print("geometric error =", gerr)
    print("center = (", sphere[0], ",", sphere[1], ",", sphere[2], ")") 
    print("radius =", sphere[3])
    print(" ")

    return sphere, err, gerr

    
#--------------------------#
#     BEST-FIT SPHERE      #
#       minimizing         #
#    GEOMETRIC distance    # 
#--------------------------#

##########################################
#     Newton-Gauss function vector       #
#  (need to be least-squares minimized)  #
##########################################
def computeSphereR(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    z = beta[0:3]
    radius = beta[3]

    # compute r #
    r = np.zeros(m)

    for i in range(m):
        vect = ep[i] - z
        r[i] = radius - np.linalg.norm(vect)
        
    return r


###################################
#  Newton-Gauss optimization step #
###################################
def NewtonGaussSphereStep(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    z = beta[0:3]
    radius = beta[3]

    # compute jacobian #
    J = np.zeros( (m, 4) )

    for i in range(m):
        for j in range(3):
            J[i,j] = (z[j]-ep[i,j])/np.linalg.norm( z - ep[i] )

        J[i, 3] = -1.0
    
    # compute function vector r #
    r = computeSphereR(beta, ep)

    # compute delta and apply #
    delta = np.linalg.lstsq(J,r)[0]
    beta = beta + delta
    
    return beta, np.linalg.norm(delta)
    

#################################
#  Newton-Gauss best-fit sphere #
#################################
def NewtonGaussSphere(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):

    ndelta = deltaThreshold + 1.0
    NGSteps = 0
    while( NGSteps < minNGSteps or ( NGSteps < maxNGSteps and ndelta > deltaThreshold ) ):
        beta, ndelta = NewtonGaussSphereStep(beta, ep)
        NGSteps += 1

    # normalize radius #
    beta[3] = abs(beta[3])

    # compute error #
    m = ep.shape[0]
    r = computeSphereR(beta, ep)
    err = np.linalg.norm(r)/m

    return beta, ndelta, NGSteps, err


def NewtonGaussSpherePrint(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):
    
    beta, ndelta, NGSteps, err = NewtonGaussSphere(minNGSteps, maxNGSteps, deltaThreshold, beta, ep)
    print("----------------------------------------------------------------")
    print("Newton-Gauss least-squares geometrical distance Sphere results :")
    print("----------------------------------------------------------------")
    print("geometric error =", err)
    print("Newton-Gauss Steps =", NGSteps)
    print("center = (", beta[0], ",", beta[1], ",", beta[2], ")")
    print("radius =", beta[3])
    print(" ")

    return beta, ndelta, NGSteps, err
    


#-------------------------#
#   BEST-FIT ELLIPSOID    #
#     minimizing          #
#   GEOMETRIC distance    #
#-------------------------#

###############################################################
# Determine trivial ellipsoid Newton-Gauss vector from sphere #
###############################################################
def sphericalEllipsoid(sphere, ep):

    m = ep.shape[0]

    # get the ellipsoid center z from sphere center #
    z = sphere[0:3]

    # get the ellipsoid coefficients a from sphere radius #
    a = np.array( [sphere[3], sphere[3], sphere[3]] )

    # compute experimental points spherical coordinates #
    phi = np.zeros(m)
    theta = np.zeros(m)

    for i in range(m):
        v = ep[i] - z
        rho = np.linalg.norm(v)
        phi[i] = asin(v[2] / rho)
        theta[i] = atan2(v[1], v[0])


    # concatenate to Newton-Gauss vector #
    return np.hstack( (phi, theta, z, a) )


##########################################
#     Newton-Gauss function vector       #
#  (need to be least-squares minimized)  #
##########################################
def computeEllipsoidR(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    phi = beta[0:m]
    theta = beta[m:2*m]
    z = beta[2*m: 2*m + 3]
    a = beta[2*m + 3: 2*m + 6]

    # compute r #
    r = np.zeros( 3*m )
    for i in range(m):
        r[i] = z[0] + a[0]*cos(phi[i])*cos(theta[i]) - ep[i,0]
        r[m+i] = z[1] + a[1]*cos(phi[i])*sin(theta[i]) - ep[i,1]
        r[2*m+i] = z[2] + a[2]*sin(phi[i]) - ep[i,2]

    return r    

###################################
#  Newton-Gauss optimization step #
###################################
def NewtonGaussEllipsoidStep(beta, ep):

    # analyse Newton-Gauss vector #
    m = ep.shape[0]
    phi = beta[0:m]
    theta = beta[m:2*m]
    z = beta[2*m: 2*m + 3]
    a = beta[2*m + 3: 2*m + 6]

    # compute jacobian #
    J = np.zeros( (3*m, 2*m + 6) )

    for i in range(m):
        J[i, i] = a[0]*sin(phi[i])*cos(theta[i])
        J[m+i, i] = a[1]*sin(phi[i])*sin(theta[i])
        J[2*m+i, i] = -a[2]*cos(phi[i])

        J[i, m+i] = a[0]*cos(phi[i])*sin(theta[i])
        J[m+i, m+i] = -a[1]*cos(phi[i])*cos(theta[i])

        J[i, 2*m + 0] = -1
        J[i, 2*m + 3] = -cos(phi[i])*cos(theta[i])

        J[m+i, 2*m + 1] = -1
        J[m+i, 2*m + 4] = -cos(phi[i])*sin(theta[i])

        J[2*m+i, 2*m + 2] = -1
        J[2*m+i, 2*m + 5] = -sin(phi[i])

    # compute function vector r #
    r = computeEllipsoidR(beta, ep)

    # compute delta and apply #
    delta = np.linalg.lstsq(J,r)[0]
    beta = beta + delta
    
    return beta, np.linalg.norm(delta)


####################################
#  Newton-Gauss best-fit ellipsoid #
####################################
def NewtonGaussEllipsoid(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):

    ndelta = deltaThreshold + 1.0
    NGSteps = 0
    while( NGSteps < minNGSteps or ( NGSteps < maxNGSteps and ndelta > deltaThreshold ) ):
        beta, ndelta = NewtonGaussEllipsoidStep(beta, ep)
        NGSteps += 1

    # normalize coeffs #
    for i in range(3):
        beta[-3+i] = abs(beta[-3+i])

    # compute error #
    m = ep.shape[0]
    r = computeEllipsoidR(beta, ep)
    err = np.linalg.norm(r)/m

    return beta, ndelta, NGSteps, err


def NewtonGaussEllipsoidPrint(minNGSteps, maxNGSteps, deltaThreshold, beta, ep):

    beta, ndelta, NGSteps, err = NewtonGaussEllipsoid(minNGSteps, maxNGSteps, deltaThreshold, beta, ep)
    print("------------------------------------------------------------------")
    print("Newton-Gauss least-squares geometrical distance Ellipsoid results :")
    print("------------------------------------------------------------------")
    print("geometric error =", err)
    print("Newton-Gauss Steps =", NGSteps)
    print("center = (", beta[-6], ",", beta[-5], ",", beta[-4], ")")
    print("coeffs = (", beta[-3], ",", beta[-2], ",", beta[-1], ")")
    print(" ")

    return beta, ndelta, NGSteps, err



#-----------------------------------------------------------#
#  Three step best-fit ellipsoid :                          #
#  1) Least-squares sphere minimizing algebraic distance    #
#  2) Newton-Gauss sphere minimizing geometric distance     #
#  3) Newton-Gauss ellipsoid minimazing geometric distance  #
#-----------------------------------------------------------#
def bestFitEllipsoid(minNGSteps, maxNGSteps, deltaThreshold, ep):

    # 1) determine algebraic least-squares sphere #
    aSphere, aerr, agerr = algebraicLeastSquaresSpherePrint(ep)

    # 2) determine geometric least-squares sphere #
    gSphere, ndelta, NGSteps, gerr = NewtonGaussSpherePrint(minNGSteps, maxNGSteps, deltaThreshold, aSphere, ep)

    # determine the corresponding Newton-Gauss ellipsoid vector #
    beta = sphericalEllipsoid(aSphere, ep)

    # 3) determine geometric least-squares ellipsoid #
    beta, ndelta, NGSteps, err = NewtonGaussEllipsoidPrint(minNGSteps, maxNGSteps, deltaThreshold, beta, ep)

    # return result #
    return aSphere, agerr, gSphere, gerr, beta, err
              
    
    

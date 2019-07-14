##########################################################################
# calibrate -- Accelerometer/magnetometer calibration based on data      #
#              recorded by calibration_recorder                          #
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
import sys
from bestfit import *
import csv
import numpy as np
from math import *


DEFAULT_RECORD_FILE = "RECORD00.IGC"
VERTACCEL_CAL_SCALE_MULTIPLIER = 16        # scale = 1 + value/2^multiplier
ACCEL_BASE_SCALE = 13                      # scaled by 2^x
MAG_BASE_PROJ_SCALE = 7                    # scaled by 2^x 

COEFFICIENT_MULTIPLIER = 15 # q15 format, 16 bit coefficient of max 2.0

# newtow gauss parameters #
MIN_NEWTON_GAUSS_STEP = 1
MAX_NEWTON_GAUSS_STEP = 500
NEWTON_GAUSS_THRESHOLD = 1.0e-10


############################
# read experimental points #
############################
haveMag = True

gyroCal = np.zeros(12);
accel = np.zeros( (0,3) )
mag = np.zeros( (0,3) )

# read csv #
if( len(sys.argv) > 1 ):
    recordFile = sys.argv[1]
else:
    recordFile = DEFAULT_RECORD_FILE


with open(recordFile, 'rt') as csvfile:
    reader = csv.reader(csvfile)

    # the first line is the gyro calibration data #
    gyroVect = next(reader);
    gyroCal = np.array(gyroVect, dtype=int)

    # next lines are accel/mag data #
    for row in reader:
        if( len(row) != 3 and len(row) != 6 ):
            print("Bad measure line !")
        else:
            newAccel = np.array( row[0:3], dtype=np.float64 )
            accel = np.vstack( (accel, newAccel) )
            if( len(row) == 3 ):
                haveMag = False
            else:
                newMag = np.array( row[3:6], dtype=np.float64 )
                mag = np.vstack( (mag, newMag) )

    
###########################
# run best fit algorithms #
###########################
print("###############################")
print("#  Accelerometer calibration  #")
print("###############################")
print(" ")
accelASphere, accelAgerr, accelGSphere, accelGerr, accelBeta, accelErr = bestFitEllipsoid(MIN_NEWTON_GAUSS_STEP, MAX_NEWTON_GAUSS_STEP, NEWTON_GAUSS_THRESHOLD, accel)
print(" ")

if haveMag:
    print("###############################")
    print("#      Mag calibration        #")
    print("###############################")
    print(" ")
    magASphere, magAgerr, magGSphere, magGerr, magBeta, magErr = bestFitEllipsoid(MIN_NEWTON_GAUSS_STEP, MAX_NEWTON_GAUSS_STEP, NEWTON_GAUSS_THRESHOLD, mag)


####################
# Apply correction #
####################

# use least square geometrical distance sphere #
applySphereCorrection(accelGSphere, accel);
if haveMag:
    applySphereCorrection(magGSphere, mag);


###################################
# compute projection coefficients #
###################################
def leastSquaresProjectionCoefficient(normalizedAccel, normalizedMag):

    m = normalizedAccel.shape[0]
    
    dotMean = 0
    dotSD = 0
    scaleMean = 0
    scaleSD = 0

    for i in range(m):
        accel = normalizedAccel[i]
        mag = normalizedMag[i]

        # compute dot product #
        dot = -accel.dot(mag)
        dotMean += dot
        dotSD += dot**2

        # project #
        proj = mag + dot*accel
        scale = np.linalg.norm(proj);
        scaleMean += scale
        scaleSD += scale**2

    # compute results #
    dotMean /= m
    dotSD /= m
    dotSD = dotSD - dotMean**2
    scaleMean /= m
    scaleSD /= m
    scaleSD = scaleSD - scaleMean**2

    pcoeff = np.zeros(2, dtype = float)
    perr = np.zeros(2)

    pcoeff[0] = dotMean
    pcoeff[1] = scaleMean
    perr[0] = dotSD
    perr[1] = scaleSD

    return pcoeff, perr

if haveMag:
    projCoeff, projErr = leastSquaresProjectionCoefficient(accel, mag)
    print("Mag projection coefficients =", projCoeff)
    print("Projection error =", projErr)
    print(" ")

##########################
# search best multiplier #
##########################

# accel multiplier #
bestAccelMultiplier = -1;
for i in range(3):
    accelMultiplier = int(floor( log( float(2**15 - 1)/abs(accelGSphere[i]), 2 ) ));
    if (bestAccelMultiplier < 0) or (accelMultiplier < bestAccelMultiplier) :
        bestAccelMultiplier = accelMultiplier 

if( bestAccelMultiplier > 30 ) :
    bestAccelMultiplier = 30

if haveMag:
    # mag multiplier #
    bestMagMultiplier = -1;
    for i in range(3):
        magMultiplier = int(floor( log( float(2**15 - 1)/abs(magGSphere[i]), 2 ) ));
        if (bestMagMultiplier < 0) or (magMultiplier < bestMagMultiplier) :
            bestMagMultiplier = magMultiplier 

    if( bestMagMultiplier > 30 ) :
        bestMagMultiplier = 30

    
#############################
# output calibration values #
#############################

print("############")
print("#  Result  #")
print("############")
print(" ")

# signal big drift #
if( bestAccelMultiplier < 5 ) :
    print("!!! Warning : you have high drift on your accel measures !!!")
    print("!!! Maybe something disturb your accelerometer.          !!!")
    print(" ")

print("Please copy and paste there settings in VarioSettings.h")
print(" ")


# gyro calibration #
sys.stdout.write("#define VERTACCEL_GYRO_CAL_BIAS {")
for i in range(12):
    sys.stdout.write( format(gyroCal[i], '#04x') )
    if( i != 11 ):
        sys.stdout.write(", ")
print("}")

# accel bias #
sys.stdout.write("#define VERTACCEL_ACCEL_CAL_BIAS {")
for i in range(3):
    sys.stdout.write(str(int(np.rint(  accelGSphere[i]*float(2**bestAccelMultiplier)  ))))
    if( i != 2 ):
        sys.stdout.write(", ")
print("}")

# accel scale #
print("#define VERTACCEL_ACCEL_CAL_SCALE", int(np.rint(  float(2**(ACCEL_BASE_SCALE+VERTACCEL_CAL_SCALE_MULTIPLIER))/accelGSphere[3] - float(2**VERTACCEL_CAL_SCALE_MULTIPLIER) )) )

if haveMag:
    # mag bias #
    sys.stdout.write("#define VERTACCEL_MAG_CAL_BIAS {")
    for i in range(3):
        sys.stdout.write(str(int(np.rint(  magGSphere[i]*float(2**bestMagMultiplier)  ))))
        if( i != 2 ):
            sys.stdout.write(", ")
    print("}")

    # mag proj scale #
    print("#define VERTACCEL_MAG_CAL_PROJ_SCALE", int(np.rint(  float(2**(MAG_BASE_PROJ_SCALE+VERTACCEL_CAL_SCALE_MULTIPLIER))/(projCoeff[1]*magGSphere[3]) - float(2**VERTACCEL_CAL_SCALE_MULTIPLIER)  )) )

    # accel multiplier #
    print("#define VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER", bestAccelMultiplier)

    # mag multiplier #
    print("#define VERTACCEL_MAG_CAL_BIAS_MULTIPLIER", bestMagMultiplier)

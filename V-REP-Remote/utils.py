from math import *
def limitToOneOfPoles(angle):
    while(angle > 2*pi):
        angle -= 2*pi
    while(angle <0):
        angle += 2*pi
    
    error = {}
    poles = [0,pi/2,pi,3.0/2*pi,2*pi]
    error = {pole:abs(pole-angle) for pole in poles}
    return min(error,key=error.get)
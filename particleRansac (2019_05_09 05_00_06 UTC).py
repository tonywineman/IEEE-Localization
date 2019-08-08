import math
import random
import matplotlib.pyplot as plt
import numpy as np
import timeit

def getDist(pt1, pt2):
    return math.sqrt(math.pow(pt2[0] - pt1[0], 2) + math.pow(pt2[1] - pt1[1], 2))

#returns true if the angCheck is between ang1 and ang2 and accounts for 360 degree crossover
def angleInRange(ang1, ang2, angCheck):
    ang1 = (ang1 + 360) % 360           #probably don't need these 2 lines but just in case
    ang2 = (ang2 + 360) % 360
    angle = ((ang2 - ang1) + 360) % 360
    if angle >= 180:
        ang1,ang2 = ang2,ang1
    if ang1 <= ang2:
        return angCheck >= ang1 and angCheck <= ang2
    else:
        return angCheck >= ang1 or angCheck <= ang2

#gets the angle of a vector denoted by pt which is (x,y)
def getAngle(pt):
    if pt[0] == 0:
        pt[0] = 0.0000000001
    angle = math.degrees(math.atan(abs(pt[1] / pt[0])))
    if pt[0] < 0 and pt[1] >= 0:        #quadrant 2
        return 180 - angle
    elif pt[0] < 0 and pt[1] < 0:       #quadrant 3
        return 180 + angle
    elif pt[0] > 0 and pt[1] < 0:       #quadrant 4
        return 360 - angle
    else:
        return angle                    #quadrant 1

def expectedRadius(pt, angleCheck):        #pt is (x, y, angle of direction)

    # #outside wall vertici angles (A is lower left going clockwise)
    # angA = getAngle((0 - pt[0], 0 - pt[1])) - pt[2]
    # angB = getAngle((0 - pt[0], 2440 - pt[1])) - pt[2]
    # angC = getAngle((2440 - pt[0], 2440 - pt[1])) - pt[2]
    # angD = getAngle((2440 - pt[0], 0 - pt[1])) - pt[2]

    m = math.tan(math.radians(pt[2] + angleCheck))
    if m == 0:
        m = 0.0000000001
    b = pt[1] - (m * pt[0])

    #inside wall vertici angles (a is left most going clockwise)
    ang_a = getAngle((1057 - pt[0], 1220 - pt[1])) - pt[2]
    ang_b = getAngle((1220 - pt[0], 1383 - pt[1])) - pt[2]
    ang_c = getAngle((1383 - pt[0], 1220 - pt[1])) - pt[2]
    ang_d = getAngle((1220 - pt[0], 1057 - pt[1])) - pt[2]

    if angleCheck + pt[2] != 90 and angleCheck + pt[2] != 270:
        line1r = -pt[0] / math.cos(math.radians((angleCheck + pt[2])))
        line3r = (2440 - pt[0]) / math.cos(math.radians((angleCheck + pt[2])))
    else:
        line1r = 9999999999999999
        line3r = 9999999999999999

    if pt[2] + angleCheck != 0 and pt[2] + angleCheck != 180:
        line2r = (2440 - pt[1]) / math.sin(math.radians(angleCheck + pt[2]))
        line4r = -pt[1] / math.sin(math.radians(angleCheck + pt[2]))
    else:
        line2r = 9999999999999999
        line4r = 9999999999999999

    rads = [line1r, line2r, line3r, line4r]
    
    if pt[0] >= 1220 and pt[1] >= 1220:     #quadrant I #checking if the point is in the upper right of the cordinate system (this is a specific color)
        
        #time to do inner box
        if angleInRange(ang_b, ang_c, pt[2] + angleCheck):
            # print("1")
            x = (2456.3 - b) / (m + 1)
            y = m * x + b
            line5r = getDist((x,y), pt)
            rads.append(line5r)
        if ang_a < ang_b and angleInRange(ang_a, ang_b, pt[2] + angleCheck):
            # print("2")
            x = (16.3 - b) / (m - 1)
            y = m * x + b
            line6r = getDist((x,y), pt)
            rads.append(line6r)
        if ang_d > ang_c and angleInRange(ang_d, ang_c, pt[2] + angleCheck):
            # print("3")
            # print(m,b)
            x = (-16.3 - b) / (m - 1)
            y = m * x + b
            # print(x,y)
            line7r = getDist((x,y), pt)
            rads.append(line7r)

    elif pt[0] <= 1220 and pt[1] >= 1220:     #quadrant II #checking if the point is in the upper left of the cordinate system (this is a specific color)
        #time to do inner box
        if angleInRange(ang_a, ang_b, pt[2] + angleCheck):
            # print("1")
            x = (16.3 - b) / (m - 1)
            y = m * x + b
            line5r = getDist((x,y), pt)
            rads.append(line5r)
        if ang_a > ang_d and angleInRange(ang_a, ang_d, pt[2] + angleCheck):
            # print("2")
            x = (2423.7 - b) / (m + 1)
            y = m * x + b
            line6r = getDist((x,y), pt)
            rads.append(line6r)
        if ang_c > ang_b and angleInRange(ang_b, ang_c, pt[2] + angleCheck) and pt[1] > 1383:
            # print("3")
            # print(m,b)
            x = (2456.3 - b) / (m + 1)
            y = m * x + b
            # print(x,y)
            line7r = getDist((x,y), pt)
            rads.append(line7r)

    elif pt[0] <= 1220 and pt[1] <= 1220:     #quadrant III #checking if the point is in the lower left of the cordinate system (this is a specific color)
        #time to do inner box
        if angleInRange(ang_a, ang_d, pt[2] + angleCheck):
            # print("1")
            x = (2456.3 - b) / (m + 1)
            y = m * x + b
            line5r = getDist((x,y), pt)
            rads.append(line5r)
        if ang_a < ang_b and angleInRange(ang_a, ang_b, pt[2] + angleCheck):
            # print("2")
            x = (16.3 - b) / (m - 1)
            y = m * x + b
            line6r = getDist((x,y), pt)
            rads.append(line6r)
        if ang_c < ang_d and angleInRange(ang_d, ang_c, pt[2] + angleCheck) and pt[1] < 1057:
            # print("3")
            # print(m,b)
            x = (2456.3 - b) / (m + 1)
            y = m * x + b
            # print(x,y)
            line7r = getDist((x,y), pt)
            rads.append(line7r)

    else:   #quadrant IV #checking if the point is in the lower right of the cordinate system (this is a specific color)
        #time to do inner box
        if angleInRange(ang_c, ang_d, pt[2] + angleCheck):
            # print("1")
            x = (-16.3 - b) / (m - 1)
            y = m * x + b
            line5r = getDist((x,y), pt)
            rads.append(line5r)
        if ang_a > ang_d and angleInRange(ang_a, ang_d, pt[2] + angleCheck):
            # print("2")
            x = (2423.7 - b) / (m + 1)
            y = m * x + b
            line6r = getDist((x,y), pt)
            rads.append(line6r)
        if ang_c > ang_b and angleInRange(ang_b, ang_c, pt[2] + angleCheck):
            # print("3")
            # print(m,b)
            x = (2456.3 - b) / (m + 1)
            y = m * x + b
            # print(x,y)
            line7r = getDist((x,y), pt)
            rads.append(line7r)

    out = 9999999999999999
    print(rads)
    for r in rads:
        if r < out and r > 0:
            out = r
    return out

        

        


print(expectedRadius((2430, 10, 0), 135))

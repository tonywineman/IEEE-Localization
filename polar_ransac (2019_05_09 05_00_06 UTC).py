import math
import random
import matplotlib.pyplot as plt
import numpy as np
import timeit
from new_square_draw import vertsToRobotLocation

"""
lidar_pts = [(theta, distance, weighting)]
color_sensor_pts = []
"""


SQUARE_SIDE_LENGTH = 2740
THRESHOLD = 0.07
ACCEPTABLE_PERCENTAGE = 0.9
ITERATIONS = 100000


def readLidarImage():
    with open("lidar_dataset/image1.txt") as f:
        content = f.readlines()

    lidar_points = []
    for line in content:
        line_data = line.split()
        quality = int(line_data[1])
        theta = float(line_data[2])
        distance = float(line_data[3])

        if (quality > 0):
            lidar_points.append((theta, distance, quality))

    return lidar_points


#takes in 2 cartesian points of the form (x,y) and outputs a list of all 4 verticies
def drawSquare(pt1, pt2):
    """
    https://www.quora.com/Given-two-diagonally-opposite-points-of-a-square-how-can-I-find-out-the-other-two-points-in-terms-of-the-coordinates-of-the-known-points
    """
    A = pt1
    C = pt2
    B = ((A[0] + C[0] + A[1] - C[1]) / 2, (C[0] - A[0] + A[1] + C[1]) / 2)
    D = ((A[0] + C[0] + C[1] - A[1]) / 2, (A[0] - C[0] + A[1] + C[1]) / 2)
    # print([A, B, C, D])
    # plt.plot((A[0], B[0], C[0], D[0], A[0]), (A[1], B[1], C[1], D[1], A[1]), color="blue")
    # plt.show()
    return [A, B, C, D]


def cartesianConvert(polarPt):
    return (polarPt[1] * math.cos(math.radians(polarPt[0])), polarPt[1] * math.sin(math.radians(polarPt[0])))


def getDist(pt1, pt2):
    return math.sqrt(math.pow(pt2[0] - pt1[0], 2) + math.pow(pt2[1] - pt1[1], 2))

def angleInRange(ang1, ang2, angCheck):
    angle = ((ang2 - ang1) + 360) % 360
    if angle >= 180:
        ang1,ang2 = ang2,ang1
    if ang1 <= ang2:
        return angCheck >= ang1 and angCheck <= ang2
    else:
        return angCheck >= ang1 or angCheck <= ang2

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


def pPtOnLine(cPt1, cPt2, pPtCheck, error_dist):
    if cPt2[0] - cPt1[0] != 0:
        m = (cPt2[1] - cPt1[1]) / (cPt2[0] - cPt1[0])
    else:
        m = (cPt2[1] - cPt1[1]) / .0000000001
    # print(m)
    b = cPt1[1] - m * cPt1[0]
    # print(b)
    r = b / (math.sin(math.radians(pPtCheck[0])) - m * math.cos(math.radians(pPtCheck[0])))
    
    if  r - error_dist <= pPtCheck[1] <= r + error_dist:
        return 1
    else:
        return 0


def ransac(lidar_points=None, color_sensor_pts=None, landmark_pts=None, impact_pts=None, error=0.02, sideLength=SQUARE_SIDE_LENGTH, percentOfPoints=.98, numIterations=100000):
    # landmark_pts = [(x, y, "camera", "pillar" or "internal wall" or "spacetels", color, how sure are that we saw it there)]
    # color_sensors_pts = [("colorsensr", color, weighting)]
    # impac_pts = ["impact_sensors"]
    
    iteration = 0
    random.seed(10)
    while iteration < numIterations:
        iteration += 1

        pPt1 = lidar_pts[random.randrange(len(lidar_pts))]
        pPt2 = lidar_pts[random.randrange(len(lidar_pts))]

        cPt1 = cartesianConvert(pPt1)
        cPt2 = cartesianConvert(pPt2)

        if getDist(cPt1, cPt2) > 1500:

            vecx = sideLength * math.sqrt(2) * (cPt2[0] - cPt1[0]) / getDist(cPt1, cPt2)
            vecy = sideLength * math.sqrt(2) * (cPt2[1] - cPt1[1]) / getDist(cPt1, cPt2)

            cPt2 = (vecx + cPt1[0], vecy + cPt1[1])
            # print(pPt1)
            # print(pPt2)
            # print(cPt1)
            # print(cPt2)

            totalPointsPossible = len(lidar_points) * 15
            # print(totalPointsPossible)\
            verts = drawSquare(cPt1, cPt2)
            # print(verts)

            # # plotting all the points and the square
            # for point in lidar_points:
            #     pt = cartesianConvert(point)
                # plt.plot(pt[0],pt[1], marker='o', markersize=3, color="red")
                
            # plt.plot((verts[0][0], verts[1][0], verts[2][0], verts[3][0], verts[0][0]), (verts[0][1], verts[1][1], verts[2][1], verts[3][1], verts[0][1]), color="blue")
            # plt.show()

            #finding angle of all the verticies
            thetaA = pPt1[0]
            thetaB = getAngle(verts[1])
            thetaC = getAngle(verts[2])
            thetaD = getAngle(verts[3])

            # print("thetas")
            # print(thetaA)
            # print(thetaB)
            # print(thetaC)
            # print(thetaD)

            # side = getDist(cPt1, cPt2)
            error_dist = sideLength * error
            score = 0
            maxCurrScore = 0

            plt.plot(0,0,marker ='o', markersize=5, color="black")
            # print(len(lidar_points))
            for point in lidar_pts:
                # print(point[0])
                if angleInRange(thetaA, thetaB, point[0]):
                    # print("1")
                    if pPtOnLine(verts[0], verts[1], point, error_dist):
                        # print("here1")
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="green")
                        score += point[2]
                    else:
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="red")
                elif angleInRange(thetaB, thetaC, point[0]):
                    # print("2")
                    if pPtOnLine(verts[1], verts[2], point, error_dist):
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="green")
                        score += point[2]
                        # print("here2")
                    else:
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="red")
                elif angleInRange(thetaC, thetaD, point[0]):
                    # print("3")
                    if pPtOnLine(verts[2], verts[3], point, error_dist):
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="green")
                        score += point[2]
                        # print("here3")
                    else:
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="red")
                else:
                    # print("4")
                    if pPtOnLine(verts[0], verts[3], point, error_dist):
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="green")
                        score += point[2]
                        # print("here4")
                    else:
                        plt.plot(point[1] * math.cos(math.radians(point[0])), point[1] * math.sin(math.radians(point[0])), marker='o', markersize=3, color="red")

                if score >= percentOfPoints * totalPointsPossible:
                    # print(score)
                    plt.plot((verts[0][0], verts[1][0], verts[2][0], verts[3][0], verts[0][0]), (verts[0][1], verts[1][1], verts[2][1], verts[3][1], verts[0][1]), color="blue")
                    plt.show()
                    return verts

                maxCurrScore += point[2]

                if maxCurrScore - score > (1 - percentOfPoints) * totalPointsPossible:
                    break
            plt.plot((verts[0][0], verts[1][0], verts[2][0], verts[3][0], verts[0][0]), (verts[0][1], verts[1][1], verts[2][1], verts[3][1], verts[0][1]), color="blue")
            plt.show()
            # print(score)




if __name__ == "__main__":
    lidar_pts = readLidarImage()
    
    # fig = plt.figure()
    # ax = plt.subplot(111, projection='polar')
    # line = ax.scatter([np.radians(point[0]) for point in lidar_pts], [point[1] for point in lidar_pts], s=5,
    #                        cmap=plt.cm.Greys_r, lw=0)
    # ax.grid(True)

    # for point in lidar_pts:
    #     pt = cartesianConvert(point)
    #     plt.plot(pt[0],pt[1], marker='o', markersize=3, color="red")
    # plt.show()

    start = timeit.default_timer()

    # print(ransac(lidar_pts))

    verts = ransac(lidar_pts)
    print(verts)
    print(vertsToRobotLocation(verts))

    stop = timeit.default_timer()

    print('Time: ', stop - start)
    # print(angleInRange(300, 87, 5))
    # print(pPtOnLine((-3,-1), (-1,-2), (216.87, 2.44, 15), .05))
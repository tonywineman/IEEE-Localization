import math
import random
import matplotlib.pyplot as plt
from random_square import random_square
import timeit

def cartesianConvert(polarCord):
    out = []
    for point in polarCord:
        out.append((point[0] * math.cos(point[1]), point[0] * math.sin(point[1])))
    return out


def getDist(pt1, pt2):
    out = math.sqrt(math.pow(pt2[0] - pt1[0], 2) + math.pow(pt2[1] - pt1[1], 2))
    return out


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


#pt1 is a tuple (x, y) that represents one point that makes up the line
#pt2 is a tuple (x, y) that represents one point that makes up the line
#ptcheck is a tuple (x,y) that we are check to see if it is on the line
#error is the acceptable percent error based on the side length to find if points lie in (range of 0 to 1)
#returns 1 if on line, otherwise 0
def ptOnLine(pt1, pt2, ptCheck, error):
    side = getDist(pt1, pt2)
    error_dist = side * error

    #making pt1 always to the left of pt2 for proper range checking
    if pt1[0] > pt2[0]:
        tmp = pt1
        pt1 = pt2
        pt2 = tmp
        
    #preparing stuff to check if x is in range
    #checking if it is not a vertical line
    if pt2[0] - pt1[0] != 0:
        m = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
    else:
        m = (pt2[1] - pt1[1]) / .0000000001
    
    # print(m)
    #if the slope is less then 1 then the line limiting the x limit is the line perpendicular to our line
    if m < 1:
        if m == 0:
            m = .0000000001
        m = -1 / m

    denom = math.cos(math.atan(1 / m))

    #check if x val is within range finally
    if pt1[0] - error_dist / denom <= ptCheck[0] <= pt2[0] + error_dist / denom:
        #checking if y is in overall range if pt1's y val is less than pt2's y val and vise versa
        yInRange = 0
        if pt1[1] < pt2[1]:
            if pt1[1] - error_dist / denom <= ptCheck[1] <= pt2[1] + error_dist / denom:
                yInRange = 1
        else:
            if pt2[1] - error_dist / denom <= ptCheck[1] <= pt1[1] + error_dist / denom:
                yInRange = 1

        if yInRange:
            #checking if it is not a vertical line and not a horizontal line
            if pt2[0] - pt1[0] != 0 and pt2[1] - pt1[1] != 0:
                m = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
                b = pt1[1] - m * pt1[0]
                yOnLine = m * ptCheck[0] + b
                if m < 1:
                    m = -1 / m
                if m != 0:
                    denom = math.cos(math.atan(1 / m))
                    if denom == 0:
                        return 1
                    else:
                        if yOnLine - error_dist / denom <= ptCheck[1] <= yOnLine + error_dist / denom:
                            return 1
                else:
                    if yOnLine - side * error <= ptCheck[1] <= yOnLine + side * error:
                        return 1
                        
            #if its a vertical line or a horizontal line and the x is in range and y is in range then it is good
            else:
                return 1
    return 0




#cartesianCord is a list of tuples of (x, y)
#sideLength is the known length of the side of the square be approximated
#error is the acceptable percent error based on the side length to find if points lie in (range of 0 to 1)
#percentOfPoints is the percentage of points that need to be included in the square to return a successful value (range of 0 to 1)
#numIterations is the number of pairs of points it will check
#outputs the verticies of the square on success or a zero on failure
def ransacSquare(cartesianCord, sideLength, error, percentOfPoints, numIterations):
    # for i in range(len(cartesianCord)):
    #     x = cartesianCord[i][0]
    #     y = cartesianCord[i][1]
    #     plt.plot(x,y, marker='o', markersize=3, color="red")

    iteration = 0
    random.seed(2)
    while iteration < numIterations:
        pt1 = cartesianCord[random.randrange(len(cartesianCord))]
        # print(pt1)
        pt2 = cartesianCord[random.randrange(len(cartesianCord))]
        # print(pt2)

        #don't bother checking points that are right next to each other and wont form a big enough square
        if sideLength * (1 - error) * math.sqrt(2) < getDist(pt1, pt2) < sideLength * (1 + error) * math.sqrt(2):
            verts = drawSquare(pt1, pt2)

            #plotting all the points and the square
            # for i in range(len(cartesianCord)):
            #     x = cartesianCord[i][0]
            #     y = cartesianCord[i][1]
            #     plt.plot(x,y, marker='o', markersize=3, color="red")
            # plt.plot((verts[0][0], verts[1][0], verts[2][0], verts[3][0], verts[0][0]), (verts[0][1], verts[1][1], verts[2][1], verts[3][1], verts[0][1]), color="blue")
            # plt.show()

            #now i need to count the number of points that are on the new square
            count = 0
            for point in cartesianCord:
                if ptOnLine(verts[0], verts[1], point, error) or ptOnLine(verts[1], verts[2], point, error) or ptOnLine(verts[2], verts[3], point, error) or ptOnLine(verts[0], verts[3], point, error):
                    count = count + 1
                    # print(count)
                    plt.plot(point[0], point[1], marker='o', markersize=3, color="green")
                else:
                    plt.plot(point[0], point[1], marker='o', markersize=3, color="red")
                if count >= percentOfPoints * len(cartesianCord):
                    plt.plot((verts[0][0], verts[1][0], verts[2][0], verts[3][0], verts[0][0]), (verts[0][1], verts[1][1], verts[2][1], verts[3][1], verts[0][1]), color="blue")
                    plt.show()
                    return verts
        iteration = iteration + 1
    return 0

def main():
    field_coords = [(n, 50) for n in range(-50, 51)]+[(50, n) for n in range(50, -51, -1)]+[(n, -50) for n in range(50, -51, -1)]+[(-50, n) for n in range(-50, 51)]
    lidar_coords = [(f[0]+random.randint(28, 32), f[1]+random.randint(28, 32)) for f in field_coords]
    # plot()
    # cart = []
    # for i in range(360):
    #     x = random.randrange(100)
    #     y = random.randrange(100)
    #     cart.append((x,y))
    #     plt.plot(x,y, marker='o', markersize=3, color="red")
    # plt.show()

    print(ransacSquare(lidar_coords, 100, .07, .8, 100000))
        
def readLidarImage():
    with open("lidar_dataset/image1.txt") as f:
        content = f.readlines();

    lidar_points = []
    for line in content:
        line_data = line.split()
        quality = int(line_data[1])
        angle = float(line_data[2])
        distance = float(line_data[3])

        if (quality > 0):
            lidar_points.append((angle, distance))

    return lidar_points

DMAX = 4000
IMIN = 0
IMAX = 50

# if __name__ == "__main__":
#     lidar_pts = readLidarImage()
    
#     fig = plt.figure()
#     ax = plt.subplot(111, projection='polar')
#     line = ax.scatter([point[0] for point in lidar_pts], [point[1] for point in lidar_pts], s=5,
#                            cmap=plt.cm.Greys_r, lw=0)
#     ax.set_rmax(DMAX)
#     ax.grid(True)

#     # plt.show()
#     print(ransacSquare(random_square(100, 1), 100, .025, .95, 100000))


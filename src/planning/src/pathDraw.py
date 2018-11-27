import numpy as np
from math import *
import paths
import matplotlib
from matplotlib import pyplot as plt
from time import sleep
from matplotlib.animation import FuncAnimation


def t(array):
    return np.asarray(array)


def norm(pnt):
    return np.linalg.norm(pnt)


def dis(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def G_(C, G, C_):
    CG = G - C
    CC_ = C_ - C
    normCQ = norm(CC_) * findCos(G, C, C_)
    normQG_ = sqrt(norm(CG) ** 2 - (norm(CC_) * findSin(G, C, C_)) ** 2)
    CG_ = (CG / norm(CG)) * (normCQ + normQG_)
    return CG_ + C


def findAng(A, B, C):
    cos_ = findCos(A, B, C)
    if cos_ > 1:
        cos_ = 1.0
    elif cos_ < -1:
        cos_ = -1.0

    return acos(cos_)


def findCos(A, B, C):
    # return cos(ang(ABC))
    BA = A - B
    BC = C - B
    return dot(BA, BC) / (norm(BA) * norm(BC))


def findSin(A, B, C):
    # print A, B, C
    cos_ = findCos(A, B, C)
    sin_ = 1 - cos_ ** 2
    return sin_


def dot(A, B):
    x1, y1 = A
    x2, y2 = B
    return x1 * x2 + y1 * y2


def findRotAngle(C, G_, C_):
    ang = findAng(C, G_, C_)
    CC_ = C_ - C
    CG_ = G_ - C
    aCC_ = atan2(CC_[1], CC_[0])
    aCG_ = atan2(CG_[1], CG_[0])
    if aCC_ < aCG_:
        return abs(ang)
    else:
        return -1. * abs(ang)


def transform(theta, pnt):
    x, y = pnt
    return t([
        [cos(theta), -sin(theta), x],
        [sin(theta), cos(theta), y],
        [0, 0, 1]
    ])


class PathGenerator:
    def __init__(self):
        self.objectPathRadius = 2.0
        self.objectPathSampling = 7
        self.contactSampling = 100
        self.objectLength = 0.725
        self.agentRadius = 0.175
        self.objectOffset = t([1.0, 0])
        self.agentInitialPosition = t([0., 0.])
        self.collideDistance = 0.2
        self.maxIter = 1000

        self.finished = False

        # object pose
        self.objectPosition = self.objectOffset
        self.objectOrientation = 0.0

        # threshold of desired location
        self.locThreshold = 0.4
        # index of the current object desired location
        self.objectPathIndex = 0
        # half circle path
        self.objectPath = []
        # generate a S-shaped path
        '''
        step = 0.6 * pi / self.objectPathSampling
        for i in range(1, self.objectPathSampling + 1):
            angle = step * i
            x = self.objectPathRadius * sin(angle)
            y = self.objectPathRadius * cos(angle) - self.objectPathRadius
            xy = t([x, y]) + self.objectOffset
            self.objectPath.append(xy)
        offset_ = t([-1 * self.objectPathRadius, -2 * self.objectPathRadius]) + self.objectOffset
        '''

        step = pi / self.objectPathSampling
        for i in range(1, self.objectPathSampling + 1):
            angle = step * i
            x = self.objectPathRadius * sin(angle)
            y = self.objectPathRadius * cos(angle) - self.objectPathRadius
            xy = t([x,y]) + self.objectOffset
            self.objectPath.append(xy)
        offset_ = t([-1 * self.objectPathRadius, -2 * self.objectPathRadius]) + self.objectOffset
        self.objectPath.append(offset_)
        for i in range(1, self.objectPathSampling + 1):
            angle = step * i
            x = -1 * self.objectPathRadius * sin(angle)
            y = self.objectPathRadius * cos(angle) - self.objectPathRadius
            xy = t([x, y]) + offset_
            self.objectPath.append(xy)
        final = offset_ + t([self.objectPathRadius, -2 * self.objectPathRadius])
        self.objectPath.append(final)
        # vertices of a cubic box
        vert = [
            [-0.5 * self.objectLength, -0.5 * self.objectLength],
            [-0.5 * self.objectLength, 0.5 * self.objectLength],
            [0.5 * self.objectLength, 0.5 * self.objectLength],
            [0.5 * self.objectLength, -0.5 * self.objectLength]
        ]
        '''
        vert = [
        [-0.5 * 0.45, -0.5 * self.objectLength],
        [-0.5 * 0.45, 0.5 * self.objectLength],
        [0.5 * 0.45, 0.5 * self.objectLength],
        [0.5 * 0.45, -0.5 * self.objectLength]
        ]
        # right triangle
        vert = [
        [self.objectLength, 0.]),
        [-0.5 * self.objectLength, 0.5 * sqrt(3) * self.objectLength],
        [-0.5 * self.objectLength, -0.5 * sqrt(3) * self.objectLength]
        ]
        # Z-shaped object
        vert = [
        [0.7, 0.6],
        [0.5, 0.6],
        [0.5, 0.1],
        [-0.7, 0.1],
        [-0.7, -0.6],
        [-0.5, -0.6],
        [-0.5, -0.1],
        [0.7, -0.1]
        ]
        # triangle
        vert = [
        [0.3, 0.3],
        [-0.3, 0.3],
        [-0.3, -1.0]
        ]
        '''

        self.box_vert = []
        for i in vert:
            self.box_vert.append(t(i))
        self.box_vert.append(t(vert[0]))


    def desiredObjectLoc(self, curLoc):
        desLoc = self.objectPath[self.objectPathIndex]
        while dis(curLoc, desLoc) < self.locThreshold:
            self.objectPathIndex += 1
            if self.objectPathIndex >= len(self.objectPath):
                self.finished = True
                return None
            desLoc = self.objectPath[self.objectPathIndex]
        return desLoc


    def crossProductZSign(self, a, b):
        x1, y1 = a
        x2, y2 = b
        return x1 * y2 - x2 * y1 > 0


    def findVerticalVector(self, vector, direction):
        '''
        :param vector: (x,y), np
        :param direction: (m,n), np
        :return: a unit vector that is perpendicular to VECTOR along DIRECTION

        a vector that is perpendicular to (x,y) can be written as (a,b),
        a = ky, b = -kx

        use the sign of cross product to detect direction
        '''
        x, y = vector
        a = y
        b = -x
        res = np.asarray([a, b])
        res /= np.linalg.norm(res)
        if self.crossProductZSign(vector, res) == self.crossProductZSign(vector, direction):
            return res
        else:
            return -1. * res


    def sampleContactPoints(self):
        trans = transform(self.objectOrientation, self.objectPosition)
        cc_ = self.collideDistance
        c = []
        c_ = []
        num_sides = len(self.box_vert) - 1
        pntPerSide = self.contactSampling / num_sides

        for i in range(num_sides):
            vector = self.box_vert[i + 1] - self.box_vert[i]
            space = np.linalg.norm(vector) / pntPerSide
            # center of mass
            direction = t([0, 0]) - self.box_vert[i]
            for m in range(int(pntPerSide)):
                init = ((m * space) / np.linalg.norm(vector)) * vector + self.box_vert[i]
                final = init + self.collideDistance * self.findVerticalVector(vector, direction)
                c.append(init)
                c_.append(final)

        for i in range(len(c)):
            c[i] = self.transPoint(c[i], trans)

        for i in range(len(c_)):
            c_[i] = self.transPoint(c_[i], trans)
        return (c, c_)


    def transPoint(self, pnt, trans):
        tmp = t([pnt[0], pnt[1], 1.0])
        tmp = trans.dot(tmp)
        return t([tmp[0], tmp[1]])


    def step(self):
        c, c_ = self.sampleContactPoints()
        g = self.objectPosition
        g_ = []
        for i in range(len(c)):
            g_.append((i, G_(c[i], g, c_[i])))
        desLoc = self.desiredObjectLoc(g)
        if self.finished:
            return (None, None)
        distance = lambda x: dis(x[1], desLoc)
        g_.sort(key=distance)
        index = g_[0][0]
        self.objectPosition = g_[0][1]
        rotAngle = findRotAngle(c[index], g_[0][1], c_[index])
        self.objectOrientation += rotAngle
        return c[index], c_[index]


    def findAgentLoc(self, c, g):
        # find the desired agent location given contact point and mass center
        gc = c - g
        rg = gc * ((norm(gc) + self.agentRadius) / norm(gc))
        return rg + g


    # check whether two line segments intersect
    # refer to: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    def checkcollision(self, line1, line2):
        # find the orientation of three points
        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            # colinear
            if val == 0:
                return 0
            # clockwise or counter-clockwise
            return 1 if val > 0 else 2

        # given colinear points p,q,r, check whether q lies on line pr
        def onSegment(p, q, r):
            if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and \
                            q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]):
                return True
            return False

        p1, q1 = line1
        p2, q2 = line2
        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        # general case
        if (o1 != o2 and o3 != o4):
            return True
        # special cases
        if (o1 == 0 and onSegment(p1, p2, q1)) or \
                (o2 == 0 and onSegment(p1, q2, q1)) or \
                (o3 == 0 and onSegment(p2, p1, q2)) or \
                (o4 == 0 and onSegment(p2, q1, q2)):
            return True
        return False


    # check whether LINE collides with any edge of object
    def checkBoxCollision(self, line, rot, g):
        trans = transform(rot, g)
        vert = []
        for i in self.box_vert:
            i_0 = i[0] + 0.01 if i[0] < 0 else i[0] - 0.01
            i_1 = i[1] + 0.01 if i[1] < 0 else i[1] - 0.01
            vert.append([i_0, i_1])
        for i in range(1, len(vert)):
            edge = (self.transPoint(vert[i - 1], trans), self.transPoint(vert[i], trans))
            if self.checkcollision(edge, line):
                return True
        return False


    # generate collision free extra path point
    def genExtraPoint(self, s, e, g):
        m = (s + e) / 2
        return (m - g) * 2 + g


    def path(self, draw=False, check_collision=True):
        agent_c = []
        agent_c_ = []
        # loc before push
        g = []
        # loc after push
        g_ = []
        rot = []
        iter = 0
        points = []
        lastPoint = self.agentInitialPosition
        while True:
            g_before = self.objectPosition
            rot_before = self.objectOrientation
            g.append(g_before)
            c, c_ = self.step()
            iter += 1
            if self.finished or iter >= self.maxIter:
                break
            else:
                g_after = self.objectPosition
                g_.append(g_after)
                rot.append(self.objectOrientation)
                agent_c.append(c)
                agent_c_.append(c_)
                line = (lastPoint, c)
                if self.checkBoxCollision(line, rot_before, g_before):
                    print('--------------')
                    print(line)
                    print(g_before)
                    extra = self.genExtraPoint(lastPoint, c, g_before)
                    if check_collision:
                        points.append(extra)
                points.append(self.findAgentLoc(c, g_before))
                points.append(self.findAgentLoc(c_, g_after))
                lastPoint = c_
        agent_c = t(agent_c)
        agent_c_ = t(agent_c_)
        g = t(g)

        res_path = paths.pathGenerate(points)
        if draw:
            paths.plot_path(res_path)

        return g, paths.pathGenerate(points)

    def draw(self):
        agent_c = [t([0, 0])]
        agent_c_ = [t([0, 0])]
        g = []
        g_before = [self.objectPosition]
        rot = [0.0]
        iter = 0
        while True:
            g_before.append(self.objectPosition)
            c, c_ = self.step()
            iter += 1
            if self.finished or iter >= self.maxIter:
                break
            else:
                g.append(self.objectPosition)
                rot.append(self.objectOrientation)
                agent_c.append(c)
                agent_c_.append(c_)
        agent_c = t(agent_c)
        agent_c_ = t(agent_c_)
        g = t(g)

        # Writer = animation.writers['ffmpeg']
        # writer = Writer(fps=5, metadata=dict(artist='Me'), bitrate=1800)

        #plt.rcParams["figure.figsize"] = (5, 5)
        fig, ax = plt.subplots()
        plt.ion()
        xdata, ydata, xdata_, ydata_ = [], [], [], []
        x, y = [], []

        ax.set_xlim(-7, 5)
        ax.set_ylim(-10, 2)

        for frame in range(len(g)):
            if frame == 0:
                while len(xdata_) > 0:
                    xdata_.pop(0)
                    xdata.pop(0)
                    ydata_.pop(0)
                    ydata.pop(0)
                while len(x) > 0:
                    x.pop(0)
                    y.pop(0)
            x.append(agent_c[frame][0])
            x.append(agent_c_[frame][0])
            y.append(agent_c[frame][1])
            y.append(agent_c_[frame][1])
            xdata.append(agent_c[frame][0])
            ydata.append(agent_c[frame][1])
            xdata_.append(agent_c_[frame][0])
            ydata_.append(agent_c_[frame][1])
            trans = transform(rot[frame], g_before[frame])
            box_x, box_y = [], []
            for i in range(len(self.box_vert)):
                box_x.append(self.transPoint(self.box_vert[i], trans)[0])
                box_y.append(self.transPoint(self.box_vert[i], trans)[1])

            # Object path
            plt.plot(g[:frame, 0], g[:frame, 1], 'r-', linewidth=3)
            # Agent
            plt.plot(xdata, ydata, 'ro')
            # agent_
            plt.plot(xdata_, ydata_, 'b*', markersize=10)
            # agent_overall
            plt.plot(x, y, 'k-')
            # box
            plt.plot(box_x, box_y, 'k-')

            arrow = plt.arrow(xdata[-1], ydata[-1], xdata_[-1] - xdata[-1], ydata_[-1] - ydata[-1], head_width=0.05)
            text = plt.text(g_before[frame][0], g_before[frame][1], "G")

            ax.add_patch(arrow)
            #ax.add_patch(text)
            print(frame)
            plt.draw()
            #plt.show()
            sleep(0.5)



if __name__ == '__main__':
    pg = PathGenerator()
    pg.draw()

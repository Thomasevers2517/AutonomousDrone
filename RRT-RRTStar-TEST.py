from collections import deque
from random import random

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import collections as mc
from line_profiler_pycharm import profile

class Line():

    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn

@profile
def Intersection(line, center, radius):

    # closest = np.dot((line.p - center),(line.dirn*line.dist))
    # lineObsDist = distance(closest,center)
    # if lineObsDist<radius:
    #     return True
    # else:
    #     return False

    #Solution 1
    a = np.dot(line.dirn, line.dirn)
    b = 2 * np.dot(line.dirn, line.p - center)
    c = np.dot(line.p - center, line.p - center) - radius * radius

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return False

    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
        return False

    return True


@profile
def distance(x, y):
    dif = np.array(x) - np.array(y)
    #take 1-norm ?
    return np.linalg.norm(dif)

@profile
def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False

@profile
def isThruObstacle(v,vex, obstacles, radius):

    for obs in obstacles:

        difV = tuple((abs(obs[0] - (v[0])), abs(obs[1] - v[1])))
        difVex= tuple((abs(obs[0] - (vex[0])), abs(obs[1] - vex[1])))
        #Optimized to do less intersection checks
        if (difV[0]<stepSize*2 and difV[1]<stepSize*2) or (difVex[0]<stepSize*2 and difVex[1]<stepSize*2):
            line = Line(v, vex)
            if Intersection(line, obs, radius):
                return True
    return False

@profile
def nearest(G, vex, obstacles, radius):
    Nvex = None
    Nidx = None
    minDist = float("inf")
    for idx, v in enumerate(G.vertices):
        dif = tuple((abs(v[0]-vex[0]),abs(v[1] - vex[1])))
        if(dif[0]<minDist and dif[1]<minDist):
            dist = distance(v, vex)
            if dist < minDist:

                if isThruObstacle(v,vex, obstacles, radius):
                    continue
                else:
                    minDist = dist
                    Nidx = idx
                    Nvex = v

    return Nvex, Nidx


def newVertex(randvex, nearvex, stepSize):
    dirn = np.array(randvex) - np.array(nearvex)
    length = np.linalg.norm(dirn)
    dirn = (dirn / length) * min (stepSize, length)

    newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
    return newvex


def window(startpos, endpos):

    width = endpos[0] - startpos[0]
    height = endpos[1] - startpos[1]
    winx = startpos[0] - (width / 2.)
    winy = startpos[1] - (height / 2.)
    return winx, winy, width, height


def isInWindow(pos, winx, winy, width, height):

    if winx < pos[0] < winx+width and \
        winy < pos[1] < winy+height:
        return True
    else:
        return False


class Graph:

    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPosition(self):
        rx = random()
        ry = random()

        posx = self.startpos[0] - (self.sx / 2.) + rx * self.sx * 2
        posy = self.startpos[1] - (self.sy / 2.) + ry * self.sy * 2
        return posx, posy




@profile
def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    # RRT star algorithm

    G = Graph(startpos, endpos)
    print("Calculating")
    firstPath = n_iter
    aPathFound = False
    for _ in range(n_iter):
        if firstPath*2.5<_:
            break

        if (_ % 15 == 0):
            print(_)
        randvex = G.randomPosition()
        if isInObstacle(randvex, obstacles, radius):
            continue

        nearvex, nearidx = nearest(G, randvex, obstacles, radius)
        if nearvex is None:
            continue

        newvex = newVertex(randvex, nearvex, stepSize)

        newidx = G.add_vex(newvex)
        dist = distance(newvex, nearvex)
        G.add_edge(newidx, nearidx, dist)
        G.distances[newidx] = G.distances[nearidx] + dist

        # update nearby vertices distance (if shorter)
        for vex in G.vertices:
            if vex == newvex:
                continue


            if ((abs(vex[0]-newvex[0])>radius) or (abs(vex[0]-newvex[0])>radius)):
                continue
            else:
                dist = distance(vex, newvex)
                if dist > radius:
                    continue


            if isThruObstacle(vex, newvex, obstacles, radius):
                continue

            idx = G.vex2idx[vex]
            if G.distances[newidx] + dist < G.distances[idx]:
                G.add_edge(idx, newidx, dist)
                G.distances[idx] = G.distances[newidx] + dist
        dist = distance(newvex, G.endpos)
        if dist < stepSize:
            endidx = G.add_vex(G.endpos)
            G.add_edge(newidx, endidx, dist)
            try:
                G.distances[endidx] = min(G.distances[endidx], G.distances[newidx] + dist)
            except:
                G.distances[endidx] = G.distances[newidx] + dist
            G.success = True
            if(not aPathFound):
                firstPath = _
                aPathFound = True
            print("Found a path")

    print("Done Calculating")
    return G



def dijkstra(G):

    srcIdx = G.vex2idx[G.startpos]
    dstIdx = G.vex2idx[G.endpos]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = dstIdx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    return list(path)

def plot(G, obstacles, radius,  title, path=None):

    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()
    plt.title(title)
    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='red')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()



if __name__ == '__main__':
    startpos = (-1, -1)
    endpos = (10, 10)
    obstacles = [(1., 1.), (2., 2.)]
    for i in range(5):
        for j in range(5):
            obstacles.append((i*2, j*2))

    n_iter = 300
    radius = 0.4
    stepSize = 0.8

    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)

    if G.success:
        cur_path = dijkstra(G)
        resFactor = 5
        resPath =[]
        lastVex = cur_path[0]
        print(str(lastVex))
        for vex in cur_path:
            for i in range(resFactor - 1):
                test = ((((vex)[0]*i + (lastVex[0] * (resFactor - i)))/resFactor), (((vex)[1]*i + (lastVex[1] * (resFactor - i)))/resFactor))
                tempTup = tuple(test)
                resPath.append(tempTup)
            lastVex = vex
        plot(G, obstacles, radius,  "Original", cur_path)
        plot(G, obstacles, radius, "Resolution path", resPath )
        smoothFac = 10
        buff = []
        smoothPath = []
        buff.append(resPath[0])
        for i in range(smoothFac):
            buff.append(resPath[i+1])
            fst_element = [x[0] for x in buff]
            snd_element = [x[1] for x in buff]
            avg0 = sum(fst_element) / len(buff)
            avg1 = sum(snd_element) / len(buff)
            smoothPath.append(tuple((avg0, avg1)))

        for point in (resPath[smoothFac:]):
            buff.pop(0)
            buff.append(point)
            fst_element = [x[0] for x in buff]
            snd_element = [x[1] for x in buff]
            avg0 = sum(fst_element) / len(buff)
            avg1 = sum(snd_element) / len(buff)
            avg = tuple((avg0, avg1))
            smoothPath.append(avg)
        for i in range(smoothFac):
            buff.pop(0)
            fst_element = [x[0] for x in buff]
            snd_element = [x[1] for x in buff]
            avg0 = sum(fst_element) / len(buff)
            avg1 = sum(snd_element) / len(buff)
            avg = tuple((avg0, avg1))
            smoothPath.append(avg)
        plot(G, obstacles, radius,  "Smoother Path",smoothPath)
        cur_path = smoothPath

    else:
        plot(G, obstacles, radius)
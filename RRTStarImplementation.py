import logging
import math
import sys
import time
from collections import deque
from math import cos, sin, radians, pi
from random import random
from threading import Event

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import collections as mc

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper




URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)


def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def DrawArrow(x, y, color, angle=0):
    def rotate(pos, angle):
        cen = (5 + x, 0 + y)
        angle *= -(math.pi / 180)
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)
        ret = ((cos_theta * (pos[0] - cen[0]) - sin_theta * (pos[1] - cen[1])) + cen[0],
               (sin_theta * (pos[0] - cen[0]) + cos_theta * (pos[1] - cen[1])) + cen[1])
        return ret

    p0 = rotate((0 + x, -4 + y), angle + 90)
    p1 = rotate((0 + x, 4 + y), angle + 90)
    p2 = rotate((20 + x, 0 + y), angle + 90)

    pygame.draw.polygon(DISPLAYSURF, color, [p0, p1, p2])
def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
class Line():

    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn


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



def distance(x, y):
    dif = np.array(x) - np.array(y)
    #take 1-norm ?
    return np.linalg.norm(dif)


def isInObstacle(vex, obstacles, radius):
    for obs in obstacles:
        if distance(obs, vex) < radius:
            return True
    return False


def isThruObstacle(v,vex, obstacles, radius):
    stepSize = 1
    for obs in obstacles:

        difV = tuple((abs(obs[0] - (v[0])), abs(obs[1] - v[1])))
        difVex= tuple((abs(obs[0] - (vex[0])), abs(obs[1] - vex[1])))
        if (difV[0]<stepSize*2 and difV[1]<stepSize*2) or (difVex[0]<stepSize*2 and difVex[1]<stepSize*2):
            line = Line(v, vex)
            if Intersection(line, obs, radius):
                return True
    return False


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



def RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize):
    # RRT star algorithm

    G = Graph(startpos, endpos)
    print("Calculating")
    firstPath = n_iter
    aPathFound = False
    for _ in range(n_iter):
        if firstPath*1.5<_:
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
    # Dijkstra algorithm for finding shortest path from start position to end.

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



import keyboard  # using module keyboardq

spd = 0.2
xoff = 0
yoff = 0
scalef = 100

goal = [0, 0]
goal_active = False
cur_path = []
path_cache = False
calculating = False
trustData = False
radius = 0.25
objectTol = 0.25


def key_control(scf):
    global trustData, calculating,xoff, yoff, spd, scalef, goal, goal_active, cur_path, path_cache
    Ypos = 0
    Xpos = 0
    Xvel = 0
    Yvel = 0
    n_iter = 40
    stepSize = 0.30

    yaw_rate = 0 #rad/s  startLinearMotion can not handle a yaw rate

    turned =0
    explore = False
    orient = False
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(2)
        trustData = True
        while 1:
            print("Goal active: ", goal_active, "P chache: ", path_cache, "Explore: ", explore, "orient: ", orient)
            if keyboard.is_pressed('esc'):
                break
            if keyboard.is_pressed('w'):
                mc.forward(0.06, spd)
            if keyboard.is_pressed('s'):
                mc.back(0.06, spd)
            if keyboard.is_pressed('a'):
                mc.left(0.06, spd)
            if keyboard.is_pressed('d'):
                mc.right(0.06, spd)

            if keyboard.is_pressed('q'):
                mc.turn_left(10)
            if keyboard.is_pressed('e'):
                mc.turn_right(10)

            if keyboard.is_pressed('p'):
                walls.clear()

            DISPLAYSURF.fill((0, 0, 0))

            if keyboard.is_pressed('up'):
                xoff += 0.03
            if keyboard.is_pressed('down'):
                xoff -= 0.03
            if keyboard.is_pressed('left'):
                yoff += 0.03
            if keyboard.is_pressed('right'):
                yoff -= 0.03

            if keyboard.is_pressed('r'):
                xoff = 0
                yoff = 0
                scalef = 100

            if keyboard.is_pressed('['):
                scalef -= 1
            if keyboard.is_pressed(']'):
                scalef += 1

            if keyboard.is_pressed('1'):
                spd = 0.2
            if keyboard.is_pressed('2'):
                spd = 0.4
            if keyboard.is_pressed('3'):
                spd = 0.6
            if keyboard.is_pressed('4'):
                spd = 0.8
            if keyboard.is_pressed('5'):
                spd = 1.0
            if keyboard.is_pressed('6'):
                spd = 1.5
            if keyboard.is_pressed('7'):
                spd = 2.0
            if keyboard.is_pressed('8'):
                spd = 3.0

            for x in walls:
                pygame.draw.circle(DISPLAYSURF, (0, 0, 255),
                                   (int((-x[1] + yoff) * scalef + 300), int((-x[0] + xoff) * scalef + 300)), 5)
            # plot position of drone
            DrawArrow(int((-position_estimate[1] + yoff) * scalef + 300),
                      int((-position_estimate[0] + xoff) * scalef + 300), (255, 0, 0), math.degrees(angle))
            avoid = 0

            if wall_dists[0] < radius*1.5: #Consider velocity drone
                avoid = 1
            elif wall_dists[1] < radius*1.5:
                avoid =2
            elif wall_dists[2] < radius*1.5:
                avoid =3
            elif explore:

                turned = turned +1
                if turned<=9:
                    mc.turn_left(10, 90)
                elif turned > 18:
                    explore = False
                    orient = True
                    turned = 0;
                elif turned>9:
                    mc.turn_right(10,90)

            elif orient and goal_active and path_cache:
                goaly = (cur_path[0][1] - position_estimate[1])
                goalx = (cur_path[0][0] - position_estimate[0])
                goal_angle = math.atan2(goaly, goalx)  # radians
                toTurn = (goal_angle - angle) % (2 * pi)  # ((angle%(2*pi)-goal_angle%(2*pi))%(2*pi))-pi
                if (toTurn >= pi):
                    toTurn = toTurn - 2 * pi
                toTurnRate = toTurn
                print("To turn: ", toTurn)

                DrawArrow(int((-position_estimate[1] + yoff) * scalef + 300),
                          int((-position_estimate[0] + xoff) * scalef + 300), (100, 0, 100), math.degrees(goal_angle))
                mc.start_linear_motion(0, 0, 0, -toTurnRate * 360 / (2 * pi))  # Change YAW
                time.sleep(0.01)

                if abs(toTurn)<0.05*pi:
                    orient = False
                    print("Epic orientation")
            else:
                orient = False

            if goal_active and path_cache:
                print(cur_path)
                for i in range(len(cur_path)-1):
                    if dist(position_estimate,cur_path[len(cur_path)-1 -i]) < radius:
                        for j in range(len(cur_path)-1 -i):
                            cur_path.pop(0)

                        if len(cur_path)==1:
                             print("reached goal")
                             goal_active = False;
                             mc.stop()
                             time.sleep(1)
                        break
                        # Draw position of goal
                pygame.draw.circle(DISPLAYSURF, (0, 255, 0),(int((-goal[1] + yoff) * scalef + 300), int((-goal[0] + xoff) * scalef + 300)), 5)
                for i in cur_path:
                    pygame.draw.circle(DISPLAYSURF, (0, 255, 255), (
                        int((-i[1] + yoff) * scalef + 300), int((-i[0] + xoff) * scalef + 300)), 2)
                DrawArrow(int((-position_estimate[1] + yoff) * scalef + 300),
                          int((-position_estimate[0] + xoff) * scalef + 300), (100, 0, 100), math.degrees(goal_angle))
            if goal_active and avoid==0 and not explore and (not orient or not path_cache):
                if not path_cache:
                    x_start, y_start = position_estimate[0], position_estimate[1]
                    x_end, y_end = goal[0], goal[1]

                    startpos = (x_start, y_start)
                    endpos = (x_end, y_end)
                    obstacles = walls
                    calculating = True
                    #time.sleep(1)
                    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
                    calculating = False
                    if G.success:
                        sol = dijkstra(G)
                        print("New path:")

                        print(sol)
                        cur_path = sol
                        path_cache = True
                        orient = True
                        resFactor = 5
                        resPath = []
                        lastVex = cur_path[0]

                        for vex in cur_path:
                            for i in range(resFactor - 1):
                                test = ((((vex)[0] * i + (lastVex[0] * (resFactor - i))) / resFactor),
                                        (((vex)[1] * i + (lastVex[1] * (resFactor - i))) / resFactor))
                                tempTup = tuple(test)
                                resPath.append(tempTup)
                            lastVex = vex
                        plot(G, obstacles, radius, "Original", cur_path)
                        plot(G, obstacles, radius, "Resolution path", resPath)
                        smoothFac = 10
                        buff = []
                        smoothPath = []
                        buff.append(resPath[0])
                        for i in range(smoothFac):
                            buff.append(resPath[i + 1])
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
                        plot(G, obstacles, radius, "Smoother Path", smoothPath)
                        cur_path = smoothPath
                    else:
                        print("no path found")
                        goal_active = False
                        path_cache = False
                        toTurnInt = 0

            #ADD SOMETHING THAT RECALCULATES AFTER HAVING RETRIED MANY TIMES
            if ((avoid == 0) and goal_active and not explore and not orient):
                prevYpos = Ypos
                prevXpos = Xpos
                prevXvel = Xvel
                prevYvel = Yvel

                Ypos = position_estimate[1]
                Xpos = position_estimate[0]
                goaly = (cur_path[0][1] - Ypos)
                goalx = (cur_path[0][0] - Xpos)
                Xvel = Xpos - prevXpos
                Yvel = Ypos - prevYpos
                Xvel_want = spd * goalx * 2.5
                Yvel_want = spd * goaly * 2.5
                #Cancel out momentum by using

                goaly = (cur_path[0][1] - position_estimate[1])
                goalx = (cur_path[0][0] - position_estimate[0])
                goal_angle = math.atan2(goaly,goalx) #radians
                toTurnProp = (goal_angle - angle)%(2*pi)      #((angle%(2*pi)-goal_angle%(2*pi))%(2*pi))-pi
                if(toTurnProp>=pi):
                    toTurnProp = toTurnProp-2*pi

                toTurnInt += toTurnProp
                toTurn = toTurnProp
                toTurnRate = toTurn
                print("To turn: ", toTurn)


                mc.start_linear_motion(spd, 0, 0, -toTurnRate*360/(2*pi))  # Change YAW
                time.sleep(0.01)

            elif avoid == 1:
                mc.back(0.10,spd)
                time.sleep(0.5)
                path_cache = False
                toTurnInt = 0
                explore = True
                print("Close to object")
            elif avoid == 2:
                mc.right(0.10,spd)
                time.sleep(0.5)
                path_cache = False
                toTurnInt = 0
                explore = True
                print("Close to object")
            elif avoid == 3:
                mc.left(0.10,spd)
                time.sleep(0.5)
                path_cache = False
                toTurnInt = 0
                explore = True
                print("Close to object")

            pygame.display.update()
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()

                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    pos = (-(pos[1] - 300) / scalef + xoff, -(pos[0] - 300) / scalef + yoff)
                    goal = pos
                    goal_active = True
                    path_cache = False
                    toTurnInt = 0
                    cur_path = []
                    if(dist(position_estimate,goal)<2):
                        n_iter = 200
                        stepSize = radius*0.75
                    else:
                        n_iter = 300
                        stepSize = radius*1.25

                    print(goal)
                    print(position_estimate)


position_estimate = [0, 0]
walls = []
angle = 0
wall_dists = [9999, 9999, 9999]


def log_pos_callback(timestamp, data, logconf):
    # print(data)
    if(not calculating):
        global position_estimate, angle, wall_dists, walls
        position_estimate[0] = data['stateEstimate.x']
        position_estimate[1] = data['stateEstimate.y']
        wall_dists = [data['range.front']/1000, data['range.left']/1000, data['range.right']/1000]
        angle = radians(data['stateEstimate.yaw'])
        # Below potentially include  roll/pitch affect on range estimation
        # Yaw angle already included in point cloud, full yaw rate is easy addition

        # For front, left and right sensor, add measurements to point cloud. (if radius cm from other datapoint)
        if trustData:
            wall = [data['stateEstimate.x'] + cos(angle) * data['range.front'] / 1000,
                    data['stateEstimate.y'] + sin(angle) * data['range.front'] / 1000]
            if not any(dist(wall, i) < radius*0.5 for i in walls):
                walls.append(wall)
            walls = [i for i in walls if dist(i, position_estimate) > data['range.front'] / 1000 - 0.1 or abs(
                math.atan2(i[1] - position_estimate[1], i[0] - position_estimate[0]) - angle) > 0.05]
            wall = [data['stateEstimate.x'] + cos(angle + pi / 2) * data['range.left'] / 1000,
                    data['stateEstimate.y'] + sin(angle + pi / 2) * data['range.left'] / 1000]
            if not any(dist(wall, i) < radius*0.5 for i in walls):
                walls.append(wall)
            wall = [data['stateEstimate.x'] + cos(angle - pi / 2) * data['range.right'] / 1000,
                    data['stateEstimate.y'] + sin(angle - pi / 2) * data['range.right'] / 1000]
            if not any(dist(wall, i) < radius*0.5 for i in walls):
                walls.append(wall)
            # if len(walls)>10000:
            #     walls.pop(0)

            # position_estimate[0] = data['range.front']



if __name__ == '__main__':
    cflib.crtp.init_drivers()

    import pygame
    from pygame.locals import *

    pygame.init()

    DISPLAYSURF = pygame.display.set_mode((600, 600))

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='data', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.yaw', 'float')
        logconf.add_variable('range.front', 'float')
        # logconf.add_variable('range.back', 'float')
        logconf.add_variable('range.left', 'float')
        logconf.add_variable('range.right', 'float')

        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf.start()
        key_control(scf)
        logconf.stop()

    print('Done!')
#


"""
Motors:

A   ^   B

B       A
"""

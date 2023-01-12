import logging
import math
import sys
import time
from math import cos, sin, radians, pi
from random import randint
from threading import Event

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

def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(2)
        mc.stop()


def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')



def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)
        mc.turn_left(180)
        time.sleep(1)
        mc.forward(0.5)
        time.sleep(1)


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
    p2 = rotate((10 + x, 0 + y), angle + 90)

    pygame.draw.polygon(DISPLAYSURF, color, [p0, p1, p2])


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def dist(a, b):
    return math.sqrt((a[0]-b[0]) ** 2 + (a[1]-b[1]) ** 2)


def astar(start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # # Make sure within range
            # if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
            #     continue
            #
            # # Make sure walkable terrain
            # if maze[node_position[0]][node_position[1]] != 0:
            #     continue

            if any(dist(node_position, (i[0]*10,i[1]*10)) < 3 for i in walls):
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)



import keyboard  # using module keyboardq

spd = 0.4
xoff = 0
yoff = 0
scalef = 100

goal = [0,0]
goal_active = False
cur_path = []
path_cache = False

def key_control(scf):
    global xoff, yoff, spd, scalef, goal, goal_active, cur_path, path_cache
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        while 1:
            if keyboard.is_pressed('esc'):
                break
            if keyboard.is_pressed('w'):
                mc.forward(0.06,spd)
            if keyboard.is_pressed('s'):
                mc.back(0.06,spd)
            if keyboard.is_pressed('a'):
                mc.left(0.06,spd)
            if keyboard.is_pressed('d'):
                mc.right(0.06,spd)

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


            # for wall in walls:
            # if len(walls) > 1:
            #     xmin,xmax = min(x[0] for x in walls),max(x[0] for x in walls+[position_estimate])
            #     ymin,ymax = min(x[1] for x in walls),max(x[1] for x in walls+[position_estimate])
            #     xdiff = xmax-xmin
            #     ydiff = ymax-ymin
            #     scalef = min(300/(xdiff or 1),300/(ydiff or 1))
            #     # print(scalef)
            #
            #     xoff = xmax-xdiff/2
            #     yoff = ymax-ydiff/2


            # print(xoff,yoff)


            for x in walls:
                pygame.draw.circle(DISPLAYSURF, (0, 0, 255),
                                   (int((-x[1]+yoff) * scalef + 300), int((-x[0]+xoff) * scalef + 300)), 5)

            # pygame.draw.circle(DISPLAYSURF, (255, 0, 0), (int((-position_estimate[1]+yoff)*scalef+300), int((-position_estimate[0]+xoff)*scalef+300)), 5)


            DrawArrow(int((-position_estimate[1]+yoff)*scalef+300), int((-position_estimate[0]+xoff)*scalef+300), (255, 0, 0), math.degrees(angle))



            if wall_dists[0] < 200:
                mc.back(0.07,spd)
            elif wall_dists[1] < 200:
                mc.right(0.07,spd)
            elif wall_dists[2] < 200:
                mc.left(0.07,spd)
            elif goal_active:
                pygame.draw.circle(DISPLAYSURF, (0, 255, 0), (int((-goal[1]+yoff)*scalef+300), int((-goal[0]+xoff)*scalef+300)), 5)

                if path_cache:
                    if dist(position_estimate,cur_path[0]) < 0.5:
                        for wall in walls:
                            for p in cur_path:
                                if dist(p,wall) < 0.2:
                                    path_cache = False
                                    break
                            else:
                                continue
                            break
                        else:
                            if dist(position_estimate,cur_path[0]) < 0.15:
                                cur_path.pop(0)
                    else:
                        path_cache = False

                if path_cache:
                    if len(cur_path) > 1:
                        for i in cur_path:
                            pygame.draw.circle(DISPLAYSURF, (0, 255, 255), (
                            int((-i[1] / 10 + yoff) * scalef + 300), int((-i[0] / 10 + xoff) * scalef + 300)), 2)
                        goal_angle = math.atan2(cur_path[1][1]/10-cur_path[0][1]/10,cur_path[1][0]/10-cur_path[0][0]/10)
                    else:
                        print("done")
                        goal_active = False
                else:

                    x_start, y_start = round(position_estimate[0]*10), round(position_estimate[1]*10)
                    x_end, y_end = round(goal[0]*10), round(goal[1]*10)
                    sol = astar((x_start, y_start), (x_end, y_end))
                    cur_path = sol
                    path_cache = True
                    if sol and len(sol)>1:
                        for i in sol:
                            pygame.draw.circle(DISPLAYSURF, (0, 255, 255), (int((-i[1]/10+yoff)*scalef+300), int((-i[0]/10+xoff)*scalef+300)), 2)

                        goal_angle = math.atan2(sol[1][1]/10-sol[0][1]/10,sol[1][0]/10-sol[0][0]/10)
                        nxt = goal
                    else:
                        print("done")
                        goal_active = False

                if goal_active:
                    print(path_cache,cur_path)
                    if abs(goal_angle-angle) > 0.2:
                        if abs(goal_angle-angle) > math.pi:
                            if goal_angle > angle:
                                mc.turn_right(10)
                            else:
                                mc.turn_left(10)
                        else:
                            if goal_angle > angle:
                                mc.turn_left(10)
                            else:
                                mc.turn_right(10)
                    else:
                        mc.forward(0.03,spd)



                    # if sol[1][0] == x_start+1:
                    #     mc.forward(0.03,spd)
                    # elif sol[1][0] == x_start-1:
                    #     mc.back(0.03,spd)
                    # elif sol[1][1] == y_start+1:
                    #     mc.left(0.03,spd)
                    # elif sol[1][1] == y_start-1:
                    #     mc.right(0.03,spd)


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
                    cur_path = []
                    print(pos)
                    print(position_estimate)

position_estimate = [0, 0]
walls = []
angle = 0
wall_dists = [9999,9999,9999]


def log_pos_callback(timestamp, data, logconf):
    # print(data)
    global position_estimate, angle, wall_dists, walls
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']
    wall_dists = [data['range.front'], data['range.left'], data['range.right']]
    angle = radians(data['stateEstimate.yaw'])
    #Below potentially include  roll/pitch affect on range estimation
    #Yaw angle already included in point cloud, full yaw rate is easy addition

    #For front, left and right sensor, add measurements to point cloud. (if 5 cm from other datapoint)
    wall = [data['stateEstimate.x'] + cos(angle)*data['range.front']/1000,data['stateEstimate.y'] + sin(angle)*data['range.front']/1000]
    if not any(dist(wall, i) < 0.05 for i in walls):
        walls.append(wall)
    walls = [i for i in walls if dist(i, position_estimate) > data['range.front']/1000-0.1 or abs(math.atan2(i[1]-position_estimate[1],i[0]-position_estimate[0]) - angle) > 0.05]
    wall = [data['stateEstimate.x'] + cos(angle+pi/2)*data['range.left']/1000,data['stateEstimate.y'] + sin(angle+pi/2)*data['range.left']/1000]
    if not any(dist(wall, i) < 0.05 for i in walls):
        walls.append(wall)
    wall = [data['stateEstimate.x'] + cos(angle-pi/2)*data['range.right']/1000,data['stateEstimate.y'] + sin(angle-pi/2)*data['range.right']/1000]
    if not any(dist(wall, i) < 0.05 for i in walls):
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

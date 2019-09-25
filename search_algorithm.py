#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
import time
import math


rospy.init_node("search_algorithms")
publisher = rospy.Publisher("/actions", String, queue_size=10)
parser = argparse.ArgumentParser()
parser.add_argument('-a', help="Please mention algorithm to use. Possible arguments = {bfs, ucs, gbfs, astar}. Default value is bfs.", metavar='bfs', action='store', dest='algorithm', default="bfs", type=str)
parser.add_argument('-c', help="Use custom heuristic function. No value needed.", action='store_true', dest='custom_heuristic')

def helper_bfs(init_state, goal_state):

    helper = problem.Helper()
    state_dictionary = helper.get_successor(init_state)
    visited = []
    visited.append([init_state.x, init_state.y, init_state.orientation])
    queue = list()
    init_cost = 1

    for key in state_dictionary:
        l = list(key.split(","))
        queue.append([l, state_dictionary[key][0], init_cost])
    cost = float('inf')
    while (queue):

        node = queue.pop(0)
        if node[1].x < 0 or node[1].y < 0:
            continue
        else:
            if [node[1].x, node[1].y, node[1].orientation] not in visited:
                if node[1].x == goal_state.x and node[1].y == goal_state.y:
                    cost = node[2]
                    break
                else:
                    visited.append([node[1].x, node[1].y, node[1].orientation])
                    successor = helper.get_successor(node[1])

                    for key in successor:
                        l1 = list(key.split(","))
                        queue.append([node[0] + l1, successor[key][0], node[2] + 1])
    # mem.update({list(init_state) : cost})
    #print(type(init_state))
    d[init_state_tuple] = cost
    return cost

def bfs(use_custom_heuristic):
    '''
    Perform BFS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)

    '''
    YOUR CODE HERE
    '''
    if init_state.x != goal_state.x and init_state.y != goal_state.y:
        visited = []
        visited.append([init_state.x, init_state.y, init_state.orientation])
        queue = list()

        for key in state_dictionary:
            l = list(key.split(","))
            queue.append([l,state_dictionary[key][0]])

        while(queue):

            node = queue.pop(0)
            if node[1].x < 0 or node[1].y < 0:
                continue
            else:
                if [node[1].x, node[1].y, node[1].orientation] not in visited:
                    if node[1].x == goal_state.x and node[1].y == goal_state.y:
                        action_list = action_list + node[0]
                        break
                    else:
                        visited.append([node[1].x,node[1].y,node[1].orientation])
                        successor = helper.get_successor(node[1])
                        for key in successor:
                            l1 = list(key.split(","))
                            queue.append([node[0]+l1,successor[key][0]])

    return action_list


def ucs(use_custom_heuristic):
    '''
    Perform UCS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)

    '''
    YOUR CODE HERE
    '''
    if init_state.x != goal_state.x and init_state.y != goal_state.y:

        visited = []
        queue = list()
        visited.append([init_state.x, init_state.y, init_state.orientation])

        for key in state_dictionary:
            l = list(key.split(","))
            queue.append([l, state_dictionary[key][0], state_dictionary[key][1]])

        while(queue):

            queue.sort(key = lambda ele: ele[2])
            node = queue.pop(0)
            if node[1].x < 0 or node[1].y < 0:
                continue
            else:
                if [node[1].x, node[1].y, node[1].orientation] not in visited:
                    if node[1].x == goal_state.x and node[1].y == goal_state.y:
                        action_list = action_list + node[0]
                        break
                    else:
                        visited.append([node[1].x,node[1].y,node[1].orientation])
                        successor = helper.get_successor(node[1])
                        # print("Move: ", node[0])
                        # print("Successor: ",successor)
                        for key in successor:
                            l1 = list(key.split(","))
                            val = successor[key][1]
                            queue.append([node[0]+l1,successor[key][0],node[2]+val])
    return action_list
    # visited = []
    # queue = list()
    # visited.append([init_state.x, init_state.y, init_state.orientation])
    #
    # c1 = 0
    # for key in state_dictionary:
    #     l = list(key.split(","))
    #     heapq.heappush(queue, ([state_dictionary[key][1], c1, (l, state_dictionary[key][0])]))
    #     c1 = c1 + 1
    #
    # while (queue):
    #     node = heapq.heappop(queue)
    #     if node[2][1].x < 0 or node[2][1].y < 0:
    #         continue
    #     else:
    #         if [node[2][1].x, node[2][1].y, node[2][1].orientation] not in visited:
    #             if node[2][1].x == goal_state.x and node[2][1].y == goal_state.y and node[2][1].orientation == goal_state.orientation:
    #                 action_list = action_list + node[2][0]
    #                 print("Cost: ", node[0])
    #                 break
    #             else:
    #                 visited.append([node[2][1].x,node[2][1].y,node[2][1].orientation])
    #                 successor = helper.get_successor(node[2][1])
    #
    #                 # print("Move: ", node[0])
    #                 # print("Successor: ",successor)
    #                 c2 = 0
    #                 for key in successor:
    #                     l1 = list(key.split(","))
    #                     val = successor[key][1]
    #                     heapq.heappush(queue, ([node[0]+val, c2, (node[2][0]+l1, successor[key][0])]))
    #                     c2 = c2 + 1



def gbfs(use_custom_heuristic):
    '''
    Perform GBFS to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)

    '''
    YOUR CODE HERE
    '''
    if use_custom_heuristic:
        if init_state.x != goal_state.x and init_state.y != goal_state.y:

            visited = []
            init_dir = []
            init_cost = 0
            queue = list()
            visited.append([init_state.x, init_state.y, init_state.orientation])
            array = gbfs_custom_heuristic(state_dictionary, goal_state, init_dir, init_cost)
            queue = queue + array

            while (queue):

                queue.sort(key = lambda ele: ele[3])
                node = queue.pop(0)
                if node[1].x < 0 or node[1].y < 0:
                    continue
                else:
                    if [node[1].x, node[1].y, node[1].orientation] not in visited:
                        if node[1].x == goal_state.x and node[1].y == goal_state.y:
                            action_list = action_list + node[0]
                            break
                        else:
                            visited.append([node[1].x, node[1].y, node[1].orientation])
                            successor = helper.get_successor(node[1])
                            out_arr = gbfs_custom_heuristic(successor, goal_state, node[0], node[2])
                            queue = queue + out_arr

        return action_list

    else:
        if init_state.x != goal_state.x and init_state.y != goal_state.y:

            visited = []
            init_dir = []
            init_cost = 0
            queue = list()
            visited.append([init_state.x, init_state.y, init_state.orientation])
            array = gbfs_manhattan_heuristic(state_dictionary, goal_state, init_dir, init_cost)
            queue = queue + array

            while (queue):

                queue.sort(key  = lambda ele: ele[3])
                node = queue.pop(0)
                if node[1].x < 0 or node[1].y < 0:
                    continue
                else:

                    if [node[1].x, node[1].y, node[1].orientation] not in visited:
                        if node[1].x == goal_state.x and node[1].y == goal_state.y:
                            action_list = action_list + node[0]
                            break
                        else:
                            visited.append([node[1].x, node[1].y, node[1].orientation])
                            successor = helper.get_successor(node[1])
                            out_arr = gbfs_manhattan_heuristic(successor, goal_state, node[0], node[2])
                            queue = queue + out_arr

        return action_list


def astar(use_custom_heuristic):
    '''
    Perform A* search to find sequence of actions from initial state to goal state
    '''
    helper = problem.Helper()
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()
    possible_actions = helper.get_actions() 
    action_list = []

    # to get the possible action->(state,cost) mapping from current state
    state_dictionary = helper.get_successor(init_state)

    '''
    YOUR CODE HERE
    '''
    if use_custom_heuristic:
        if init_state.x != goal_state.x and init_state.y != goal_state.y:

            visited = []
            init_dir = []
            init_cost = 0
            queue = list()
            # heuristic = 0.5
            # start_to_goal_man_dis = abs(init_state.x - goal_state.x) + abs(init_state.y - goal_state.y)
            visited.append([init_state.x, init_state.y, init_state.orientation])

            array = astar_custom_heuristic(state_dictionary, goal_state, init_dir, init_cost, init_state, visited)
            queue = queue + array

            while (queue):

                queue.sort(key=lambda ele: ele[3])
                node = queue.pop(0)
                if node[1].x < 0 or node[1].y < 0:
                    continue
                else:
                    if [node[1].x, node[1].y, node[1].orientation] not in visited:
                        if node[1].x == goal_state.x and node[1].y == goal_state.y:
                            action_list = action_list + node[0]
                            break
                        else:
                            visited.append([node[1].x, node[1].y, node[1].orientation])
                            successor = helper.get_successor(node[1])
                            out_arr = astar_custom_heuristic(successor, goal_state, node[0], node[2], init_state, visited)
                            queue = queue + out_arr


        return action_list

    else:
        if init_state.x != goal_state.x and init_state.y != goal_state.y:

            visited = []
            init_dir = []
            init_cost = 0
            queue = list()
            visited.append([init_state.x, init_state.y, init_state.orientation])
            array = astar_manhattan_heuristic(state_dictionary, goal_state, init_dir, init_cost, visited)
            queue = queue + array

            while (queue):

                queue.sort(key=lambda ele: ele[3])
                node = queue.pop(0)
                if node[1].x < 0 or node[1].y < 0:
                    continue
                else:
                    if [node[1].x, node[1].y, node[1].orientation] not in visited:
                        if node[1].x == goal_state.x and node[1].y == goal_state.y:
                            action_list = action_list + node[0]
                            break
                        else:
                            visited.append([node[1].x, node[1].y, node[1].orientation])
                            successor = helper.get_successor(node[1])
                            out_arr = astar_manhattan_heuristic(successor, goal_state, node[0], node[2], visited)
                            queue = queue + out_arr

        return action_list


def exec_action_list(action_list):
    '''
    publishes the list of actions to the publisher topic
    action_list: list of actions to execute
    '''
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

def gbfs_custom_heuristic(dictionary, goal_state, prev_dir, cost_to_reach_node):

    temp = []
    for key in dictionary:
        if dictionary[key][0].x < 0 or dictionary[key][0].y < 0:
            continue
        else:

            # distance = helper_bfs(dictionary[key][0], goal_state)
            state = (dictionary[key][0])
            if state.orientation == 'EAST' or state.orientation == 'NORTH':
                    distance = abs(dictionary[key][0].x - goal_state.x) + abs(dictionary[key][0].y - goal_state.y)
                    distance = distance - (distance) / 3
            else:
                distance = abs(dictionary[key][0].x - goal_state.x) + abs(dictionary[key][0].y - goal_state.y)
                distance = distance + (distance) / 3
            g = dictionary[key][1] + cost_to_reach_node
            dir = list(key.split(","))
            temp.append([prev_dir+dir, state, g, distance])

    return temp

def gbfs_manhattan_heuristic(dictionary, goal_state, prev_dir, cost_to_reach_node):

    temp = []
    for key in dictionary:
        if dictionary[key][0].x < 0 or dictionary[key][0].y < 0:
            continue
        else:
            distance = abs(dictionary[key][0].x - goal_state.x) + abs(dictionary[key][0].y - goal_state.y)
            g = dictionary[key][1] + cost_to_reach_node
            dir = list(key.split(","))
            state = (dictionary[key][0])
            temp.append([prev_dir+dir, state, g, distance])

    return temp

def astar_custom_heuristic(dictionary, goal_state, prev_dir, cost_to_reach_node, init_state, visited):

    temp = []

    for key in dictionary:
        if dictionary[key][0].x < 0 or dictionary[key][0].y < 0:
            continue
        else:
            if [dictionary[key][0].x, dictionary[key][0].y, dictionary[key][0].orientation] not in visited:
                #h1 = math.sqrt(((dictionary[key][0].x - goal_state.x)**2) + ((dictionary[key][0].y - goal_state.y)**2))
                #h2 = math.sqrt(((dictionary[key][0].x - init_state.x)**2) + ((dictionary[key][0].y - init_state.y)**2))
                #h2 = abs(dictionary[key][0].x - goal_state.x) + abs(dictionary[key][0].y - goal_state.y)
                #h = max(h1,h2)
                #h = helper_bfs(dictionary[key][0], goal_state)

                state = (dictionary[key][0])
                if state.orientation == 'EAST' or state.orientation == 'NORTH':
                    if state.x >= 0 and state.y >= 0:
                        h = abs(dictionary[key][0].x - goal_state.x) + abs(dictionary[key][0].y - goal_state.y)
                        h = h - h/3
                else:
                    h = abs(dictionary[key][0].x - goal_state.x) + abs(dictionary[key][0].y - goal_state.y)
                    h = h + h/3
                g = dictionary[key][1] + cost_to_reach_node

                total_distance = h + g
                dir = list(key.split(","))
                temp.append([prev_dir+dir, state, g, total_distance])


    return temp

def astar_manhattan_heuristic(dictionary, goal_state, prev_dir, cost_to_reach_node, visited):

    temp = []
    for key in dictionary:
        if dictionary[key][0].x < 0 or dictionary[key][0].y < 0:
            continue
        else:
            if [dictionary[key][0].x, dictionary[key][0].y, dictionary[key][0].orientation] not in visited:
                h = abs(dictionary[key][0].x - goal_state.x) + abs(dictionary[key][0].y - goal_state.y)
                g = dictionary[key][1] + cost_to_reach_node
                total_distance = h + g
                dir = list(key.split(","))
                state = (dictionary[key][0])
                temp.append([prev_dir+dir, state, g, total_distance])

    return temp

if __name__ == "__main__":
    # DO NOT MODIFY BELOW CODE
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    if args.algorithm in ["bfs", "ucs"] and args.custom_heuristic == True:
        print ("Error: "+args.algorithm+" called with heuristic")
        exit(1)

    start_time = time.time()
    actions = algorithm(args.custom_heuristic)
    time_taken = time.time() - start_time
    print("Time Taken = " + str(time_taken))
    print("Plan = " + str(actions))
    exec_action_list(actions)

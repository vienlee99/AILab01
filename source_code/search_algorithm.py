import pygame
import graphUI
import time
import math
from node_color import white, yellow, black, red, blue, purple, orange, green, grey

"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""


def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    # TODO: your code
    print("Implement BFS algorithm.")

    graph[start][3] = red
    graphUI.updateUI()
    
    # Saving node was discovered 
    disNodeList = [start]

    # Saving node was completed
    compNodeList = []

    # Saving previous node
    preNode = {}

    newDisNodeList = []

    while True:
        for i in disNodeList:

            graph[i][3] = yellow
            graphUI.updateUI()

            # Discovering adjacent nodes
            for j in graph[i][1]:
                if j not in compNodeList + disNodeList + newDisNodeList:
                    newDisNodeList.append(j)
                    preNode[j] = i
                    graph[j][3] = red
                    edges[edge_id(i, j)][1] = white

            graph[i][3] = blue
            compNodeList.append(i)
            graphUI.updateUI()

            # Filling goal, start node and shortest path
            if i == goal:
                graph[i][3] = purple
                graphUI.updateUI()
                des = goal

                while des != start:
                    pre = preNode[des]
                    edges[edge_id(pre, des)][1] = green
                    des = pre
                    graphUI.updateUI()

                graph[start][3] = orange
                graphUI.updateUI()
                return
        time.sleep(3)
        disNodeList = newDisNodeList
        newDisNodeList = []
    pass


def DFS(graph, edges, edge_id, start, goal):
    """
    DFS search
    """
    # TODO: your code
    print("Implement DFS algorithm.")

    graph[start][3] = red
    graphUI.updateUI()
    time.sleep(1)

    DfsRecursion(graph, edges, edge_id, start, goal)
    
    graph[start][3] = orange
    graphUI.updateUI()

    pass

# DFS agorithm by recursion
def DfsRecursion(graph, edges, edge_id, start, goal):
    if start == goal:
        graph[goal][3] = purple
        graphUI.updateUI()
        return True

    graph[start][3] = yellow
    graphUI.updateUI()
    time.sleep(1)

    disNodeList = graph[start][1]
    for i in disNodeList:
        if graph[i][3] != blue:
           graph[i][3] = red
    graphUI.updateUI()
    time.sleep(1)

    graph[start][3] = blue
    graphUI.updateUI()
    time.sleep(1)

    for i in disNodeList:
        if graph[i][3] != blue:
           
            des = DfsRecursion(graph, edges, edge_id, i, goal)
            if des:
                edges[edge_id(start, i)][1] = green
                time.sleep(1)
                graphUI.updateUI()
                return True

    pass

def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    # TODO: your code
    print("Implement Uniform Cost Search algorithm.")

    graph[start][3] = red
    graphUI.updateUI()
    time.sleep(1)

    # Start node has weight==0
    node = Node(weight=0)

    # Saving node was discovered 
    disNodeList = {start: node}

     # Saving node was completed
    compNodeList = {}

    graph[start][3] = yellow
    graphUI.updateUI()

    while True:

        i = FindMinWeight(disNodeList)

        if i == goal:
            graph[i][3] = purple
            graphUI.updateUI()
            des = goal
            compNodeList[des] = disNodeList[des]
            while des != start:
                pre = compNodeList[des].preNode
                edges[edge_id(pre, des)][1] = green
                des = pre
                graphUI.updateUI()
            graph[start][3] = orange
            graphUI.updateUI()
            return

        graph[i][3] = yellow
        graphUI.updateUI()
        time.sleep(1)

        for j in graph[i][1]:
            if j not in compNodeList.keys():
                jWeight = math.sqrt((graph[i][0][0]-graph[j][0][0])**2+(graph[i][0][1]-graph[j][0][1])**2) + disNodeList[i].weight
                

                if j in disNodeList.keys():
                    if jWeight > disNodeList[j].weight:
                        continue
                    edges[edge_id(disNodeList[j].preNode, j)][1] = grey

                disNodeList[j] = Node(weight=jWeight, preNode=i)
                graph[j][3] = red
                edges[edge_id(i, j)][1] = white


        graph[i][3] = blue
        compNodeList[i] = disNodeList[i]
        disNodeList.pop(i)
        graphUI.updateUI()
        time.sleep(1)

    pass

class Node:
    def __init__(self, weight=0, heuristic=0, preNode=None):

        # Weight of Node
        self.weight = weight

        # Heuristic of Node
        self.heuristic = heuristic

        # Previous Node
        self.preNode = preNode


# Finding minimal weight
def FindMinWeight(disNodeList, heurList = None):

    minNodeKey = list(disNodeList.keys())[0]

    for nodeKey, node in disNodeList.items():
        if heurList:
            if node.weight + heurList[nodeKey] < disNodeList[minNodeKey].weight + heurList[minNodeKey]:
                minNodeKey = nodeKey
        else:
            if node.weight < disNodeList[minNodeKey].weight:
                minNodeKey = nodeKey
    return minNodeKey
    pass

# Calculating Heuristic for all Nodes in graph
def InitHeurList(graph, goal):
    heurList = {}
    for i, node in enumerate(graph):
        heurList[i] = math.sqrt((graph[i][0][0]-graph[goal][0][0])**2+(graph[i][0][1]-graph[goal][0][1])**2)
    return heurList
    pass


def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    print("Implement A* algorithm.")

    graph[start][3] = red
    graphUI.updateUI()
    time.sleep(1)

     # Start node has weight==0
    node = Node(weight=0)

    # Saving node was discovered 
    disNodeList = {start: node}

     # Saving node was completed
    compNodeList = {}

    heurList = InitHeurList(graph,goal)

    graph[start][3] = yellow
    graphUI.updateUI()

    while True:

        i = FindMinWeight(disNodeList, heurList)

        if i == goal:
            graph[i][3] = purple
            graphUI.updateUI()
            des = goal
            compNodeList[des] = disNodeList[des]
            while des != start:
                pre = compNodeList[des].preNode
                edges[edge_id(pre, des)][1] = green
                des = pre
                graphUI.updateUI()
            graph[start][3] = orange
            graphUI.updateUI()
            return

        graph[i][3] = yellow
        graphUI.updateUI()
        time.sleep(1)

        for j in graph[i][1]:
            if j not in compNodeList.keys():
                jWeight = math.sqrt((graph[i][0][0]-graph[j][0][0])**2+(graph[i][0][1]-graph[j][0][1])**2) + disNodeList[i].weight
                if j not in disNodeList.keys():
                    disNodeList[j] = Node(weight=jWeight, preNode=i)
                    graph[j][3] = red
                    edges[edge_id(i, j)][1] = white
                else:
                    if jWeight < disNodeList[j].weight:
                        edges[edge_id(disNodeList[j].preNode, j)][1] = black
                        disNodeList[j] = Node(weight=jWeight, preNode=i)
                        graph[j][3] = red
                        edges[edge_id(i, j)][1] = white


        graph[i][3] = blue
        compNodeList[i] = disNodeList[i]
        disNodeList.pop(i)
        graphUI.updateUI()
        time.sleep(2)
    pass


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    (139, 140),             # position of node when draw on UI
                                    [1, 2],                 # list of adjacent node
                                    (100, 100, 100),        # grey - node edged color
                                    (0, 0, 0)               # black - node fill color
                                ],
                                [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()

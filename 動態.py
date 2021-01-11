import random
import numpy
import numpy as np
import math
import time
import collections
import networkx as nx
import copy
from itertools import islice
from matplotlib import pyplot as plt
from matplotlib import colors
#import gurobipy as gp
#from gurobipy import GRB

class Task:
    def __init__(self,task_no, start_time, start_position,goal_position):
        self.task_no = task_no
        self.start_time = start_time
        self.start_position = start_position
        self.goal_position = goal_position
        self.end_time = -1
        self.estimate_end_time = 0
    def printTask(self):
        print("Task no:",self.task_no)
        print("Start time:",self.start_time)
        print("Start position:",self.start_position)
        print("End position:",self.goal_position)
        print("")
    def estimateTaskTime(self,grid_size):
        sr,sc = self.start_position//grid_size, self.start_position%grid_size
        gr,gc = self.goal_position//grid_size, self.goal_position%grid_size
        self.estimate_end_time =  self.start_time + np.abs(gr-sr) + np.abs(gc-sc)

class Agent:
    def __init__(self, No, position):
        self.No = No
        self.position = position
        self.path = []
        self.idle = True
        self.onTask = False
    def assignTask(self,task):
        self.task = task
        self.idle = False
        

# Task func.
def assignTasks(agentDict,WaitingTaskList,Grid_size):
    idleAgentsIdx = getIdleAgentsIdx(agentDict)
    while len(WaitingTaskList)>0 and WaitingTaskList[0].start_time <= t and len(idleAgentsIdx)>0:
        task = WaitingTaskList.pop(0)
        dmin = Grid_size * Grid_size
        tmpNo = -1
        for i in idleAgentsIdx:
            d = calculateDist(task.start_position, agentDict[i].position, Grid_size)
            if(d<dmin):
                dmin = d
                tmpNo = i
        print('Task' + str(task.task_no) ,' assign to agent:',tmpNo)
        agentDict[tmpNo].assignTask(task)
        idleAgentsIdx.remove(tmpNo)
def diffTypeTask(gs,shelfPos):
    s,g = -1,-1 
    randomNum = random.random()
    if randomNum < 0.25: #from station to shelf
        s = random.randint(gs*(gs-1), gs*gs-1)
        g = shelfPos[random.randint(0, len(shelfPos)-1)]
    elif randomNum < 0.5: #from shelf to shelf
        while s==g:
            s = shelfPos[random.randint(0, len(shelfPos)-1)]
            g = shelfPos[random.randint(0, len(shelfPos)-1)]
    elif randomNum < 0.75: #from shelf to picking station
        s = shelfPos[random.randint(0, len(shelfPos)-1)]
        g = random.randint(0, gs-1)
    else: #from picking station to shelf
        s = random.randint(0, gs-1)
        g = shelfPos[random.randint(0, len(shelfPos)-1)]
    return s,g
def disBetweenTask(_from,_to,grid_size):
    r1,c1 = _from.goal_position//grid_size, _from.goal_position%grid_size
    r2,c2 = _to.start_position//grid_size, _to.start_position%grid_size
    return np.abs(r1-r2) + np.abs(c1-c2)
def taskFeasible(taskList,task,now,totalTime,taskCounter,maxAgentNum,gridSize):
    for ta in taskList:
        if ta.start_time==task.start_time:
            t1s, t1g = ta.start_position, ta.goal_position
            t2s, t2g = task.start_position, task.goal_position
            if t1s==t2s or t1s==t2g or t1g==t2s or t1g==t2g:
                return False
    return True
def GenerateILPTask(gridSize):
    taskList = []
    start_time = []
    starts = []
    goals = []
    tmpGoals = []
    for i in range(agentNum):
        tmp = random.randint(0,gridSize*gridSize-gridSize-1)
        while tmp in tmpGoals:
            tmp = random.randint(0,gridSize*gridSize-gridSize-1)
        tmpGoals.append(tmp)
        taskList.append(Task(i,random.randint(0,2*gridSize),
                             random.randint(0,gridSize*gridSize-1),tmp))
                        
    taskList = sorted(taskList, key = lambda s: s.start_time)
    for ta in taskList:
        start_time.append(ta.start_time)
        starts.append(ta.start_position)
        goals.append(ta.goal_position)
    return taskList,start_time,starts,goals
def GenerateTaskSeries(totalTime,gridSize,maxAgentNum):
    probOfNewTask = 0.4
    timeOffset = 1
    addTaskMaxAtOnce = 2
    task_no = 2
    taskCounter = 1
    taskList = []
    
    if gridSize==5:
        shelfPos = [6,8,16,18]
    elif gridSize==7:
        shelfPos = [15,16,18,19,29,30,32,33]
    elif gridSize==10:
        shelfPos = [21,22,23,26,27,28,31,32,33,36,37,38,
                    61,62,63,66,67,68,71,72,73,76,77,78]
    elif gridSize==15:
        shelfPos = [31,32,33,34,35,36,38,39,40,41,42,43,
                    46,47,48,49,50,51,53,54,55,56,57,58,
                    91,92,93,94,95,96,98,99,100,101,102,103,
                    106,107,108,109,110,111,113,114,115,116,117,118,
                    151,152,153,154,155,156,158,159,160,161,162,163,
                    166,167,168,169,170,171,173,174,175,176,177,178]
    elif gridSize==20:
        shelfPos = []
    else:
        print('Shelf position of this grid size is not defined.')
        shelfPos = []
    
    firstTask = Task(1,0,random.randint(gridSize*(gridSize-1), gridSize*gridSize-1),shelfPos[random.randint(0, len(shelfPos)-1)])
    firstTask.estimateTaskTime(gridSize)
    taskList.append(firstTask)
    
    for t in range(totalTime):
        for ta in taskList:
            if ta.estimate_end_time==t:
                taskCounter -= 1
        if random.random()<probOfNewTask:
            addTask = random.randint(1, addTaskMaxAtOnce)
            for a in range(addTask):
                if taskCounter<maxAgentNum: #there must be some idle agent
                    start,goal = diffTypeTask(gridSize,shelfPos)
                    tmp = Task(task_no,t,start,goal)
                    tmp.estimateTaskTime(gridSize)
                    while not taskFeasible(taskList, tmp, t, totalTime, taskCounter,maxAgentNum,gridSize):
                        start,goal = diffTypeTask(gridSize,shelfPos)
                        tmp = Task(task_no,t,start,goal)
                        tmp.estimateTaskTime(gridSize)
                    if tmp.estimate_end_time >= totalTime - timeOffset:
                        continue
                    taskList.append(tmp)
                    task_no += 1
                    taskCounter += 1
    return taskList


# Map func.
def GenerateMap(Mapsize, Obs_ratio, Robot_num):
    Arr = numpy.zeros((Mapsize, Mapsize))
    ran_list = list(range(0, Mapsize * Mapsize))
    Obs_num = int((Mapsize * Mapsize) * Obs_ratio)
    Robot_position = int(Robot_num * 2)

    output_list = random.sample(ran_list, Obs_num + Robot_position)

    Obs_list = output_list[0:Obs_num]
    Robot_list = output_list[Obs_num:]

    for i in Obs_list:
        row = int(i / Mapsize)
        col = i % Mapsize
        Arr[row][col] = 1

    s = 2
    for j in range(0, len(Robot_list)):
        if (j % 2 == 0):
            # start
            i = Robot_list[j]
            row = int(i / Mapsize)
            col = i % Mapsize
            Arr[row][col] = -1 * s
        else:
            i = Robot_list[j]
            row = int(i / Mapsize)
            col = i % Mapsize
            Arr[row][col] = s
            s = s + 1

    return Arr.astype(int)
def GenerateClearMap(Mapsize, Obs_ratio):
    Arr = numpy.zeros((Mapsize, Mapsize))
    ran_list = list(range(0, Mapsize * Mapsize))
    Obs_num = int((Mapsize * Mapsize) * Obs_ratio)
    Obs_list = random.sample(ran_list, Obs_num)

    for i in Obs_list:
        row = int(i / Mapsize)
        col = i % Mapsize
        Arr[row][col] = 1

    return Arr.astype(int)
def GenerateKivaMap(map_index):
    if (map_index == 0):
        # 6x6
        Arr = numpy.zeros((6, 6))
        for i in range(6):
            for j in range(6):
                if i == 0 or i == 2:
                    if j % 3 != 1:
                        Arr[i][j] = 1
        return Arr.astype(int)
    
    if (map_index == 1):                
        # 9x9
        Arr = numpy.zeros((9, 9))
        for i in range(9):
            for j in range(9):
                if i == 0 or i == 2 or i == 3 or i == 5:
                    if j % 4 != 0:
                        Arr[i][j] = 1
        return Arr.astype(int)
   
    if (map_index == 2):
        # 12x12
        Arr = numpy.zeros((12, 12))
        for i in range(12):
            for j in range(12):
                if i == 0 or i == 2 or i == 3 or i == 5 or i == 6 or i == 8:
                    if j != 0 and j != 4 and j != 7 and j != 11:
                        Arr[i][j] = 1
        return Arr.astype(int)
def getCurrentMap(agentDict,ClearGraph, Grid_size):  
    pos_fetch = {}
    for k,v in agentDict.items():
        pos_fetch[k] = v.position 
            
    tmp_Graph = ClearGraph.copy()
    for k, v in pos_fetch.items():
        sr,sc = v//Grid_size, v%Grid_size
        tmp_Graph[sr][sc] = k
    return tmp_Graph
def GenerateSNG(GraphAtTimet,newAgentsAmount,Grid_size):
    start_arr = []
    goal_arr = []
    
    empty_spaces = []
    for i in range(0,Grid_size*Grid_size):
        x,y = convertNoToXY(i,Grid_size)
        if(GraphAtTimet[x][y]==0):
            empty_spaces.append(i)
    
    for k in range(0,newAgentsAmount):
        #suppose len(empty_spaces)>newAgentAmount
        station_area_bound = Grid_size * (Grid_size - 3)
        
        #select the start position
        index = random.randrange(len(empty_spaces))
        while(empty_spaces[index]<station_area_bound):
            index = random.randrange(len(empty_spaces))
        start_arr.append(empty_spaces[index])
        empty_spaces.pop(index)
        
        #select the goal position
        index = random.randrange(len(empty_spaces))
        while(empty_spaces[index]>=station_area_bound):
            index = random.randrange(len(empty_spaces))
        goal_arr.append(empty_spaces[index])
        empty_spaces.pop(index)
        
    return start_arr,goal_arr
def change_map(ClearGraph,starts,goals,Grid_size): 
    for k,v in starts.items():
        sr,sc = starts[k]//Grid_size, starts[k]%Grid_size
        gr,gc = goals[k]//Grid_size, goals[k]%Grid_size
        ClearGraph[sr][sc] = -1 * k
        ClearGraph[gr][gc] = k
    return ClearGraph


# Path func.
def getCurrentStep(nodenum_onelayer, agents_path):
    pos_fetch = {k:[] for k in agents_path.keys()}
    for k, v in agents_path.items():
        tmp = (v[0]) % nodenum_onelayer
        pos_fetch[k].append(tmp)
    return pos_fetch  
def popCurrentStep(nodenum_onelayer, agents_path):
    delKey = []
    for k, v in agents_path.items():
        v.pop(0)
        if(len(v)==0):
            delKey.append(k)
            continue
        i = 1
        m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
        while(v[len(v)-1] >= m):
            i = i + 1
            m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
        
        while(v[0]>=m):
            v.pop(0)
        v.pop(0)
    for k in delKey:
        del agents_path[k]
    return delKey
def popCurrentStepMultiPath(nodenum_onelayer, agents_path):
    for k, vv in agents_path.items():
        if(len(vv)==0):
            continue
        for v in vv:
            v.pop(0)
            
            i = 1
            m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
            while(v[len(v)-1] >= m):
                i = i + 1
                m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
            
            while(v[0]>=m):
                v.pop(0)
            v.pop(0)
def prunePath(selectedPaths,Grid_size):
    for k, v in selectedPaths.items():
        if isinstance(v[0], list):
            for i in v:
                while( len(i)>2 and (i[-2]-i[-3]) == (Grid_size*Grid_size) ):
                    i.pop(-1)
        else:
            while( len(v)>2 and (v[-2]-v[-3]) == (Grid_size*Grid_size) ):
                v.pop(-1)
def remove_coll_path(all_path_dict,exist_path_dict,grid_size,C):
    new_path = {}
    vertex_num = grid_size*grid_size
    edge_num = 2 * grid_size * (grid_size-1)
    node_num = vertex_num * (2 * C * grid_size + 1)
    gadget_index_start_diffC = 2 * grid_size * vertex_num
    
    for k,v in all_path_dict.items(): #each agent
        path = []
        for pi in v: #each path of agent k  
            colli_path = False
            for ok,ov in exist_path_dict.items(): #each old agent
                if k==ok:
                    continue
                j = oj = 0 #index of each path
                
                #define C is 1 or 2
                node_num_old = -1
                C_BOUND = vertex_num * (2 * 1 * grid_size + 1)
                OldC = 1
                if (ov[len(ov)-1] > C_BOUND):
                    node_num_old = vertex_num * (2 * 2 * grid_size + 1)
                    OldC = 2
                else:
                    node_num_old = C_BOUND
                    
                while(j < len(pi) and oj < len(ov)):
                    if(ov[oj]>=node_num_old and pi[j]<node_num):
                        oj = oj+2
                    elif(ov[oj]<node_num_old and pi[j]>=node_num):
                        j = j+2
                    else:
                        if(pi[j]<node_num): 
                            if((ov[oj]-pi[j]) % vertex_num==0):
                                colli_path = True
                                break
                            j = j+1
                            oj = oj+1
                        else: #both r gadget
                            if(C!=OldC):
                                if (abs(ov[oj]-pi[j])-gadget_index_start_diffC) % edge_num==0:
                                    colli_path = True
                                    break
                            else:
                                if((ov[oj]-pi[j]) % edge_num==0):
                                    colli_path = True
                                    break  
                            j = j+2
                            oj = oj+2
                if(colli_path==True):
                    break
            if(colli_path==False):
                path.append(pi)
        if(len(path)==0):
            new_path[k] = []
            print('\x1b[6;30;42m' + str(k) + ' has no path' + '\x1b[0m')
        else:
            new_path[k] = path
    return new_path
def getAllNewPaths(CleanGraph,agentPath,starts,goals,Grid_size,K_PATH):
    tmpGraph = CleanGraph.copy()
    Arr = change_map(tmpGraph,starts,goals,Grid_size)
    D1_map = Arr.reshape(-1)
    nodenum_onelayer = len(D1_map)
    Q_time_list = []
    Part_Q_time_list = []
    path_dict = {}
    
    for i in range(1,3):
        path_find = True
        map_time = time.time()
        timespace_G, timespace_s, timespace_goal, timespace_G_mapping, Gadget_mapping = timespace_map_Q(Arr,i,starts,goals)
        Q_time_list.append(time.time() - map_time)
        Part_Q_time_list.append(time.time() - map_time)
        G, s, goal, G_mapping = timespace_map(Arr,i,starts,goals,agentPath)
        try:
            path_dict = get_path_dict(G, timespace_G, s, goal, G_mapping, timespace_G_mapping, Gadget_mapping,nodenum_onelayer,K_PATH)
        except:
            print("Can't reach destination in k-shortest path")
            continue
        
        path_dict = remove_coll_path(path_dict,agentPath,Grid_size,i)
        
        delKey = []
        for (k,v) in path_dict.items():
            if(len(v)==0):
                delKey.append(k)
                path_find = False
        for k in delKey:
            del path_dict[k]
        if(path_find==False):
            print("No path after remove collision path, c=", i)
            continue
        break
    
    return path_dict,timespace_G


# Q related func.
def timespace_map_Q(MapArr, t_ratio, starts, goals):
    D1_map = MapArr.reshape(-1)
    nodenum_onelayer = len(D1_map)
    Map_length = len(MapArr)

    # getAdj
    Adj = [[] for i in range((Map_length * Map_length))]
    for i in range(0, Map_length):
        for j in range(0, Map_length):
            if (MapArr[i][j] != 1):
                try:
                    if (MapArr[i][j + 1] != 1):
                        Adj[(i * Map_length + j)].append(i * Map_length + j + 1)
                except:
                    pass

                try:
                    if (MapArr[i][j - 1] != 1 and (j - 1) >= 0):
                        Adj[(i * Map_length + j)].append(i * Map_length + j - 1)
                except:
                    pass

                try:
                    if (MapArr[i - 1][j] != 1 and (i - 1) >= 0):
                        Adj[(i * Map_length + j)].append((i - 1) * Map_length + j)
                except:
                    pass

                try:
                    if (MapArr[i + 1][j] != 1):
                        Adj[(i * Map_length + j)].append((i + 1) * Map_length + j)
                except:
                    pass

    # CreateMap
    G = nx.DiGraph()
    mapping = {}
    # add Basic node
    for i in range(0, Map_length * t_ratio * 2 + 1):
        if i != (Map_length * t_ratio * 2):
            # except final

            if i % 2 == 0:
                # Adj gadget
                for j in range(0, nodenum_onelayer):
                    tempAdj = [x + nodenum_onelayer * i for x in Adj[j]]
                    G.add_node(j + nodenum_onelayer * i, Gadget=False, Attr=D1_map[j], Adja=tempAdj, Prime=True)
                    mapping[(j + nodenum_onelayer * i)] = "t" + str(int(i / 2)) + "x" + str(j % Map_length) + "y" + str(
                        int(j / Map_length))
            else:
                # Adj self
                for j in range(0, nodenum_onelayer):
                    if (D1_map[j] == 1):
                        tempAdj = []
                    else:
                        tempAdj = [j + nodenum_onelayer * (i + 1)]
                    G.add_node(j + nodenum_onelayer * i, Gadget=False, Attr=D1_map[j], Adja=tempAdj, Prime=False)

        else:
            for j in range(0, nodenum_onelayer):
                tempAdj = [j + nodenum_onelayer * (i + 1)]
                G.add_node(j + nodenum_onelayer * i, Gadget=False, Attr=D1_map[j], Adja=[], Prime=None)
                mapping[(j + nodenum_onelayer * i)] = "t" + str(int(i / 2)) + "x" + str(j % Map_length) + "y" + str(
                    int(j / Map_length))
    # add Basic edge
    Prime = nx.get_node_attributes(G, 'Prime')
    Adja = nx.get_node_attributes(G, 'Adja')
    for i in range(0, Map_length * t_ratio * 2):
        for j in range(0, nodenum_onelayer):
            if (len(Adja[(nodenum_onelayer * i) + j]) > 0):
                if (Prime[(nodenum_onelayer * i) + j] == True):
                    G.add_edge((nodenum_onelayer * i) + j, (nodenum_onelayer * (i + 1)) + j, capa=1, cost=1)
                else:
                    G.add_edge((nodenum_onelayer * i) + j, (nodenum_onelayer * (i + 1)) + j, capa=1, cost=0)

    # Gadget
    temp_G = G.copy()
    Gadget_start_index = len(G.nodes)
    Gadget_mapping = {}

    for (p, d) in temp_G.nodes(data=True):
        if (d['Prime'] != None):
            if (len(d['Adja']) > 0):
                if (d['Gadget'] == False):
                    if (d['Prime'] == True):
                        for a in d['Adja']:
                            for (k, v) in G.nodes.data():
                                find = False
                                if (v['Gadget'] == True):
                                    if (v['Adja'] == [p, a] or v['Adja'] == [a, p]):
                                        G.add_edge(p, k, capa=1, cost=0)
                                        find = True
                                        Gadget_mapping[(p,a+nodenum_onelayer)] = k
                                        break
                            if (find == False):
                                G.add_node(Gadget_start_index, Gadget=True, Adja=[p, a])
                                G.add_edge(p, Gadget_start_index, capa=1, cost=0)
                                Gadget_mapping[(p,a+nodenum_onelayer)] = Gadget_start_index
                                Gadget_start_index = Gadget_start_index + 1

    temp_G = G.copy()
    for (p, d) in temp_G.nodes(data=True):
        if (d['Gadget'] == True):
            tempAdja = [x + nodenum_onelayer for x in d['Adja']]
            G.add_node(Gadget_start_index, Gadget=True, Adja=tempAdja)
            G.add_edge(p, Gadget_start_index, capa=1, cost=1)
            for i in tempAdja:
                G.add_edge(Gadget_start_index, i, capa=1, cost=0)

            Gadget_start_index = Gadget_start_index + 1

    start_dict = {}
    goal_dict = {}

    vertex_num = Map_length * Map_length
    node_num = vertex_num * (2 * t_ratio * Map_length + 1)
    for k,v in starts.items():
        start_dict[k] = v
    for k,v in goals.items():
        goal_dict[k] = node_num - vertex_num + v

    return [G, start_dict, goal_dict, mapping , Gadget_mapping]
def timespace_map(MapArr, t_ratio, starts, goals,agentPath):
    D1_map = MapArr.reshape(-1)
    nodenum_onelayer = len(D1_map)
    Map_length = len(MapArr)

    # getAdj
    Adj = [[] for i in range((Map_length * Map_length))]
    for i in range(0, Map_length):
        for j in range(0, Map_length):

            if (MapArr[i][j] != 1):
                try:
                    if (MapArr[i][j + 1] != 1):
                        Adj[(i * Map_length + j)].append(i * Map_length + j + 1)
                except:
                    pass

                try:
                    if (MapArr[i][j - 1] != 1 and (j - 1) >= 0):
                        Adj[(i * Map_length + j)].append(i * Map_length + j - 1)
                except:
                    pass

                try:
                    if (MapArr[i - 1][j] != 1 and (i - 1) >= 0):
                        Adj[(i * Map_length + j)].append((i - 1) * Map_length + j)
                except:
                    pass

                try:
                    if (MapArr[i + 1][j] != 1):
                        Adj[(i * Map_length + j)].append((i + 1) * Map_length + j)
                except:
                    pass

    # add Basic node
    G = nx.DiGraph()
    mapping = {}

    for i in range(0, Map_length * t_ratio + 1):
        if (i != Map_length * t_ratio):
            for j in range(0, nodenum_onelayer):
                tempAdj = [x + nodenum_onelayer * (i + 1) for x in Adj[j]]
                tempAdj.append(j + nodenum_onelayer * (i + 1))
                G.add_node(j + nodenum_onelayer * i, Attr=D1_map[j], adja=tempAdj)
                mapping[(j + nodenum_onelayer * i)] = "t" + str(i) + "x" + str(j % Map_length) + "y" + str(
                    int(j / Map_length))
        else:
            for j in range(0, nodenum_onelayer):
                tempAdj = []
                G.add_node(j + nodenum_onelayer * i, Attr=D1_map[j], adja=tempAdj)
                mapping[(j + nodenum_onelayer * i)] = "t" + str(i) + "x" + str(j % Map_length) + "y" + str(
                    int(j / Map_length))
    #final_index = (Map_length * t_ratio) * nodenum_onelayer - 1

    # add edge
    # Adja = nx.get_node_attributes(G, 'Adja')
    for k, v in G.nodes(data=True):
        t = k // nodenum_onelayer
        G.add_edges_from([(k, x) for x in v['adja'] if (x-k) != nodenum_onelayer], 
                         weight=2**t)
        G.add_edges_from([(k, x) for x in v['adja'] if (x-k) == nodenum_onelayer], 
                         weight=1)

    real_agents_path = convertAgentsPath(nodenum_onelayer, agentPath)
    for a,p in real_agents_path.items():
        # update_path = []
        shift = 0
        while len(p) > 1:
            u = p[1] + shift
            v = p[0] + shift + nodenum_onelayer
            try:
                G[u][v]['weight'] = 9999
            except:
                break
            p.pop(0)
            shift += nodenum_onelayer
    
    start_dict = {}
    goal_dict = {}

    vertex_num = Map_length * Map_length
    node_num = vertex_num * (t_ratio * Map_length + 1)
    for k,v in starts.items():
        start_dict[k] = v
    for k,v in goals.items():
        goal_dict[k] = node_num - vertex_num + v

    return [G, start_dict, goal_dict, mapping]
def CalMultiPathCost(G, path_dict,CAPA):
    # Hyperpara penalty
    penalty = 9999
    path_list = list(path_dict.values())
    # update edge capacity
    temp_G = G.copy()
    temp_capa = {}
    for k,v in CAPA.items():
        temp_capa[k] = {'capa':v}
    for p in path_list:
        for i in range(0, len(p) - 1):
            temp_capa[p[i], p[i+1]]['capa'] -= 1
    nx.set_edge_attributes(temp_G, temp_capa)

    capa_2 = nx.get_edge_attributes(temp_G, 'capa')
    Gadget = nx.get_node_attributes(temp_G, 'Gadget')
    Attr = nx.get_node_attributes(temp_G, 'Attr')
    cost_att = nx.get_edge_attributes(temp_G, 'cost')

    path_cost = {}
    for k, p in path_dict.items():
        cost = 0
        for i in range(1, len(p)):
            if (capa_2[p[i - 1], p[i]] < 0):
                cost = cost + penalty

            elif ((Gadget[p[i - 1]] == False) and (Gadget[p[i]] == False)):
                if (Attr[p[i - 1]] == Attr[p[-1]] and Attr[p[i - 1]] == Attr[p[i]]):
                    cost = cost + 0
                else:
                    cost = cost + cost_att[p[i - 1], p[i]]
            else:
                cost = cost + cost_att[p[i - 1], p[i]]
        path_cost[k] = cost
    return path_cost
def CalMultiPathCost_Modified(G, path_dict, CAPA, grid_size):
    C = -1
    cBound = grid_size*grid_size*(2*1*grid_size + 1)
    threshold = cBound + 2 * grid_size * (grid_size-1)
    penalty = 9999
    path_cost = {}
    tmp_path = path_dict.copy()
    prunePath(tmp_path, grid_size)
    for k,v in tmp_path.items():
        cost = 0
        if C==-1:
            if (v[1]-v[0]) > threshold:
                C = 2
            else:
                C = 1
        for i in range(0,len(v)-1):
            colli = False
            for ok,ov in tmp_path.items():
                if k==ok or i > len(ov)-2:
                    continue
                if v[i]==ov[i] and v[i+1]==ov[i+1]: #collision
                    colli = True
                    cost += penalty
            if not colli and v[i]<cBound and v[i+1]<cBound:
                cost += 1
        path_cost[k] = cost
    return path_cost
def CalSinglePathCost(G, path):
    # wait at end cost 0
    cost = 0
    cost_att = nx.get_edge_attributes(G, 'cost')
    Gadget = nx.get_node_attributes(G, 'Gadget')
    Attr = nx.get_node_attributes(G, 'Attr')

    for i in range(1, len(path)):
        if ((Gadget[path[i - 1]] == False) and (Gadget[path[i]] == False)):
            if (Attr[path[i - 1]] == Attr[path[-1]] and Attr[path[i - 1]] == Attr[path[i]]):
                cost = cost + 0
            else:
                cost = cost + cost_att[path[i - 1], path[i]]
        else:
            cost = cost + cost_att[path[i - 1], path[i]]

    return cost
def GetAllKeyByValue(dic, search):
    return [k for k, v in dic.items() if v == search]
def path_mapping(path , timemap, nodenum_onelayer, ori_map_mapping, time_map_mapping, Gadget_mapping):
    p = []
    time_map_mapping_inv = {v: k for k, v in time_map_mapping.items()}

    for i in range(0, len(path)):
        if (i != len(path) - 1):
            mapping_l = ori_map_mapping[path[i]]
            mapping_r = ori_map_mapping[path[i + 1]]
            time_mapping_l =  time_map_mapping_inv[mapping_l]
            above_time_mapping_r = (time_map_mapping_inv[mapping_r]-nodenum_onelayer)

            # checkifStay
            if(mapping_l.split('x')[1] == mapping_r.split('x')[1]):
                temp_p = ([time_mapping_l,above_time_mapping_r])
            else:
                Gad = Gadget_mapping[(time_mapping_l,above_time_mapping_r)]
                Gad_r = list(timemap[Gad])[0]
                temp_p = ([time_mapping_l,Gad,Gad_r,above_time_mapping_r])

            # temp_p = list(all_simple_paths_graph_modify(timemap, time_mapping_l, above_time_mapping_r,nodenum_onelayer))

            p.extend(temp_p)

    #add goal vertex
    p.append(time_map_mapping_inv[mapping_r])
    return p
def all_simple_paths_graph(G, source, targets):
    visited = collections.OrderedDict.fromkeys([source])
    stack = [iter(G[source])]

    while stack:
        children = stack[-1]
        child = next(children, None)
        if child is None:
            stack.pop()
            visited.popitem()
        elif len(visited):
            if child in visited:
                continue

            if child == targets:
                yield list(visited) + [child]
            else:
                visited[child] = None
                stack.append(iter(G[child]))
def k_shortest_paths(G, source, target, k, weight=None):
    return list(
        islice(nx.shortest_simple_paths(G, source, target, weight=weight), k)
    )
def all_simple_paths_graph_modify(G, source, targets, nodenum_onelayer):
    Gadget = nx.get_node_attributes(G, 'Gadget')
    #Attr = nx.get_node_attributes(G, 'Attr')

    visited = collections.OrderedDict.fromkeys([source])
    stack = [iter(G[source])]

    while stack:
        children = stack[-1]
        child = next(children, None)
        if child is None:
            stack.pop()
            visited.popitem()
        elif len(visited):
            if child in visited:
                continue

            if (len(visited) >= 3 and Gadget[child] == False):
                rev_list = list(reversed(visited))
                if (Gadget[rev_list[0]] == True):
                    if (child == rev_list[2] + nodenum_onelayer):
                        continue
            if child == targets:
                yield list(visited) + [child]
            else:
                visited[child] = None
                stack.append(iter(G[child]))
def get_path_dict(mapp, timemap, start_dict, goal_dict, map_mapping, time_map_mapping, Gadget_mapping, nodenum_onelayer,K_PATH):
    all_path_dict = {}
    #start,goal_dict is generate from map
    for k,v in start_dict.items():
        mapping_path = []
        temp_p = k_shortest_paths(mapp, v, goal_dict[k], K_PATH, weight="weight")
        for i in temp_p:
            mapping_path.append(path_mapping(i,timemap,nodenum_onelayer, map_mapping, time_map_mapping, Gadget_mapping))
        all_path_dict[k] = mapping_path

    return all_path_dict
def pass_makespan_threshold(Grid_size,selectedPaths):
    maximum = 0
    real_agents_path = convertAgentsPath(Grid_size*Grid_size,selectedPaths)
    
    for k,TXG_path in real_agents_path.items():
        l = len(TXG_path)
        if(l>maximum):
            maximum = l
    if(l>2*Grid_size):
        return False
    return True
def QLearning(G, T, L, E, all_path_dict,Grid_size):
    # L(Lambda) E(Epilson)
    Convergence_count = T * 0.2

    # get valid path (del through gadget stay)
    Valid_all_path_dict = all_path_dict.copy()

    Q_table = {}
    Q_table_count = {}
    for k, v in Valid_all_path_dict.items():
        Q_table[k] = {i: CalSinglePathCost(G, v[i]) for i in range(0, len(v))}
        Q_table_count[k] = {i: 0 for i in range(0, len(v))}
        
    action_list = {}
    action_index_list = {}

    CAPA = nx.get_edge_attributes(G, 'capa')
    tmpT = 0
    
    for i in range(0, T):
        if (Convergence_count <= 0):
            #print('Convergence')
            break
        a = math.pow(L, i + 1)
        b = math.pow(E, i + 1)  # epsilon

        for k, v in Valid_all_path_dict.items():
            # epsilon-greedy
            if (len(v) == 0):
                #print('agent', k, 'cannot get valid path')
                return

            temp_float = random.random()

            if (temp_float <= b):
                # random
                temp_rand = random.randint(0, len(v) - 1)
                action = v[temp_rand]
                action_index_list[k] = temp_rand

            else:
                # greedy
                best = min(list(Q_table[k].values()))
                best_key_list = GetAllKeyByValue(Q_table[k], best)
                temp_rand = random.randint(0, len(best_key_list) - 1)

                action = v[best_key_list[temp_rand]]
                action_index_list[k] = best_key_list[temp_rand]

            action_list[k] = (action)

        # cal travel time
        tt = time.time()
        #path_cost = CalMultiPathCost(G, action_list,CAPA)
        path_cost = CalMultiPathCost_Modified(G, action_list, CAPA, Grid_size)
        tmpT += (time.time()-tt)

        for k, v in Q_table.items():
            action_index = action_index_list[k]
            if (Q_table[k][action_index] == ((1 - a) * Q_table[k][action_index] + (a * path_cost[k]))):
                Convergence_count = Convergence_count - 1
            else:
                Convergence_count = T * 0.2

            Q_table[k][action_index] = (1 - a) * Q_table[k][action_index] + (a * path_cost[k])
            Q_table_count[k][action_index] = Q_table_count[k][action_index] + 1

    # get best action
    for k, v in Valid_all_path_dict.items():
        best = min(list(Q_table[k].values()))

        best_key_list = GetAllKeyByValue(Q_table[k], best)
        count = [Q_table_count[k][i] for i in best_key_list]
        count_most = numpy.argmax(count)

        action = v[best_key_list[count_most]]
        action_index_list[k] = best_key_list[count_most]
        action_list[k] = action
        
    #best_cost = CalMultiPathCost(G, action_list,CAPA)
    best_cost = CalMultiPathCost_Modified(G, action_list, CAPA, Grid_size)
    
    for k,v in best_cost.items():
        if v>=9999:
            print('\x1b[6;30;42m' + 'Collision Path' + '\x1b[0m')
            print('Colli path:',k,v,action_list[k])
        #else:
            #print(k,v,action_list[k])

    return action_list


# ILP func.   
def ILP_GenerateMap(task_s, task_g, Mapsize):
    Arr = numpy.zeros((Mapsize, Mapsize))
    s = -2
    for item in task_s:
        i = item // Mapsize
        j = item % Mapsize
        Arr[i][j] = s
        s -= 1
    g = 2
    for item in task_g:
        i = item // Mapsize
        j = item % Mapsize
        Arr[i][j] = g
        g += 1

    return Arr.astype(int)
def timespace_map_ILP(MapArr, t_ratio):
    # getAdj
    D1_map = MapArr.reshape(-1)
    D1_map_node = []
    Adj = [[] for i in range((len(MapArr) * len(MapArr)))]
    map_len = len(MapArr)

    for x in range(0, len(D1_map)):
        D1_map_node.append(x)

    for i in range(0, len(MapArr)):
        for j in range(0, len(MapArr)):
            try:
                if (MapArr[i][j + 1] != 1):
                    Adj[(i * map_len + j)].append(i * map_len + j + 1)
            except:
                pass

            try:
                if (MapArr[i][j - 1] != 1 and (j - 1) >= 0):
                    Adj[(i * map_len + j)].append(i * map_len + j - 1)
            except:
                pass

            try:
                if (MapArr[i - 1][j] != 1 and (i - 1) >= 0):
                    Adj[(i * map_len + j)].append((i - 1) * map_len + j)
            except:
                pass

            try:
                if (MapArr[i + 1][j] != 1):
                    Adj[(i * map_len + j)].append((i + 1) * map_len + j)
            except:
                pass

    conv_dict = {}
    for i in range(0, len(D1_map_node)):
        conv_dict[D1_map_node[i]] = i

    # CreateMap
    # add Basic Nodes
    DiG_root = nx.DiGraph()
    map_len = len(D1_map_node)
    prime = 0
    for i in range(0, len(MapArr) * t_ratio * 2 + 1):
        prime = prime + 1
        if (i != 0):
            if (prime % 2 == 1):
                for j in range(0, map_len):
                    tempAdj = [conv_dict[x] + (map_len) * i for x in Adj[D1_map_node[j]]]
                    DiG_root.add_node(j + (map_len) * i, Gadget=False, Attr=D1_map[D1_map_node[j]], Adja=tempAdj,
                                      Prime=False)
            else:
                for j in range(0, map_len):
                    tempAdj = [(j + (map_len) * i) + map_len]
                    DiG_root.add_node(j + (map_len) * i, Gadget=False, Attr=D1_map[D1_map_node[j]], Adja=tempAdj,
                                      Prime=True)
        else:
            for j in range(0, map_len):
                tempAdj = [conv_dict[x] + (map_len) * i for x in Adj[D1_map_node[j]]]
                DiG_root.add_node(j + (map_len) * i, Gadget=False, Attr=D1_map[D1_map_node[j]], Adja=tempAdj,
                                  Prime=False)

    #Add Basic Edges
    Prime = nx.get_node_attributes(DiG_root, 'Prime')
    for i in range(0, len(MapArr) * t_ratio * 2):
        for j in range(0, map_len):
            if (Prime[j + (map_len) * i] == True):
                # blue edge
                DiG_root.add_edge(j + (map_len) * i, j + (map_len) * (i + 1), capa=1, cost=0)
            else:
                # green edge
                DiG_root.add_edge(j + (map_len) * i, j + (map_len) * (i + 1), capa=1, cost=1)

    # Gadget
    G = DiG_root.copy()
    temp_G = DiG_root.copy()

    Gadget_start_index = len(G.nodes)
    layer_num = len(MapArr) * t_ratio * 2 + 1
    num_in_onelayer = len(D1_map_node)
    final_layer_index = num_in_onelayer * (layer_num - 1)

    for (p, d) in temp_G.nodes(data=True):
        if (p < final_layer_index):
            if (d['Gadget'] == False):
                if (d['Prime'] == False):
                    for x in d['Adja']:
                        for (k, v) in G.nodes(data=True):
                            if (v['Gadget'] == True):
                                if ((v['pos_a'] == p and v['pos_b'] == x)):
                                    find_res = k
                                    break
                                elif ((v['pos_a'] == x and v['pos_b'] == p)):
                                    find_res = k
                                    break
                                else:
                                    find_res = False
                            else:
                                find_res = False

                        if (find_res != False):
                            G.add_edge(p, find_res, capa=1, cost=0)
                        else:
                            G.add_node(Gadget_start_index, Gadget=True, pos_a=p, pos_b=x)
                            G.add_edge(p, Gadget_start_index, capa=1, cost=0)
                            Gadget_start_index = Gadget_start_index + 1

    new_index = len(G.nodes)

    temp_G = G.copy()
    for (p, d) in temp_G.nodes(data=True):
        if (d['Gadget'] == True):
            G.add_node(new_index, Gadget=True, pos_c=d['pos_a'] + map_len, pos_d=d['pos_b'] + map_len)
            G.add_edge(p, new_index, capa=1, cost=1)
            G.add_edges_from([(new_index, d['pos_a'] + map_len), (new_index, d['pos_b'] + map_len)], capa=1, cost=0)
            new_index = new_index + 1

    start_dict = {}
    goal_dict = {}

    for (p, d) in G.nodes(data=True):
        if (d['Gadget'] == False):
            if (p < map_len):
                if (d['Attr'] < 0):
                    start_dict[abs(d['Attr'])] = p
            if (p >= final_layer_index):
                if (d['Attr'] > 1):
                    goal_dict[abs(d['Attr'])] = p

    # set Loopback edges

    for (p, d) in goal_dict.items():
        G.add_edge(d, start_dict[p], capa=1, cost=0, loopback=True)

    
    return [G, start_dict, goal_dict]
def ILP(G, start_dict, goal_dict,task_start_time,task_start,Grid_size):
    start = collections.OrderedDict(sorted(start_dict.items()))
    goal = collections.OrderedDict(sorted(goal_dict.items()))
    taskStart = collections.OrderedDict(sorted(task_start.items()))
    
    # Gurobi preprocessing
    nodes = list(G.nodes)
    #C1 = nx.get_edge_attributes(G, 'capa')
    #C2 = nx.get_edge_attributes(G, 'cost')
    e = []
    loopback = nx.get_edge_attributes(G, 'loopback')
    edges = list(G.edges)
    for i in range(1, len(start_dict) + 1):
        for j in loopback:
            if (j[1] == start_dict[i + 1]):
                e.append(j)
                edges.remove(j)
    e.extend(edges)
    Robot = []
    for i in range(1, len(start_dict) + 1):
        Robot.append(i)

    Robot_num = len(Robot)

    e_index = []
    index_to_edge = {} #給index可查出是哪條邊

    for i in range(len(e)):
        e_index.append(i + 1)
        index_to_edge[i + 1] = e[i]
    
    #給邊可查出index 
    inverse_index_to_edge = dict((v, k) for k, v in index_to_edge.items())
    
    back_shift = 0
    while True:
        # Gurobi
        m = gp.Model('netflow')
        m.setParam('OutputFlag',False)
        # Create variables
        x = m.addVars(Robot, e_index,vtype=GRB.BINARY, name="x")
        m.setObjective(gp.quicksum(x[i, i] for i in range(1, Robot_num + 1)), GRB.MAXIMIZE)
        # Constraint1: 每個時間點的每個邊只能有一個人通過
        m.addConstrs(
            (x.sum('*', i) <= 1 for i in e_index), "cons1")

        # Constraint2: agent之間不經過彼此的loopback edge
        m.addConstrs(
            (x[i, j] == 0 for i in range(1, Robot_num + 1)
            for j in range(1, Robot_num + 1)
            if i != j), name='Cons2')

        # Constraint3: 對每個agent自己的每個點，入邊總和要等於出邊總和
        for v in nodes:
            for r in range(1, Robot_num + 1):
                temp_in_edges = []
                temp_out_edges = []
                for i in G.in_edges(v):
                    temp_in_edges.append(inverse_index_to_edge[i])
                for i in G.out_edges(v):
                    temp_out_edges.append(inverse_index_to_edge[i])

                m.addConstr(
                    (gp.quicksum(x[r, j] for j in temp_in_edges) ==
                    gp.quicksum(x[r, j] for j in temp_out_edges)
                    ))
        
        # Constraint4: agent要在規定的時間過後才能離開初始位置
        # 利用start_dict中每個agent的起點丟進G.out_edges(v)來找出不動的那個邊
        # 判斷那個邊是我們要的並得知其連接的另一個點的號碼，存進list並執行下一個迴圈
        
        # edge_must_stay = {}
        # n = Grid_size*Grid_size
        # i = -1
        # for (a,s) in start.items():
        #     edge_must_stay[a] = []
        #     next_node = s
        #     i += 1
        #     for j in range(2*task_start_time[i]):
        #         for e in G.out_edges(next_node):
        #             if (e[1]-e[0] == n):
        #                 edge_must_stay[a].append(inverse_index_to_edge[e])
        #                 tmp_next_node = e[1]
        #         next_node = tmp_next_node
        # r = 0
        # for a,e in edge_must_stay.items():
        #     r += 1
        #     if (len(e) == 0):
        #         continue
        #     m.addConstrs(x[r, j] == 1 for j in e)
        
        
        # constraint5:要經過task起始位置
        n = Grid_size*Grid_size
        time = next(iter(goal_dict.items()))[1] // n
        task_route = {}
        i = -1
        for (a,s) in taskStart.items():
            task_route[a] = []
            i += 1
            next_node = s + 2*task_start_time[i]*n
            for j in range(2*task_start_time[i],time+1):
                for e in G.in_edges(next_node):
                    task_route[a].append(inverse_index_to_edge[e])
                next_node += n
        r = 0
        for a,e in task_route.items():
            r += 1
            m.addConstr(gp.quicksum(x[r, i] for i in e) >= 1)
        
            
        # Constraint6(optional):縮短大家到達的deadline
        if back_shift>0:
            edge_to_stay = {}
            for (a,g) in goal.items():
                edge_to_stay[a] = []
                next_node = g
                for i in range(back_shift):
                    for e in G.in_edges(next_node):
                        if (e[1]-e[0] == n):
                            edge_to_stay[a].append(inverse_index_to_edge[e])
                            tmp_next_node = e[0]
                    next_node = tmp_next_node
            r = 0
            for a,e in edge_to_stay.items():
                r += 1
                m.addConstrs(x[r, j] == 1 for j in e)
        
        m.optimize()
        
        try:
            obj = m.getObjective()
            obj_getValue = obj.getValue()           
            
            #all_path = []
            all_path_dict = {}
            if m.status == GRB.OPTIMAL:
                solution = m.getAttr('x', x)
                # print("solution", solution)
                for r in Robot:
                    all_path_dict[r + 1] = []
                    for j in e_index:
                        if solution[r, j] > 0:
                            #print(r,j,index_to_edge[j])
                            #all_path.append(index_to_edge[j])
                            all_path_dict[r + 1].append(index_to_edge[j])
            
            # print("all_path_dict", all_path_dict)
            
            # print("s",start_dict)
            # print("g",goal_dict)
            
            Robot_path = {}
            for (p, d) in start_dict.items():
                Robot_path[p] = [d]
                while ((Robot_path[p])[-1] != goal_dict[p]):
                    if (len(all_path_dict[p]) < 1):
                        print('error,ILP go next round')
                        break

                    temp_list = all_path_dict[p].copy()
                    for i in temp_list:
                        if (i[0] == (Robot_path[p])[-1]):
                            Robot_path[p].append(i[1])
                            all_path_dict[p].remove(i)
                            if ((Robot_path[p])[-1] == goal_dict[p]):
                                break
            # print("robot_path",Robot_path)
            
            # calc makespan of every agent
            makespan = 0
            agent_makespan = {}
            for a,s in Robot_path.items():
                t_start = 2*task_start_time[a-2]
                t_end = s[-1] // n
                for j in range(-2,t_start-len(s),-2):
                    if s[j]-s[j-1] == n:
                        t_end -= 2
                        continue
                    break
                else:
                    print("error: robot never move")        
                #print("agent",a," task end at time ",t_end,sep='')
                if t_end>makespan:
                    makespan = t_end
                
                agent_makespan[a] = (t_end - t_start) // 2
            print("agent_makespan", agent_makespan)
            
        except Exception as e:
            print("ILP teminate,",e)
            real_robot_path = convertAgentsPath(n,Robot_path)
            print("robot_path",real_robot_path)
            break
            
        back_shift += 2
        
    # path_cost = CalMultiPathCost(G, Robot_path)
    # makespan = max(path_cost.values())
   
    avg_work_time = 0
    for a,m in agent_makespan.items():
        avg_work_time += m
    
    
    return [makespan//2, avg_work_time, obj_getValue]
def Main_ILP(Grid_size,Robot_num,Arr,task_start_time,task_start_list):
    t = time.time()
    
    # i可以直接開成我要的大小 然後跑一次就完事
    i = (task_start_time[-1] // Grid_size) + 2
    
    ILP_G, ILP_start, ILP_goal = timespace_map_ILP(Arr,i)
    print("start", ILP_start)
    print("goal", ILP_goal)
    task_start = {}
    for j in range(len(task_start_list)):
        task_start[j+2] = task_start_list[j]
    
    Makespan_ILP, avg_work_time, ObjValue = ILP(ILP_G,ILP_start,ILP_goal,task_start_time,task_start,Grid_size)
    avg_work_time = avg_work_time/Robot_num
    if(ObjValue == Robot_num):
        print('ILP find sol')
        return [Makespan_ILP,avg_work_time,(time.time()-t)]

    #if T = 2 can't find
    print('ILP cannot find')
    return False


# Tool & Mathematic func.
def getIdleAgentsIdx(agentDict):
    l = []
    for k,v in agentDict.items():
        if(agentDict[k].idle):
            l.append(k)
    return l
def calculateDist(a,b,Grid_size):
    tx,ty = convertNoToXY(a, Grid_size)
    ax,ay = convertNoToXY(b, Grid_size)
    tmp = abs(tx-ax) + abs(ty-ay)
    return tmp
def transferOneItemList2Int(li):
    return li[0]
def convertNoToXY(i,Grid_size):
    return i//Grid_size,i%Grid_size
def convertAgentsPath(nodenum_onelayer, agents_path):
    agents_path_tmp = copy.deepcopy(agents_path)
    real_agents_path = {k:[] for k in agents_path_tmp.keys()}
    for k, v in agents_path_tmp.items():
        
        while(len(v) > 0):
            tmp = (v.pop(0)) % nodenum_onelayer
            real_agents_path[k].append(tmp)
            if(len(v)==0):
                continue
            
            i = 1
            m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
            while(v[len(v)-1] >= m):
                i = i + 1
                m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
            
            while(v[0]>=m):
                v.pop(0)
            v.pop(0)
    return real_agents_path
def convertMultiAgentsPath(nodenum_onelayer, agents_path):
    agents_path_tmp = copy.deepcopy(agents_path)
    real_agents_path = {k:[] for k in agents_path_tmp.keys()}
    for k, p in agents_path_tmp.items():
        for v in p:
            path_to_convert = []
            while(len(v) > 0):
                tmp = (v.pop(0)) % nodenum_onelayer
                path_to_convert.append(tmp)
                if(len(v)==0):
                    continue
                
                i = 1
                m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
                while(v[len(v)-1] >= m):
                    i = i + 1
                    m = nodenum_onelayer*(2*(i*math.sqrt(nodenum_onelayer))+1)
                
                while(v[0]>=m):
                    v.pop(0)
                v.pop(0)
            real_agents_path[k].append(path_to_convert)
    return real_agents_path
def visualize(data,Grid_size,t,agentDict,agentNum):
    colorList = ['white','blue','green','black','red','yellow','orange']
    cmap = colors.ListedColormap(colorList)
    plt.figure(figsize=(Grid_size,Grid_size))
    plt.pcolor(data[::-1],cmap=cmap,edgecolors='k', linewidths=3)
    plt.title('t = ' + str(t))
    
    for i in range(2,2+agentNum):
        if(agentDict[i].idle):
            plt.figtext(0.95, 0.8 - (i-2)*0.1 , colorList[i] + ' is idle')
        else:
            task = agentDict[i].task
            plt.figtext(0.95, 0.8 - (i-2)*0.1 , colorList[i] + '\'s task ' 
                        + str(task.start_position) + ' -> ' + str(task.goal_position))
    plt.show()
def isLegal(pos, dire, Grid_size):
    action = [True, True, True, True]
    p = pos % (Grid_size*Grid_size)
    idleI, idleJ = convertNoToXY(p,Grid_size)
    if idleI == 0:
        action[0] = False
    if idleI == Grid_size-1:
        action[2] = False
    if idleJ == 0:
        action[3] = False
    if idleJ == Grid_size-1:
        action[1] = False
    return action[dire]
def isGoalSame(goals):
    for k,v in goals.items():
        for kk,vv in goals.items():
            if k==kk:
                continue
            elif v==vv:
                return True
    return False
def getAdjacentNum(a,Grid_size):
    if(a==0 or a== (Grid_size*Grid_size -1) or a == (Grid_size-1) or a==(Grid_size*(Grid_size-1)) ):
        return 2
    elif( (a//Grid_size)==0 or (a%Grid_size)==0 or (a//Grid_size)==(Grid_size-1) or (a%Grid_size)== (Grid_size-1)):
        return 3
    else:
        return 4
def maxAdjacentP(List,Grid_size):
    maxAdj = -1
    maxAdj_p = -1
    for i in List:
        tmp_adj = getAdjacentNum(i,Grid_size)
        if(tmp_adj > maxAdj):
            maxAdj = tmp_adj
            maxAdj_p = i
    return maxAdj_p
def FindMinDist(pos,agentPath,Grid_size):
    if not agentPath:
        return 99
    
    minStep = 99
    p = pos % (Grid_size *  Grid_size)
    for k,v in agentPath.items():
        threshold = (Grid_size *  Grid_size) * (2 * 1 * Grid_size + 1)
        if len(v)==0:
            continue
        if v[len(v)-1]>=threshold:
            threshold = (Grid_size *  Grid_size) * (2 * 2 * Grid_size + 1)
        count = 0
        j = 0
        colli = False
        while j<len(v):
            if v[j] < threshold: #not gadget
                count += 1
                if p == v[j] % (Grid_size *  Grid_size):
                    colli = True
                    break
            if j!=0:
                j+=1 #pass two same nodes
            j+=1
        if count < minStep and colli:
            minStep = count
    return minStep
def DynamicToStatic(agentDict):
    newAgentStarts = {}
    newAgentGoals = {}
    agentPath = {}
    
    for k,v in agentDict.items():
        if v.idle == False:
            if(len(v.path)!=0): #路途中 (可能是前往task起點 或者 前往task終點)
                agentPath[k] = v.path
            elif(v.position==v.task.start_position):   #終點轉起點
                agentDict[k].onTask = True
                print('Agent ',k,' get the commodity at t-1')
                newAgentStarts[k] = v.position  #這樣會在原地多停留一個時間
                newAgentGoals[k] = v.task.goal_position
            #elif(v.position==v.task.goal_position):
            #    print('bug!!')
            else: #前往task的位置
                newAgentStarts[k] = v.position #會在原地多停留一個時間  --  可以把train好的路徑去掉頭
                newAgentGoals[k] = v.task.start_position
    return newAgentStarts,newAgentGoals,agentPath


if __name__ == '__main__':
    #--------------Experiment---------------
    Experiment_start = time.time()

    #-------Static varible initilize--------
    Grid_size = 15
    Obs_ratio = 0
    agentNum = 5
    K_PATH = 100
    agentDict = {}
    preAgentPath = {}
    for i in range(agentNum):
        agentDict[i+2] = Agent(i+2,i + Grid_size*(Grid_size-1))
    
    CleanGraph = GenerateClearMap(Grid_size,Obs_ratio)
    MapAtTimet = CleanGraph.copy()
    
    Time_length = 30  #determine length of task series in dynamic environment
    totalTaskTime = 0
    Makespan = 0
    thinkingTime = 0.0
    
    #----------------Task------------------
    #WaitingTaskList,taskStartTimeILP,startsILP,goalsILP = GenerateILPTask(Grid_size) #for ILP
    WaitingTaskList = GenerateTaskSeries(Time_length,Grid_size,agentNum)
    totalTask = len(WaitingTaskList)
    for ta in WaitingTaskList:
        print('task'+str(ta.task_no) + ' t=' + str(ta.start_time)+ ', ' + str(ta.start_position) + '->' + str(ta.goal_position) )    
    
    #----------Dynamic Q Main Loop-----------
    for t in range(0,100):
        print('---Time:',t,'---')
        
        assignTasks(agentDict,WaitingTaskList,Grid_size)

        MapAtTimet = getCurrentMap(agentDict,CleanGraph, Grid_size)
        
        visualize(MapAtTimet,Grid_size,t,agentDict,agentNum)
            
        newAgentStarts,newAgentGoals,agentPath = DynamicToStatic(agentDict)

        #---------New Agent Q-Learning----------
        if(len(newAgentStarts)>0):   
            print('\t new agent')
            print('start:',newAgentStarts)
            print('goal: ',newAgentGoals)
            
            allNewPaths,timespace_G = getAllNewPaths(CleanGraph,preAgentPath,newAgentStarts,newAgentGoals,Grid_size,K_PATH)
            thinkTmp = time.time()        
            tmpi = len(newAgentStarts)
            episode = int(K_PATH * (tmpi*tmpi))
            if isGoalSame(newAgentGoals):
                prunePath(allNewPaths,Grid_size)
            selectedPaths = QLearning(timespace_G,episode,0.9,0.1,allNewPaths,Grid_size)
            
            thinkingTime += (time.time() - thinkTmp)
            #print('Current thinking time:', time.time() - thinkTmp)
            
            prunePath(selectedPaths,Grid_size)
            print('selectedPaths:' , selectedPaths )
            popCurrentStep(Grid_size*Grid_size,selectedPaths)
            
            agentPath.update(selectedPaths)
        preAgentPath = copy.deepcopy(agentPath)
        
        
        if(bool(agentPath)):    
            #----------------Dodge------------------
            for k,v in agentDict.items(): 
                agent = agentDict[k]
                maxDistNextT = -1
                sameDistPList = []
                decision_direction = -1
                directionList = [-5,1,5,-1]
                if(agent.idle):
                    d = FindMinDist(agent.position,agentPath,Grid_size)
                    
                    if(d <= 2):
                        #print('idle agent',k, 'min Dist:',d)
                        
                        tmp_cs = getCurrentStep(Grid_size*Grid_size,agentPath)
                        currentStepList = []
                        for key, value in tmp_cs.items():
                            currentStepList.append(transferOneItemList2Int(value))
                            
                        apNextT = copy.deepcopy(agentPath)
                        popCurrentStep(Grid_size*Grid_size,apNextT)
                        
                        for direction in range(4):         #0上1右2下3左
                            if( isLegal(agent.position,direction,Grid_size) ):
                                tmp_p = agent.position + directionList[direction]
                                tx,ty = convertNoToXY(tmp_p,Grid_size)
                                tmpk = MapAtTimet[tx][ty]
                                if ( tmpk!=0 and (agentDict[tmpk].idle or 
                                    agentPath[tmpk][0] % (Grid_size*Grid_size) ==agent.position)):
                                    if(agentDict[tmpk].idle): 
                                        print('move to '+str(tmp_p) + ' would meet idle agent !')
                                    elif(agentPath[tmpk][0] % (Grid_size*Grid_size) ==agent.position ):
                                        print('move to '+str(tmp_p) + ' would occur crossover !')
                                    #crossover or idle agent!!
                                else:
                                    if tmp_p in currentStepList:
                                        print('move to '+str(tmp_p) + ' would occur collision !')
                                        continue
                                    
                                    DistAtNextT = FindMinDist(tmp_p,apNextT,Grid_size)
                                    print('if move to '+str(tmp_p) + ', Dist:',DistAtNextT)
                                    if(DistAtNextT > maxDistNextT):
                                        maxDistNextT = DistAtNextT
                                        sameDistPList.clear()
                                        sameDistPList.append(tmp_p)
                                    elif(DistAtNextT == maxDistNextT):
                                        sameDistPList.append(tmp_p)
                                        
                if(maxDistNextT==0):
                    print('dead lock, GG')
                elif( maxDistNextT!=-1):
                    #print('\x1b[6;31;46m' + 'dodge!' + '\x1b[0m')
                    agent.position = maxAdjacentP(sameDistPList,Grid_size)
                          
            #------------Move One Step--------------
            current_steps = getCurrentStep(Grid_size*Grid_size,agentPath)
            for k,v in current_steps.items():
                tmpint = transferOneItemList2Int(v)
                agentDict[k].position = tmpint 
                if(agentDict[k].position == agentDict[k].task.start_position): 
                    pass
                    #agentDict[k].onTask = True
                    #print('agent ',k,' get the commodity')
                elif(agentDict[k].onTask and agentDict[k].position == agentDict[k].task.goal_position): #agent must have a task here
                    agentDict[k].onTask = False
                    agentDict[k].idle = True
                    agentDict[k].task.end_time = t
                    totalTaskTime += (t - agentDict[k].task.start_time + 1)
                    Makespan = t
                    print('agent ',k,' finished a task from ',agentDict[k].task.start_time,' to ',t)
                    
            finishedList = popCurrentStep(Grid_size*Grid_size,agentPath)
            for i in finishedList:
                agentDict[i].path = []
            
            for k,v in agentPath.items():
                agentDict[k].path = v
            
        elif(len(WaitingTaskList)==0):
            print('All tasks are completed')
            break
    Experiment_time = time.time() - Experiment_start
    print('----------Dynamic Q----------')
    print('Experiment time =',Experiment_time)
    print('Q-learning time =',thinkingTime)
    print('Total task time =',totalTaskTime)
    print('# of task =',totalTask)
    print('Avg task time =',totalTaskTime/totalTask)
    print('Makespan:',Makespan)
    print(totalTask,totalTaskTime,thinkingTime,Experiment_time,sep="\t")
    
    #----------------ILP Main------------------
    Experiment_start = time.time()
    agent_start = []
    for i in range(agentNum):
        agent_start.append(i + Grid_size*(Grid_size-1))
    ILP_Arr = ILP_GenerateMap(agent_start, goalsILP, Grid_size)
    #ILP start
    try:
        makespan_ILP, work_ILP, time_ILP = Main_ILP(Grid_size,agentNum,ILP_Arr,taskStartTimeILP,startsILP)
        Experiment_time = time.time() - Experiment_start
        print('----------ILP----------')
        print('Experiment time =',Experiment_time)
        print('# of task =',totalTask)
        print("Avg task time =",work_ILP)
        print("Makespan =",makespan_ILP)
    except Exception as e:
        print("exception",e)
        print('cannot find ILP')
    # ILP end
import time
from matplotlib import pyplot as plt
import numpy as np
from pymaze import maze, agent, COLOR, textLabel
from queue import PriorityQueue
from memory_profiler import memory_usage
import tracemalloc


def dfs(maze):
    st = time.time()
    goal_node = (1,1)
    start_node = (maze.rows, maze.cols)
    visited_nodes = [start_node]
    visiting_nodes = [start_node]
    dfs_path={}
    dSearch = []
    while len(visiting_nodes)>0:
        current_node = visiting_nodes.pop()
        if(current_node == goal_node):
            break;
        for direction in "ESNW":
            if maze.maze_map[current_node][direction]==1:
                if direction == "W":
                    child_node = (current_node[0],current_node[1]-1)
                elif direction == "N":
                    child_node = (current_node[0]-1,current_node[1])
                elif direction == "S":
                    child_node = (current_node[0]+1,current_node[1])
                elif direction == "E":
                    child_node = (current_node[0],current_node[1]+1)
                if child_node in visited_nodes:
                    continue
                visited_nodes.append(child_node)
                visiting_nodes.append(child_node)
                dfs_path[child_node]=current_node
                dSearch.append(child_node)

    traversed_path={}
    final_node = goal_node
    while final_node!=start_node:
        traversed_path[dfs_path[final_node]]=final_node
        final_node = dfs_path[final_node]
    et = time.time()
    elapsed_time = et - st
    return dSearch, traversed_path, elapsed_time

def bfs(maze):
    st = time.time()
    goal_node = (1,1)
    start_node = (maze.rows,maze.cols)
    queue = [start_node]
    visited_nodes = [start_node]
    bfs_path={}
    bSearch = []
    while len(queue)>0:
        current_node = queue.pop(0)
        if(current_node == goal_node):
            break;
        for direction in "ESNW":
            if maze.maze_map[current_node][direction]==1:
                if direction == "W":
                    child_node = (current_node[0],current_node[1]-1)
                elif direction == "N":
                    child_node = (current_node[0]-1,current_node[1])
                elif direction == "S":
                    child_node = (current_node[0]+1,current_node[1])
                elif direction == "E":
                    child_node = (current_node[0],current_node[1]+1)
                if child_node in visited_nodes:
                    continue
                queue.append(child_node)
                visited_nodes.append(child_node)
                bfs_path[child_node]=current_node
                bSearch.append(child_node)
    traversed_path={}
    final_node = goal_node
    while final_node!=start_node:
        traversed_path[bfs_path[final_node]]=final_node
        final_node = bfs_path[final_node]
    et = time.time()
    elapsed_time = et - st
    return bSearch, traversed_path, elapsed_time

def get_manhattan_distance(node_1, node_2):
     x1,y1 = node_1
     x2,y2 = node_2 

     return abs(x1-x2) + abs(y1-y2)

def atstar_algorithm(maze):
    st = time.time()
    goal_node = (1,1)
    start_node = (maze.rows, maze.cols)
    g = {node:float('inf') for node in maze.grid}
    g[start_node] = 0 
    f = {node:float('inf') for node in maze.grid}
    f[start_node] = get_manhattan_distance(start_node,goal_node)
    priority_queue = PriorityQueue()
    priority_queue.put((get_manhattan_distance(start_node,goal_node), get_manhattan_distance(start_node,goal_node), start_node))
    apath={}
    searchPath=[start_node]
    while not priority_queue.empty():
        current_node = priority_queue.get()[2]
        searchPath.append(current_node)
        if current_node==goal_node:
            break
        for direction in "ESNW":
            if maze.maze_map[current_node][direction]==1:
                if direction == "W":
                    child_node = (current_node[0],current_node[1]-1)
                elif direction == "N":
                    child_node = (current_node[0]-1,current_node[1])
                elif direction == "S":
                    child_node = (current_node[0]+1,current_node[1])
                elif direction == "E":
                    child_node = (current_node[0],current_node[1]+1)
                
                g_temp = g[current_node]+1
                f_temp = g_temp + get_manhattan_distance(child_node, goal_node)

                if f_temp < f[child_node]:
                    g[child_node] = g_temp
                    f[child_node] = f_temp
                    priority_queue.put((f_temp, get_manhattan_distance(child_node, goal_node), child_node))
                    apath[child_node] = current_node
    traversed_path={}
    final_node = goal_node
    while final_node!=start_node:
        traversed_path[apath[final_node]]=final_node
        final_node = apath[final_node]
    et = time.time()
    elapsed_time = et - st
    return searchPath, traversed_path, elapsed_time

def value_iteration(P, R, gamma=0.9, theta=0.001):
    V = {state: 0 for state in P}
    count = 1
    while True:
        count = count+1
        delta = 0
        for s in P:
            v = V[s]
            max_q = -np.inf
            for a in P[s]:
                q = R[s][a] + gamma * V[P[s][a]]
                if q > max_q:
                    max_q = q
            V[s] = max_q
            delta = max(delta, abs(v - V[s]))
        if delta < theta:
            break
    return V, count

def mdp_value_iteration(maze):
    actions = ['N', 'S', 'E', 'W']
    T = {}
    R = {}
    for i in range(1, maze.rows+1):
        for j in range(1, maze.cols+1):
            state = (i, j)
            T[state] = {}
            R[state] = {}
            for a in actions:
                if a == 'N':
                    next_state = (i-1, j)
                elif a == 'S':
                    next_state = (i+1, j)
                elif a == 'E':
                    next_state = (i, j+1)
                elif a == 'W':
                    next_state = (i, j-1)

                if maze.maze_map[state][a] == 1:
                    T[state][a] = next_state
                    R[state][a] = -1
                else:
                    T[state][a] = state
                    R[state][a] = -1000
            if state == (1, 1):
                R[state]['E'] = 1000
    st = time.time()
    V, count= value_iteration(T, R)
    et = time.time()
    elapsed_time = et - st
    policy = {}
    for s in T:
        max_q = -np.inf
        for a in T[s]:
            q = R[s][a] + V[T[s][a]]
            if q > max_q:
                max_q = q
                policy[s] = a

    path = [(maze.rows, maze.cols)]
    current_state = (maze.rows, maze.cols)
    while current_state != (1, 1):
        action = policy[current_state]
        if action == 'N':
            next_state = (current_state[0]-1, current_state[1])
        elif action == 'S':
            next_state = (current_state[0]+1, current_state[1])
        elif action == 'E':
            next_state = (current_state[0], current_state[1]+1)
        elif action == 'W':
            next_state = (current_state[0], current_state[1]-1)
        path.append(next_state)
        current_state = next_state

    return path, elapsed_time, count

def policy_iteration(P, R, actions, gamma=0.9, theta=0.001):
    V = {state: 0 for state in P}
    policy = {state: actions[0] for state in P}
    count = 1
    while True:
        count = count+1
        # Policy evaluation
        while True:
            delta = 0
            for s in P:
                v = V[s]
                a = policy[s]
                V[s] = sum([P[s][a][next_state] * (R[s][a] + gamma * V[next_state]) for next_state in P[s][a]])
                delta = max(delta, abs(v - V[s]))
            if delta < theta:
                break
        
        # Policy improvement
        policy_stable = True
        for s in P:
            old_action = policy[s]
            max_q = -np.inf
            for a in P[s]:
                q = sum([P[s][a][next_state] * (R[s][a] + gamma * V[next_state]) for next_state in P[s][a]])
                if q > max_q:
                    max_q = q
                    policy[s] = a
            if old_action != policy[s]:
                policy_stable = False
        if policy_stable:
            break
    
    return policy, V, count


def mdp_policy_iteration(maze):
    actions = ['N', 'S', 'E', 'W']
    T = {}
    R = {}
    for i in range(1, maze.rows+1):
        for j in range(1, maze.cols+1):
            state = (i, j)
            T[state] = {}
            R[state] = {}
            for a in actions:
                if a == 'N':
                    next_state = (i-1, j)
                elif a == 'S':
                    next_state = (i+1, j)
                elif a == 'E':
                    next_state = (i, j+1)
                elif a == 'W':
                    next_state = (i, j-1)

                if maze.maze_map[state][a] == 1:
                    T[state][a] = {next_state: 1}
                    R[state][a] = -1
                else:
                    T[state][a] = {state: 1}
                    R[state][a] = -1000
            if state == (1, 1):
                R[state]['E'] = 1000
    st = time.time()
    policy, V, count = policy_iteration(T, R, actions)
    et = time.time()
    elapsed_time = et - st
    path = [(maze.rows, maze.cols)]
    current_state = (maze.rows, maze.cols)
    while current_state != (1, 1):
        action = policy[current_state]
        if action == 'N':
            next_state = (current_state[0]-1, current_state[1])
        elif action == 'S':
            next_state = (current_state[0]+1, current_state[1])
        elif action == 'E':
            next_state = (current_state[0], current_state[1]+1)
        elif action == 'W':
            next_state = (current_state[0], current_state[1]-1)
        path.append(next_state)
        current_state = next_state
    return path, elapsed_time, count

#Runing Depth First Search Algorithm and Initiating its Maze Agent
def run_dfs(maze):
    dfs_search, path_dfs, execution_time = dfs(maze)
    print("Depth First Search Execution Time in Milliseconds : ",execution_time*1000)
    a = agent(maze, footprints=True, color = COLOR.blue)
    maze.tracePath({a:path_dfs})
    l=textLabel(maze,'No of Nodes Traversed in DFS : ',len(dfs_search)+1)
    l=textLabel(maze,'Shortest Length in DFS : ',len(path_dfs)+1)

#Running Breath First Search Algorithm and Initiating its Maze Agent
def run_bfs(maze):
    bfs_search, path_bfs, execution_time = bfs(maze)
    print("Breath First Search Execution Time in Milliseconds : ",execution_time*1000)
    b = agent(maze, footprints=True, color = COLOR.green)
    maze.tracePath({b:path_bfs})
    l=textLabel(maze,'No of Nodes Traversed in BFS : ',len(bfs_search)+1)
    l=textLabel(maze,'Shortest Length in BFS : ',len(path_bfs)+1)

#Running A Star Algorithm and Initiating its Maze Agent
def run_atstar(maze):
    astart_search, path_astar, execution_time = atstar_algorithm(maze)
    print("A Star Execution Time in Milliseconds: ",execution_time*1000)
    c = agent(maze, footprints=True, color = COLOR.red)
    maze.tracePath({c:path_astar})
    l=textLabel(maze,'No of Nodes Traversed in A-STAR : ',len(astart_search)+1)
    l=textLabel(maze,'Shortest Length in A-STAR : ',len(path_astar)+1)

#Running MDP - Value Iteration Algorithm and Initiating its Maze Agent
def run_mdp_vi(maze):
    mdp_vi, execution_time, count_vi = mdp_value_iteration(maze)
    print("MDP - Value Iteration's Execution Time in Milliseconds: ",execution_time*1000)
    print("No of runs in MDP - Value Iteration : ", count_vi)
    d = agent(maze, footprints=True, color=COLOR.purple)
    maze.tracePath({d: mdp_vi})
    l = textLabel(maze, 'Shortest Length in MDP Value Iteration : ', len(mdp_vi))

#Running MDP - Policy Iteration Algorithm and Initiating its Maze Agent
def run_mdp_pi(maze):
    mdp_pi, execution_time, count_pi = mdp_policy_iteration(maze)
    print("MDP - Policy Iteration's Execution Time in Milliseconds: ",execution_time*1000)
    print("No of runs in MDP - Policy Iteration : ", count_pi)
    e = agent(maze, footprints=True, color=COLOR.dark_golden_rod)
    maze.tracePath({e: mdp_pi})
    l = textLabel(maze, 'Shortest Length in MDP Policy Iteration : ', len(mdp_pi))

def maze_memory_usage(maze_rows, maze_columns, algorithm):

    from pymaze import maze
    maze = maze(maze_rows,maze_columns)
    maze.CreateMaze(loopPercent=100, theme=COLOR.light)
    tracemalloc.start()
    if algorithm=='dfs' or algorithm=='all':
        result = run_dfs(maze)
    if algorithm=='bfs' or algorithm=='all':
        result = run_bfs(maze)
    if algorithm=='astar' or algorithm=='all':
        result = run_atstar(maze)
    if algorithm=='value_iteration' or algorithm=='all':
        result = run_mdp_vi(maze)
    if algorithm=='policy_iteration' or algorithm=='all':
        result = run_mdp_pi(maze)
    current, peak = tracemalloc.get_traced_memory()

    print(f"Current memory usage: {current/1024} KB")
    print(f"Peak memory usage: {peak/1024} KB")
    
    tracemalloc.stop()

def run_maze_ai(maze_rows, maze_columns, algorithm):

    from pymaze import maze
    maze = maze(maze_rows,maze_columns)
    maze.CreateMaze(loopPercent=100, theme=COLOR.light)

    if algorithm=='dfs' or algorithm=='all':
        run_dfs(maze)
    if algorithm=='bfs' or algorithm=='all':
        run_bfs(maze)
    if algorithm=='astar' or algorithm=='all':
        run_atstar(maze)
    if algorithm=='value_iteration' or algorithm=='all':
        run_mdp_vi(maze)
    if algorithm=='policy_iteration' or algorithm=='all':
        run_mdp_pi(maze)
    
    maze.run()
    
def main():

    run_maze_ai(5, 15, 'all')
    maze_memory_usage(5, 15, 'dfs') 

if __name__ == "__main__":
    main();
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Directions
from typing import List

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    print("Start:", problem.getStartState())
    dfs_stack = util.Stack()
    visited = []

    # pushing the initial successors from the starting point
    for successor in problem.getSuccessors(problem.getStartState()):    
        dfs_stack.push((successor, [successor[1]]))

    #iterative DFS
    while not dfs_stack.isEmpty():
        #keeping track of the visited vertex and the path to get to said vertex
        visited_vertex, actions_to_goal = dfs_stack.pop()

        #checking if we have reached the goal
        if problem.isGoalState(visited_vertex[0]):
            return actions_to_goal

        #if not the goal, we append to visited then get its successors to add to the stack
        visited.append(visited_vertex[0])
        successor_list = problem.getSuccessors(visited_vertex[0])
        for successor in successor_list:    
            if(successor[0] not in visited):
                dfs_stack.push((successor, actions_to_goal + [successor[1]]))
    # util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    search_queue = util.Queue() ##queue
    visited = set() ##2d array

    search_queue.push((problem.getStartState(), [])) ##add the start state to the search tree
    visited.add(problem.getStartState())

    while not search_queue.isEmpty():
        current_state, path = search_queue.pop()

        if problem.isGoalState(current_state):
            return path
        
        for successor, action, stepcost in problem.getSuccessors(current_state):
            if successor not in visited:
                search_queue.push ((successor, path + [action]))
                visited.add(successor)
    return []
    ##util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    priority_q = util.PriorityQueue()
    visited = []
    visited.append(problem.getStartState())

    for successor in problem.getSuccessors(problem.getStartState()):    
        # pushing in the successor and its action, then its cost is the priority
        priority_q.push(item=(successor, [successor[1]]), priority=successor[2])

    while not priority_q.isEmpty():
        #  pop to get min priority item in queue; you get vertex, and then its action
        visited_vertex, action = priority_q.pop()

        # if it has been already visited, do not explore more
        if visited_vertex[0] in visited:
            continue
        #if it is the goal we return a list of actions
        if problem.isGoalState(visited_vertex[0]):
            return action
        
        #otherwise, mark as visited and explore its successors by adding to priority q  
        visited.append(visited_vertex[0])
        successor_list = problem.getSuccessors(visited_vertex[0])
        for successor in successor_list:    
            if(successor[0] not in visited):
                priority_q.push(item=(successor, action + [successor[1]]), priority=(problem.getCostOfActions(action)+successor[2]))
    
    # util.raiseNotDefined()

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

# each node has an f, g and h where:
# f is the total (estimated) cost of the path through the current node (this is also the priority in the priority queue)
# g is the distance between start node and current node
# h is the (estimated) cost from the current node to the goal 
def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    open_list = util.PriorityQueue()
    closed_list = set()

    start_state = problem.getStartState()
    #print(f"Start State: {start_state}\n") ## I want to know what the start state looks like depending on the problem
    #open_list structure: (current_node, directions, g_cost) with priority = f_cost (g_cost + heurisitic)
    open_list.push((start_state, [], 0), heuristic(start_state, problem)) ## (f_cost is simply the heuristic in this case)
    
    # a map to keep track of the best g_cost for each state
    g_cost_map = {start_state: 0}

    while not open_list.isEmpty():
        # pop the state with the lowest f_cost
        current_node, path, g_cost = open_list.pop()
        # if we've reached the goal, return the map
        if problem.isGoalState(current_node):
            return path
        # if the node has already been expanded with a lower cost, skip it
        if current_node in closed_list and g_cost >= g_cost_map[current_node]:
            continue
        # mark the current node as visited
        closed_list.add(current_node)
        for successor, action, stepCost in problem.getSuccessors(current_node):
            new_g_cost = g_cost + stepCost
            f_cost = new_g_cost + heuristic(successor, problem=problem)
            # if the successor is already in the closed list and the new path is better
            if successor in closed_list and new_g_cost < g_cost_map.get(successor, float('inf')):
                # remove it from the closed list so the node can be expanded again
                closed_list.remove(successor)
            if successor not in g_cost_map or new_g_cost < g_cost_map[successor]:
                g_cost_map[successor] = new_g_cost
                open_list.push((successor, path + [action], new_g_cost), f_cost)
    return[]
    #util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

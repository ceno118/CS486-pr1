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

from tracemalloc import start
import util

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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
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

    '''
    Recursive DFS implementation that adds a successor state's action to a list of actions if that state leads to
    the goal state.
    '''
    '''
    CDT Mason Nunn '23 D4. Written assistance to the author.
    I was having trouble understanding how the successors were presented in the triple after calling getSuccessors.
    CDT Nunn explained the structure of the triple and how I could index it to access the specific parts that I needed
    at a given point. I used that information in all of the problems in this project.
    '''


    actions = []
    visited = set()
    start_state = problem.getStartState()
    
    def dfs(problem, start):
        if problem.isGoalState(start):
            return True
        if start not in visited:
            visited.add(start)
            successors = problem.getSuccessors(start)
            successors.reverse()
            for state in successors:
                if dfs(problem, state[0]):
                    actions.insert(0, state[1])
                    return True
        return False


    dfs(problem, start_state)
    return actions


    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    '''
    https://stackoverflow.com/questions/8922060/how-to-trace-the-path-in-a-breadth-first-search
    I understood how BFS worked in general, but I was having trouble implementing a way to add only the correct
    actions to the list. I found this website, which helped me understand that I could queue entire paths, rather than
    individual nodes. I was able to use that information to solve BFS, and used it for the rest of the search implementations.
    '''
    
    actions = []
    final_path = []
    start_state = problem.getStartState()
    q = util.Queue()
    q.push(start_state)
    visited = set()
    graph = {}
    found = False
    start = True

    def makeActions(p): # Function to turn a path of states into a list of actions to move through those states.
        actList = []
        for i in range(len(p) - 1):
            for next in graph[p[i]]:
                if next[0] == p[i+1]:
                    actList.append(next[1])
        return actList
    
    
    if not problem.isGoalState(start_state):
        while not q.isEmpty() and not found:
            if start:
                path = [q.pop()]
                curr = path[0]
                start = False
            else: 
                path = q.pop()
                curr = path[-1]
            if problem.isGoalState(curr):
                final_path = path
                found = True
            elif curr not in visited:
                visited.add(curr)
                successors = problem.getSuccessors(curr)
                graph[curr] = successors
                for state in successors:
                    t_path = list(path)
                    t_path.append(state[0])
                    if problem.isGoalState(state[0]):
                        q.push_front(t_path) 
                        # Method I added to util.Queue to force something to the front of the queue.
                        # This was the best way I could find to fix a problem where I could find a path to the goal,
                        # but it still expanded another node after due to how things are added to a queue.
                    else:
                        q.push(t_path)

    return makeActions(final_path)

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    final_path = []
    start_state = problem.getStartState()
    q = util.PriorityQueue()
    q.push(start_state, 0)
    visited = set()
    graph = {}
    found = False
    start = True

    def makeActions(p):
        actList = []
        for i in range(len(p) - 1):
            for next in graph[p[i]]:
                if next[0] == p[i+1]:
                    actList.append(next[1])
        return actList
    
    
    if not problem.isGoalState(start_state):
        while not q.isEmpty() and not found:
            if start:
                path = [q.pop()]
                start = False
            else: 
                path = q.pop()
            curr = path[-1]
            if problem.isGoalState(curr):
                final_path = path
                found = True
            elif curr not in visited:
                visited.add(curr)
                successors = problem.getSuccessors(curr)
                graph[curr] = successors
                for state in successors:
                    t_path = list(path)
                    t_path.append(state[0])
                    tAct = makeActions(t_path)
                    t_cost = problem.getCostOfActions(tAct)
                    q.push(t_path, t_cost)

    return makeActions(final_path)



    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"

    final_path = []
    start_state = problem.getStartState()
    q = util.PriorityQueue()
    q.push(start_state, 0)
    visited = set()
    graph = {}
    found = False
    start = True

    def makeActions(p):
        actList = []
        for i in range(len(p) - 1):
            for next in graph[p[i]]:
                if next[0] == p[i+1]:
                    actList.append(next[1])
        return actList
    
    
    if not problem.isGoalState(start_state):
        while not q.isEmpty() and not found:
            if start:
                path = [q.pop()]
                start = False
            else: 
                path = q.pop()
            curr = path[-1]
            if problem.isGoalState(curr):
                final_path = path
                found = True
            elif curr not in visited:
                visited.add(curr)
                successors = problem.getSuccessors(curr)
                graph[curr] = successors
                for state in successors:
                    t_path = list(path)
                    t_path.append(state[0])
                    tAct = makeActions(t_path)
                    t_cost = problem.getCostOfActions(tAct) + heuristic(state[0], problem)
                    q.push(t_path, t_cost)

    return makeActions(final_path)
    
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

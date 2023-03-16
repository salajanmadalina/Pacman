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


class NodeT:

    def __init__(self, position, direction, neighbours, parent, cost=1):
        self.position = position
        self.direction = direction
        self.neighbours = neighbours
        self.parent = parent
        self.cost = cost

    def getDirection(self):
        return self.direction

    def getParent(self):
        return self.parent

    def getNeighbours(self):
        return self.neighbours

    def getPosition(self):
        return self.position

    def getCost(self):
        return self.cost

    def toString(self):
        return (self.position, self.neighbours, self.state)


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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
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
    return_list = []
    stack = util.Stack()
    visited = []
    root_node = NodeT(problem.getStartState(), None, None, None)
    stack.push(root_node)

    while not stack.isEmpty():
        node = stack.pop()

        if node.getPosition() not in visited:
            visited.append(node.getPosition())

            if problem.isGoalState(node.getPosition()):
                while node.getParent() is not None:
                    return_list.append(node.getDirection())
                    node = node.getParent()
                return_list.reverse()
                return return_list

            for child in problem.getSuccessors(node.getPosition()):
                if child[0] not in visited:
                    stack.push(NodeT(child[0], child[1], None, node))


def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    my_list = []
    my_queue = util.Queue()
    root_node = NodeT(problem.getStartState(), None, problem.getSuccessors(problem.getStartState()), None)

    visited = [root_node.getPosition()]
    my_queue.push(root_node)

    while not my_queue.isEmpty():
        node = my_queue.pop()

        for child in node.getNeighbours():
            if problem.isGoalState(child[0]):
                my_list.append(child[1])
                while node.getParent() is not None:
                    my_list.append(node.getDirection())
                    node = node.getParent()
                my_list.reverse()
                return my_list

            if child[0] not in visited:
                new_node = NodeT(child[0], child[1], problem.getSuccessors(child[0]), node)
                visited.append(new_node.getPosition())
                my_queue.push(new_node)


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    my_list = []
    my_priority_queue = util.PriorityQueue()
    root_node = NodeT(problem.getStartState(), None, None, None, 0)
    solution = 0
    node_solution = root_node

    visited = {root_node.getPosition(): 0}
    my_priority_queue.push(root_node, 0)

    while not my_priority_queue.isEmpty():
        node = my_priority_queue.pop()

        if problem.isGoalState(node.getPosition()):
            break

        for child in problem.getSuccessors(node.getPosition()):
            if child[0] not in visited or (child[2] + node.getCost()) < visited[child[0]]:
                visited[child[0]] = child[2] + node.getCost()
                new_node = NodeT(child[0], child[1], None, node, child[2] + node.getCost())
                my_priority_queue.update(new_node, new_node.getCost())

                if problem.isGoalState(child[0]):
                    if solution == 0:
                        solution = new_node.getCost()
                        node_solution = new_node
                    else:
                        if new_node.getCost() < solution:
                            s = new_node.getCost()
                            node_solution = new_node

    while node_solution.getParent() is not None:
        my_list.append(node_solution.getDirection())
        node_solution = node_solution.getParent()

    my_list.reverse()
    return my_list


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    "*** YOUR CODE HERE ***"

    my_list = []
    my_priority_queue = util.PriorityQueue()
    root_node = NodeT(problem.getStartState(), None, None, None, 0)
    solution = 0
    node_solution = root_node

    visited = {root_node.getPosition(): 0}
    my_priority_queue.push(root_node, 0)

    while not my_priority_queue.isEmpty():
        node = my_priority_queue.pop()

        if problem.isGoalState(node.getPosition()):
            break

        for child in problem.getSuccessors(node.getPosition()):
            if child[0] not in visited or (child[2] + node.getCost()) < visited[child[0]]:
                heuristicCost = child[2] + node.getCost() + heuristic(child[0], problem)
                visited[child[0]] = child[2] + node.getCost()
                new_node = NodeT(child[0], child[1], None, node, child[2] + node.getCost())
                my_priority_queue.update(new_node, heuristicCost)

                if problem.isGoalState(child[0]):
                    if solution == 0:
                        solution = new_node.getCost()
                        node_solution = new_node
                    else:
                        if new_node.getCost() < solution:
                            solution = new_node.getCost()
                            node_solution = new_node

    while node_solution.getParent() is not None:
        my_list.append(node_solution.getDirection())
        node_solution = node_solution.getParent()

    my_list.reverse()
    return my_list


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

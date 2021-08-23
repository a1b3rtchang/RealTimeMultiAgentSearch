# myAgents.py
# ---------------
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

from game import Agent
from game import Directions
from game import Actions
from searchProblems import PositionSearchProblem

import util
import time
import search

"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def createAgents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

foodCount = 1
north = Directions.NORTH
south = Directions.SOUTH
east = Directions.EAST
west = Directions.WEST
reverse = {north:south, south:north, east:west, west:east}
class MyAgent(Agent):
    """
    Implementation of your agent.
    """
    customFood = None
    foodLeft = 0
    specialWalls = {}
    finding = []

    def getAction(self, state):
        """
        Returns the next action the agent will take
        """
        "*** YOUR CODE HERE ***"
        x, y = state.getPacmanPosition(self.index)
        numPacmen = state.getNumPacmanAgents()
        if not MyAgent.customFood:
            MyAgent.customFood = state.getFood()
            MyAgent.foodLeft = len(MyAgent.customFood.asList())

        #if not self.foodIsThere(x, y):
        #    self.path = None
        #trueLen = len(state.getFood().asList())
        #if not self.path and self.index < trueLen and trueLen < numPacmen:
        #    problem = MySearchProblem(state, self.index, 1, state.getFood())
        #    self.path = search.bfs(problem)
        if self.path and self.path[0] == 'place':
            if sum(MyAgent.finding) == 1:
                MyAgent.specialWalls[(x, y)] = self.path[1]
            self.path = None

        if not self.path and MyAgent.foodLeft > 0:
            problem = MySearchProblem(state, self.index, min(foodCount, MyAgent.foodLeft), MyAgent.customFood, MyAgent.specialWalls, MyAgent.finding)

            self.path = cbfs(problem)

            

            nx, ny = x, y
            if not self.path:
                return state.getLegalActions(self.index)[0]
            for i in range(len(self.path)):
                action = self.path[i]
                if action == 'place':
                    MyAgent.finding[self.index] = False
                    break
                MyAgent.finding[self.index] = True
                dx, dy = Actions.directionToVector(action)
                nx, ny = int(nx + dx), int(ny + dy)
                check = MyAgent.customFood[nx][ny]
                if check:
                    MyAgent.foodLeft -= 1
                    MyAgent.customFood[nx][ny] = False

        if not self.path:
            return state.getLegalActions(self.index)[0]
        dir = self.path.pop(0)
        return dir

    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """

        "*** YOUR CODE HERE"
        self.path = []
        MyAgent.customFood = None
        MyAgent.foodLeft = 0
        MyAgent.specialWalls = {}
        self.followOne = False
        if self.index == 0:
            MyAgent.finding = []
        MyAgent.finding.append(False)

    

"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class MySearchProblem:

    def __init__(self, gameState, agentIndex, numFood, customFoodGrid, specialWalls, finding):
        self.walls = gameState.getWalls()
        self.position = gameState.getPacmanPosition(agentIndex)
        self.nOfAgents = gameState.getNumPacmanAgents()
        self.specialWalls = specialWalls
        self.pacmen = [gameState.getPacmanPosition(i) for i in range(self.nOfAgents)]
        self.finding = finding
        self.food = customFoodGrid
        self.numFood = numFood
        self.index = agentIndex


    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        #return (self.position, self.food.copy())
        return self.position

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        x, y = state
        return self.food[x][y]

    def isPacman(self, state):
        x, y = state
        if sum(self.finding) > 1:
            return False
        for i in range(self.nOfAgents):
            a, b = self.pacmen[i]
            if i != self.index and self.finding[i] and x == a and y == b:
                return True
        return False
    #def getID(self, state):
    #    x, y = state
    #    i 

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        successors = []
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            #foodV = list(state[1])
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            pos = (nextx, nexty)
            myBlock = False
            if pos in self.specialWalls:
                myBlock = self.specialWalls[pos] == direction
            if not self.walls[nextx][nexty] and not myBlock and not self.isPacman(state):
                #if self.food[nextx][nexty] and not repeat:    
                #foodV.append((nextx, nexty))
                #foodV = tuple(foodV)
                successors.append( ( (nextx, nexty), direction, 1 ) )

        return successors

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        x,y= self.position
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost








class ClosestDotAgent(Agent):

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)


        "*** YOUR CODE HERE ***"
        return search.bfs(problem)

    def getAction(self, state):
        return self.findPathToClosestDot(state)[0]

class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState, agentIndex):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        "*** YOUR CODE HERE ***"
        return self.food[x][y]

def customBreadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    i = 0
    dirList = []
    closed = util.Counter()
    fringe = util.Queue()
    state = problem.getStartState()
    followPac = []
    closed[hash(state)] = 1

    for triple in problem.getSuccessors(state):
        fringe.push((triple, dirList.copy()))
    while not fringe.isEmpty():
        i += 1
        state = fringe.pop()
        succ = state[0][0]
        act = state[0][1]
        cost = state[0][2]
        dirList = state[1]
        dirList.append(act)
        
        if problem.isGoalState(succ):
            return dirList
        if problem.isPacman(succ):
            followPac.append(dirList.copy())
        if closed[hash(succ)] == 0:
            closed[hash(succ)] = 1
            for triple in problem.getSuccessors(succ):
                fringe.push((triple, dirList.copy()))
    if not followPac:
        return
    followPac = max(followPac, key=lambda x: len(x))
    last = followPac.pop()
    followPac.append(last)
    followPac.append('place')
    followPac.append(reverse[last])
    return followPac.copy()

cbfs = customBreadthFirstSearch


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
from multiprocessing import Process
"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def createAgents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

foodCount = 1

class MyAgent(Agent):
    """
    Implementation of your agent.
    """
    customFood = None
    foodLeft = 0

    

    def getAction(self, state):
        """
        Returns the next action the agent will take
        """
        "*** YOUR CODE HERE ***"
        if not MyAgent.customFood:
            MyAgent.customFood = state.getFood()
            MyAgent.foodLeft = len(MyAgent.customFood.asList())
        if not self.path and MyAgent.foodLeft > 0:
            x, y = state.getPacmanPosition(self.index)
            problem = MySearchProblem(state, self.index, 1, MyAgent.customFood)
            p = Process(target=search.bfs, args=[problem])
            p.start()
            p.join()
            for action in self.path:
                dx, dy= Actions.directionToVector(action)
                x, y = int(x + dx), int(y + dy)
                if MyAgent.customFood[x][y]:
                    MyAgent.foodLeft -= 1
                    MyAgent.customFood[x][y] = False
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
        MyAgent.foodLeft = None

    
        

"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class MySearchProblem:

    def __init__(self, gameState, agentIndex, numFood, customFoodGrid):
        self.walls = gameState.getWalls()
        self.position = gameState.getPacmanPosition(agentIndex)
        self.food = customFoodGrid


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
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
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



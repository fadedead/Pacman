# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]


class State:
  """
  This class keeps the state for a current node
  The coordinates field can be filled with the current status of the node
  The route shall keep the path that was taken to reach the current state
  """
  def __init__(self, coordinates, route):
    self.coordinates = coordinates
    self.route = route

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """

  #We keep a stack to maintain the fringeList of the nodes to be visited next
  fringeList = util.Stack()

  #We store the current state in the fringeList, which is the root node,
  #along with a null path, since we have no path till now.
  initialState = State(problem.getStartState(), [])
  fringeList.push(initialState)

  #Visited list keeps track of the states that have been already visited
  visited = []
  
  while(not fringeList.isEmpty()):
    currentState = fringeList.pop()     #Pop the last inserted node from the stack, as is done in a dfs algo
    currNode = currentState.coordinates #Here we get the state and
    route = currentState.route          #route taken till now
    
    for child in problem.getSuccessors(currNode):
      newNode = child[0]    #The first element is the node details
      direction = child[1]  #The second element is the direction taken to reach here
      cost = child[2]       #The third element is the cost taken to get here
      if not newNode in visited:
        
        if(problem.isGoalState(newNode)): #If the node we get is the node we are looking for, we output the route taken
          return route + [direction]
        
        newState = State(newNode, route + [direction]) #We update the direction and make the new state with the current node
        visited = visited + [currNode]                 #We update the visited list with the new node
        fringeList.push(newState)

  return [] #In case we are exhauted with all the paths, and we did not get any output, we return a emtpy list of moves

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 81]"

  #We maintain a queue as is done in a BFS algo
  fringeList = util.Queue()
  
  initialState = State(problem.getStartState(), []) #The initial state consists of the root node, along with a null path
  fringeList.push(initialState)
  visited = [] #We maintain the list of visited nodes
  
  while(not fringeList.isEmpty()):
    currentState = fringeList.pop()     #We get the oldest inserted node from the queue
    currNode = currentState.coordinates
    route = currentState.route
    
    for child in problem.getSuccessors(currNode):
      newNode = child[0]    #The current node
      direction = child[1]  #The direction taken to reach here
      cost = child[2]       #The cost of the path

      if newNode not in visited:
        if problem.isGoalState(newNode): #If the goal is reached, return the path
          return route + [direction]
        newState = State(newNode, route + [direction]) #New state is of the node explored and the total direction to this node
        fringeList.push(newState)
        visited = visited + [newNode]                  #Add the node to the visited list
        
  return []
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  #We maintain a priority queue which outputs the min-most priority node from the current list at O(1) cost
  fringeList = util.PriorityQueue() 

  initialState = State(problem.getStartState(), []) #The intitial state is of the root node and the empty route
  fringeList.push(initialState, 0)
  explored = [] #We maintain a list of explored nodes

  while not fringeList.isEmpty():
    currState = fringeList.pop()     #Get the node with the least priority
    currNode = currState.coordinates #The state of the current node
    currRoute = currState.route      #The route till here

    if problem.isGoalState(currNode):#If goal is reached, output the route
      return currRoute               #This is to be done only after the node has been explored, not when putting int the fringe list

    explored.append(currNode)        #Add poped node to the explored list

    for child in problem.getSuccessors(currNode):
      newCoordinate = child[0]
      direction = child[1]
      cost = child[2]
      if not newCoordinate in explored:  #Node is not explored, then explore it
        newRoute = currRoute + [direction]
        newState = State(newCoordinate, newRoute)
        newCost = problem.getCostOfActions(newRoute)
        fringeList.push(newState, newCost)
        
  return []

  
def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  fringeList = util.PriorityQueue()                 #Priority queue for the A* search
  initialState = State(problem.getStartState(), [])
  fringeList.push(initialState, heuristic(initialState.coordinates, problem)) #Insert with the priority provided by the heuristic function
  explored = []

  while not fringeList.isEmpty():
    currState = fringeList.pop()
    currNode = currState.coordinates
    currRoute = currState.route

    if problem.isGoalState(currNode):   #Output if the node has been explored, and we have reached the goal
      return currRoute

    explored.append(currNode)

    for child in problem.getSuccessors(currNode):
      newCoordinate = child[0]
      direction = child[1]
      cost = child[2]
      if not newCoordinate in explored:
        newRoute = currRoute + [direction]
        newState = State(newCoordinate, newRoute)
        newCost = heuristic(newCoordinate, problem) + problem.getCostOfActions(newRoute) #The total cost is that of the cost of the action and the
                                                                                         #cost provided by the heuristic function
        fringeList.push(newState, newCost) #Push the node to explore later (similar to the UCS search)
  return []
    
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

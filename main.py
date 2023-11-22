import gurobipy as gp
from gurobipy import GRB
import sys
import time
import random
import math


"""
Part 1:
The base optimiser is:

min: sum(length(edge)*x(edge) for every edge in E)
beholdend to:
x(d(v)) = 2 for every v in V
x(edge) >= 0 for every edge in E

we take the source = vertex 0
After solving the base problem, we check if it is a valid solution by checking the maxflow:

for sink in V/{source}:
    construct a network from source to sink with the capacities of the arc uv = x(uv)
    perform the Ford Fulkerson algorithm to find the max flow
    if the max flow < 2:
        with the residual network is U, now d(U) is the min cut
        add the constraint d(U)>=2 to the problem, solve it and check it again

if no networks are found where the max flow is less than 2, the solution is feasible and we are done


Our Ford Fulkerson algorithm goes as follows:

initialize max flow to 0
while a path can be found (using depth first search):
    x = the found path
    add the max flow along the path, to the total maxflow
    decrease the capacity of the path by the max flow along the path
    increase the capacity of the path in reverse by the max flow along the path
    if the total maxflow >= 2:
        return the total maxflow
if no more path can be found and the total maxflow is less than 2Y
    return the total maxflow and residual network
"""



edges = []
flows = []
n = 0
A =[[]]
nodesInCut = []
task2 = False
task3 = True


def readDat(filename):
    with open(filename) as datFile:
        global n
        x = [data.split() for data in datFile]
        x2 = [[int(y) for y in lst] for lst in x]
        n = int(x2[0][0])
        A = x2[:][1:]
        return n, A


def Model(n, A):
    # Create a new model
    global edges
    env = gp.Env(params={"OutputFlag":0})
    m = gp.Model("Subtour", env=env)

    # Create variables
    # n = 318 #aanpassen naar eerste getal in .dat --> nu linhp218.dat
    edges = m.addVars(n, n, vtype="C", name="x")

    for i in range(n):
        for j in range(n):
            if A[i][j] == 0:
                m.addConstr((edges[i, j] == 0), name="donotuseedgeswithoutweight")
            else:
                m.addConstr((edges[i, j] >= 0), name="positiveweight")
                # m.addConstr((edges[i, j] <= 1), name="positiveweight")

    m.addConstrs((edges[i, j] - edges[j, i] == 0 for i in range(n) for j in range(i + 1, n)), name="symmetric")

    m.addConstrs((gp.quicksum(edges[v, j] for j in range(n) if j != v) == 2 for v in range(n)))

    # Set objective
    m.setObjective((gp.quicksum(edges[i, j] * A[i][j] for i in range(n) for j in range(n))) / 2, GRB.MINIMIZE)
    return m


def Model_extended(n, A):
    # Create a new model
    global edges, flows

    env = gp.Env(params={"OutputFlag": 0})
    m = gp.Model("Extended", env=env)
    # m = gp.Model("Extended")
    nodelist = [*range(n)]
    slist = [*range(1, n)]
    flows = m.addVars(slist, nodelist, nodelist, vtype="C", name="Flows")
    edges = m.addVars(n, n, vtype="C", name="x")  # Bayes29: 2020 I 2013.5 C

    for i in range(n):
        for j in range(n):
            if A[i][j] == 0:
                m.addConstr((edges[i, j] == 0), name="donotuseedgeswithoutweight")
            else:
                m.addConstr((edges[i, j] >= 0), name="positiveweight")

    m.addConstrs((edges[i, j] - edges[j, i] == 0 for i in range(n) for j in range(i + 1, n)), name="symmetric")

    m.addConstrs(((flows[s, i, j] >= 0) for s in range(1, n) for i in range(n) for j in range(n)),
                 name="positive flows")

    m.addConstrs((gp.quicksum(edges[v, j] for j in range(n) if j != v) == 2 for v in range(n)), name="two edges")

    m.addConstrs(((-flows.sum(s, '*', 0) + flows.sum(s, 0, '*')) >= 2 for s in range(1, n)), name="Outflow start")

    m.addConstrs(((flows.sum(s, '*', k) == flows.sum(s, k, '*')) for s in range(1, n) for k in range(1, n) if s != k),
                 name="Inflow is outflow")

    m.addConstrs((flows[s, i, j] <= edges[i, j] for s in range(1, n) for i in range(n) for j in range(n)))

    m.setObjective((gp.quicksum(edges[i, j] * A[i][j] for i in range(n) for j in range(n))) / 2, GRB.MINIMIZE)
    return m


def Model_extended_Integer(n, A):
    # Create a new model
    global edges, flows

    env = gp.Env(params={"OutputFlag": 0})
    m = gp.Model("Extended Integer", env=env)
    # m = gp.Model("Extended")
    nodelist = [*range(n)]
    slist = [*range(1, n)]
    flows = m.addVars(slist, nodelist, nodelist, vtype="C", name="Flows")
    edges = m.addVars(n, n, vtype="I", name="x")  # Bayes29: 2020 I 2013.5 C

    for i in range(n):
        for j in range(n):
            if A[i][j] == 0:
                m.addConstr((edges[i, j] == 0), name="donotuseedgeswithoutweight")
            else:
                m.addConstr((edges[i, j] >= 0), name="positiveweight")

    m.addConstrs((edges[i, j] - edges[j, i] == 0 for i in range(n) for j in range(i + 1, n)), name="symmetric")

    m.addConstrs(((flows[s, i, j] >= 0) for s in range(1, n) for i in range(n) for j in range(n)),
                 name="positive flows")

    m.addConstrs((gp.quicksum(edges[v, j] for j in range(n) if j != v) == 2 for v in range(n)), name="two edges")

    m.addConstrs(((-flows.sum(s, '*', 0) + flows.sum(s, 0, '*')) >= 2 for s in range(1, n)), name="Outflow start")

    m.addConstrs(((flows.sum(s, '*', k) == flows.sum(s, k, '*')) for s in range(1, n) for k in range(1, n) if s != k),
                 name="Inflow is outflow")

    m.addConstrs((flows[s, i, j] <= edges[i, j] for s in range(1, n) for i in range(n) for j in range(n)))

    m.setObjective((gp.quicksum(edges[i, j] * A[i][j] for i in range(n) for j in range(n))) / 2, GRB.MINIMIZE)
    return m


# m.addConstrs((gp.quicksum(flows[s, j, k] for j in range(n) for k in rangje(n))
#                      == (gp.quicksum(flows[i, s, s] for i in range(1, n)))), name="f_in is f_out")


#     # Create variables
#     edges = m.addVars(n,n, vtype="C", name="x")
#     flows = m.addVars(n,n, vtype = "C", name="f")
#
#     for i in range(n):
#         for j in range(n):
#             if A[i][j] == 0:
#                 m.addConstr((edges[i, j] == 0), name="donotuseedgeswithoutweight")
#             else:
#                 m.addConstr((edges[i, j] >= 0), name="positiveweight")
#
#     m.addConstrs((gp.quicksum(edges[v,j] for j in range(n)) == 2 for v in range(n)))
#     m.setObjective((gp.quicksum(edges[i, j] * A[i][j] for i in range(n) for j in range(n))) / 2, GRB.MINIMIZE)
#
#     U = m.optimize() #wordt returned in de vorm van (i,j)
#     solution = m.getAttr("X", edges)
#     r=0
#
#  #voor i in U zodat (i, 1), (i, 6), (i, 9)
#     for s in range(n): #voor j in U zodat 1, 6, 9 worden gepakt van hierboven
#         if solution[i] > 0:
#             if (r,s) in solution:
#                  print((i,j))
#
# #    m.addConstrs(
# #        (flows.sum('*', i, j) <= capacities[i, j] for i, j in m), "cap")
#
# #     # Set objective
# #     m.setObjective((gp.quicksum(edges[i,j] * A[i][j] for i in range(n) for j in range(n)))/2, GRB.MINIMIZE)
# #     return m

def OptimizeAndPrint(m):
    m.optimize()
    solution = m.getAttr("X", edges)
    print("\n Solution with objective value %f" % m.ObjVal)

    for i, j in solution:
        if solution[i, j] > 0:
            print((i, j))
            print(solution[i, j])


def OptimizeAndPrintNewObj(m):
    m.optimize()
    # solution = m.getAttr("X", edges)
    print("\n Solution with objective value %f" % m.ObjVal)


def FinalPrint(m1, m2, m3):
    print("\n Value of relaxation:")
    st1 = time.time()
    m1 = CuttingPlanes(m1)
    et1 = time.time()
    elapsed_time1 = et1 - st1
    print(" - Cutting plane: %f, %fs" % (m1.ObjVal, elapsed_time1))
    st2 = time.time()
    m2.optimize()
    et2 = time.time()
    elapsed_time2 = et2 - st2
    print(" - Extended: %f, %fs" % (m2.ObjVal, elapsed_time2))
    st3 = time.time()
    m3.optimize()
    et3 = time.time()
    elapsed_time3 = et3 - st3
    print("\n Integer Optimal: %f, %fs" % (m3.ObjVal, elapsed_time3))

    #     # Add constraint: x + 2 y + 3 z <= 4
    #     m.addConstr(x + 2 * y + 3 * z <= 4, "c0")
    #
    #     # Add constraint: x + y >= 1
    #     m.addConstr(x + y >= 1, "c1")
    #
    #     # Optimize model
    #     m.optimize()
    #
    #     for v in m.getVars():
    #         print('%s %g' % (v.VarName, v.X))
    #
    #     print('Obj: %g' % m.ObjVal)
    #
    # except gp.GurobiError as e:
    #     print('Error code ' + str(e.errno) + ': ' + str(e))
    #
    # except AttributeError:
    #     print('Encountered an attribute error')
    #


def findNodesInCut(flowCapacityDict, node):
    global nodesInCut
    nodesInCut.append(node)
    for leaf in flowCapacityDict[node]:
        if leaf not in nodesInCut:
            findNodesInCut(flowCapacityDict, leaf)


def findPath(flowCapacityDict, path, sink, depthDict, depth):
    start = path[-1]
    if sink in flowCapacityDict[path[-1]]:
        path.append(sink)
        return True, path, flowCapacityDict[start][sink]
    for end in flowCapacityDict[path[-1]]:
        if end not in path:  # and depth == depthDict[end]:
            path.append(end)
            pathBool, path, maxpathflow = findPath(flowCapacityDict, path, sink, depthDict, depth + 1)
            if pathBool:
                arcFlow = flowCapacityDict[start][end]
                maxpathflow = min(arcFlow, maxpathflow)
                return pathBool, path, maxpathflow
            else:
                path.pop()
    return False, path, 0


def FordFulkerson(solution, sink):
    flowCapacityDict = {}
    for start in range(n):
        if start != sink:
            flowCapacityDict[start] = {}
            for end in range(1, n):
                if solution[start, end] > 0:
                    flowCapacityDict[start][end] = solution[start, end]
    pathBool = True
    maxflow = 0
    depthDict = findDepthDict(flowCapacityDict)
    while pathBool:
        path = [0]

        pathBool, path, maxpathflow = findPath(flowCapacityDict, path, sink, depthDict, 1)
        # print(path)
        # print(maxpathflow)
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]
            flowCapacityDict[start][end] -= maxpathflow
            if flowCapacityDict[start][end] == 0:
                del flowCapacityDict[start][end]
            if end in flowCapacityDict:
                if start in flowCapacityDict[end]:
                    flowCapacityDict[end][start] += maxpathflow
                else:
                    flowCapacityDict[end][start] = maxpathflow
            else:
                flowCapacityDict[end] = {start: maxpathflow}
        maxflow += maxpathflow
        # print(maxpathflow)
        # print(path)
        if maxflow >= 1.99999999998:
            # print("flow over requirement with maxflow " + str(maxflow))
            return True, path
    global nodesInCut
    nodesInCut = []
    findNodesInCut(flowCapacityDict, 0)
    # print(flowCapacityDict)
    # print(maxflow)
    # print("error found with sink: %i" % sink)
    # print("Min Cut is")
    # print(nodesInCut)
    return False, nodesInCut


def findDepthDict(flowCapacityDict):
    depthDict = {0: 0}
    depth = 0
    while depth < n:
        currentDict = depthDict.copy()
        for node, nodeDepth in currentDict.items():
            if nodeDepth == depth:
                if node in flowCapacityDict:
                    for end in flowCapacityDict[node]:
                        if end not in depthDict:
                            depthDict[end] = depth + 1
        depth += 1
    return depthDict


def CuttingPlanes(m):
    feasible = False
    while not feasible:
        m.optimize()
        # OptimizeAndPrintNewObj(m)
        solution = m.getAttr("X", edges)
        for sink in range(1, n):
            flowBool, nodes = FordFulkerson(solution, sink)
            if not flowBool:
                allNodes = [*range(n)]
                for u in nodes:
                    allNodes.remove(u)
                # cutEdges = [(u,v) for u in allNodes for v in nodes]
                # print(sum(solution[u,v] for u,v in cutEdges))
                m.addConstr(gp.quicksum(edges[u, v] for u in nodes for v in allNodes) >= 2,
                            name="neededCutConstraint")
                # print(sink)
                break
            if flowBool and sink == n - 1:
                feasible = True
    return m


def nearestNeighbour(n, A):
    nodesLeft = [*range(n)]
    min = 1e10
    minPair = [0,1]
    for i in nodesLeft:
        for j in nodesLeft:
            if i < j:
                if A[i][j] < min:
                    min = A[i][j]
                    minPair = [i, j]
    path = minPair
    value = min
    nodesLeft.remove(minPair[0])
    nodesLeft.remove(minPair[1])
    while len(path) < n:
        i = path[-1]
        min = 1e10
        minVert = nodesLeft[0]
        for j in nodesLeft:
            if A[i][j] < min:
                min = A[i][j]
                minVert = j
        path.append(minVert)
        value += min
        nodesLeft.remove(minVert)

    value += A[path[0]][path[-1]]
    return value, path


def evaluateFitness(pathList):
    value = A[pathList[-1]][pathList[0]]
    for v in range(len(pathList)-1):
        value += A[pathList[v]][pathList[v+1]]
    return value


def beeColonyOptimization():
    noPatches = round(math.sqrt(n))
    noBeesOptimal = noPatches
    noBeesSubOptimal = round(noBeesOptimal/5)
    patches = {}
    for i in range(noPatches):
        patch = [*range(n)]
        random.shuffle(patch)
        fitnessPatch = evaluateFitness(n, A)




##### Genetic Algorithm

class individual:
    def __init__(self) -> None:
        self.gnome = []
        self.fitness = 0


    def __str__(self):
        return f"{self.gnome} with fitness {self.fitness}"
    def __lt__(self, other):
        return self.fitness < other.fitness

    def __gt__(self, other):
        return self.fitness > other.fitness


# Function to return a random number in the path
def rand_num(start, end):
    return random.randint(start, end -1 )

def choose_parent(population_size):
    weights =[30, 20, 15, 10, 7, 5, 5, 5, 3]
    for i in range(population_size - 9):
        weights.append(1)
    return random.choices(range(population_size), weights=weights, k=1)


# Function to return a mutated gnome by swapping two random cities in the gnome
def mutatedGene(gnome):
    while True:
        r = rand_num(0, n)
        r1 = rand_num(0, n)
        if r1 != r:
            temp = gnome[r]
            gnome[r] = gnome[r1]
            gnome[r1] = temp
            break
    return gnome


# Function to return a valid gnome by appending non-visited cities until length is n
def create_gnome():
    gnome = []
    while True:
        if len(gnome) == n:
            break
        city = rand_num(0, n)
        if city not in gnome:
            gnome.append(city)
    return gnome

def cooldown(temp):
    return (90 * temp) / 100


def crossover(parent1, parent2):
    crossover_point = rand_num(0, n)
    # Initialize child with the first part from parent1
    child_gnome = parent1.gnome[:crossover_point]
    # Keeping track of cities in the child's route
    child_city_set = set(child_gnome)
    # Fill in the remaining part from parent2
    for city in parent2.gnome:
        if city not in child_city_set:
            child_gnome.append(city)
            child_city_set.add(city)

    child = individual()
    child.gnome = child_gnome
    child.fitness = evaluateFitness(child_gnome)
    return child


def Genetic_Alg(A):
    population_size = 10
    population = []
    value, path = nearestNeighbour(n, A)

    for i in range(1):
        start = individual()
        start.gnome = path
        start.fitness = evaluateFitness(start.gnome)
        population.append(start)

    # Populating the gnome pool
    for i in range(population_size - 1):
        new = individual()
        new.gnome = create_gnome()
        new.fitness = evaluateFitness(new.gnome)
        population.append(new)


    print("\nInitial population: \nGNOME     FITNESS VALUE\n")
    for i in range(population_size):
        print(population[i].gnome, population[i].fitness)
    print()

    found = False
    temperature = 1000
    gen = 1
    gen_thres = n

    while temperature > 100 and gen <= gen_thres:
        population.sort()
        print("\nCurrent temp: ", temperature)
        new_population = []
        # Perform crossover between the best two parents (by population.sorted)
        for i in range(population_size):
            j = choose_parent(population_size)[0]
            parent1 = population[j]
            k = choose_parent(population_size)[0]
            parent2 = population[k]
            child = crossover(parent1, parent2)
            new_population.append(child)
            # print(child)

        sum_fitness = 0
        population = new_population

        for i in range(population_size):
            sum_fitness += population[i].fitness
        mean_fitness = sum_fitness/population_size

        for i in range(population_size):
            p1 = population[i]

            while True:
                new_g = mutatedGene(p1.gnome)
                new_gnome = individual()
                new_gnome.gnome = new_g
                new_gnome.fitness = evaluateFitness(new_gnome.gnome)

                if new_gnome.fitness <= mean_fitness:
                    new_population.append(new_gnome)
                    break

                else:
                    # Accepting the rejected children at a possible probability above threshold.
                    prob = pow(
                        2.7,
                        -1
                        * (
                                (float)(new_gnome.fitness - mean_fitness) * 50
                                / (temperature * mean_fitness)
                        ),
                    )
                    if prob > random.random():
                        new_population.append(new_gnome)
                        break
        print(mean_fitness)
        temperature = cooldown(temperature)
        print("Generation", gen)
        print("GNOME     FITNESS VALUE")

        for i in range(population_size):
            print(population[i].gnome, population[i].fitness)
        gen += 1

    value = 100000
    for i in range(population_size):
        # print(population[i].gnome, population[i].fitness)
        if population[i].fitness < value:
            value = population[i].fitness
            optimalpath = population[i].gnome
    return value, optimalpath

def main():
    global n
    global A
    if len(sys.argv) != 2:
        n, A = readDat("att48.dat")
    else:
        n, A = readDat(sys.argv[1])
    if task2:
        m2 = Model_extended(n, A)
        m3 = Model_extended_Integer(n, A)
        # OptimizeAndPrint(m)
        m = Model(n, A)
        FinalPrint(m, m2, m3)
        # CuttingPlanes(m)
    if task3:
        st1 = time.time()
        x, p = nearestNeighbour(n, A)
        et1 = time.time()
        elapsed_time1 = et1 - st1

        print(f"Solve in {elapsed_time1} seconds, with value {x} and path {p}")
        st2 = time.time()
        value, path = Genetic_Alg(A)
        et2 = time.time()
        elapsed_time2 = et2 - st2
        print(f"Solve in {elapsed_time2} seconds, with value {value} and path {path}")
main()

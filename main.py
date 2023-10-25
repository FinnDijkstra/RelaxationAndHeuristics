import gurobipy as gp
from gurobipy import GRB
import numpy
import pandas
import matplotlib
import sys

# def print_hi(name):
#     # Use a breakpoint in the code line below to debug your script.
#     print(f'Hello, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
#
#
# # Press the green button in the gutter to run the script.
# if __name__ == '__main__':
#     print_hi('PyCharm')
#
#
# # This example formulates and solves the following simple MIP model:
# #  maximize
# #        x +   y + 2 z
# #  subject to
# #        x + 2 y + 3 z <= 4
# #        x +   y       >= 1
# #        x, y, z binary
#
#
#
# try:
#
#     # Create a new model
#     m = gp.Model("mip1")
#
#     # Create variables
#     x = m.addVar(vtype=GRB.BINARY, name="x")
#     y = m.addVar(vtype=GRB.BINARY, name="y")
#     z = m.addVar(vtype=GRB.BINARY, name="z")
#
#     # Set objective
#     m.setObjective(x + y + 2 * z, GRB.MAXIMIZE)
#
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



#m.addConstrs((x[i][j] == x[j][i] for i in range(n) for j in range(i+1,n)), name='c')
edges = []
n=0
nodesInCut = []
def readDat(filename):
    with open(filename) as datFile:
        x = [data.split() for data in datFile]
        x2 = [[int(y) for y in lst] for lst in x]
        n = int(x2[0][0])
        A = x2[:][1:]
        return n, A


def Model(n,A):
    # Create a new model
    global edges
    m = gp.Model("Subtour")

    # Create variables
    # n = 318 #aanpassen naar eerste getal in .dat --> nu linhp218.dat
    edges = m.addVars(n,n, vtype="C", name="x")

    for i in range(n):
        for j in range(n):
            if A[i][j] == 0:
                m.addConstr((edges[i,j] == 0), name="donotuseedgeswithoutweight")
            else:
                m.addConstr((edges[i,j] >= 0), name="positiveweight")
                #m.addConstr((edges[i, j] <= 1), name="positiveweight")

    m.addConstrs((edges[i,j] - edges[j,i] == 0 for i in range(n) for j in range(i+1,n)), name="symmetric")


    m.addConstrs((gp.quicksum(edges[v,j] for j in range(n) if j != v) == 2 for v in range(n)))

    # Set objective
    m.setObjective((gp.quicksum(edges[i,j] * A[i][j] for i in range(n) for j in range(n)))/2, GRB.MINIMIZE)
    return m

def OptimizeAndPrint(m):
    m.optimize()
    solution = m.getAttr("X", edges)
    print("\n Solution with objective value %f" % m.ObjVal)

    for i,j in solution:
        if solution[i,j] > 0:
            print((i,j))
            print(solution[i, j])




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
            findNodesInCut(flowCapacityDict,leaf)


def findPath(flowCapacityDict, path, sink):
    start = path[-1]
    for end in flowCapacityDict[path[-1]]:
        if end == sink:
            path.append(end)
            return True, path, flowCapacityDict[start][end]
        if end not in path:
            path.append(end)
            pathBool, path, maxpathflow = findPath(flowCapacityDict, path, sink)
            if pathBool:
                arcFlow = flowCapacityDict[start][end]
                maxpathflow = min(arcFlow,maxpathflow)
                return pathBool, path, maxpathflow
            else:
                path.pop()
    return False, path, 0




def FordFulkerson(solution, sink):
    flowCapacityDict = {}
    for start in range(n):
        if start != sink:
            flowCapacityDict[start] = {}
            for end in range(1,n):
                if solution[start,end]>0:
                    flowCapacityDict[start][end] = solution[start,end]
    pathBool = True
    maxflow = 0

    while pathBool:
        path = [0]
        pathBool, path, maxpathflow = findPath(flowCapacityDict, path, sink)
        #print(path)
        #print(maxpathflow)
        for i in range(len(path)-1):
            start = path[i]
            end = path[i+1]
            flowCapacityDict[start][end] -= maxpathflow
            if flowCapacityDict[start][end] == 0:
                del flowCapacityDict[start][end]
        maxflow += maxpathflow
        #print(maxflow)
        if maxflow >= 2:
            #print("flow over requirement with maxflow " + str(maxflow))
            return True, path
    global nodesInCut
    nodesInCut = []
    findNodesInCut(flowCapacityDict, 0)
    return False, nodesInCut


def CuttingPlanes(m):
    feasible = False
    while not feasible:
        # m.optimize()
        OptimizeAndPrint(m)
        solution = m.getAttr("X", edges)
        for sink in range(1,n):
            flowBool, nodes = FordFulkerson(solution, sink)
            if not flowBool:
                allNodes = [*range(n)]
                for u in nodes:
                    allNodes.remove(u)
                m.addConstr(gp.quicksum(edges[u,v] for u in nodes for v in allNodes if v != u) >= 2,
                                name="neededCutConstraint")
                break
            if flowBool and sink == n-1:
                feasible = True
    return m



def main():
    global n
    if len(sys.argv) != 2:
        n, A = readDat("burma14.dat")
        m = Model(n, A)
        OptimizeAndPrint(m)
        CuttingPlanes(m)
    else:
        n, A = readDat(sys.argv[1])
        m = Model(n,A)
        print(n)
        # print(A)


main()


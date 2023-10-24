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


def readDat(filename):
    with open(filename) as datFile:
        x = [data.split() for data in datFile]
        x2 = [[int(y) for y in lst] for lst in x]
        n = int(x2[0][0])
        A = x2[:][1:]
        return n, A


def Model(n,A):
    # Create a new model
    m = gp.Model("Subtour")

    # Create variables
    # n = 318 #aanpassen naar eerste getal in .dat --> nu linhp218.dat
    x = m.addVars(n,n, vtype="C", name="x")

    for i in range(n):
        for j in range(n):
            if A[i][j] == 0:
                m.addConstr((x[i,j] == 0), name="donotuseedgeswithoutweight")
            else:
                m.addConstr((x[i,j] >= 0), name="positiveweight")

    m.addConstrs((x[i,j] - x[j,i] == 0 for i in range(n) for j in range(i+1,n)), name="symmetric")


    m.addConstrs((gp.quicksum(x[v,j] for j in range(n)) == 2 for v in range(n)))

    # Set objective
    m.setObjective((gp.quicksum(x[i,j] * A[i][j] for i in range(n) for j in range(n)))/2, GRB.MINIMIZE)
    return m

def OptimizeAndPrint(m):
    m.optimize()
    solution = m.getAttr("X", x)
    print("\n Optimal basket content:")

    for i in solution:
        if solution[i] > 0:
            print(i)



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

def main():
    if len(sys.argv) != 2:
        n, A = readDat("a280.dat")
        m = Model(n, A)
    else:
        n, A = readDat(sys.argv[1])
        m = Model(n,A)
        print(n)
        #print(A)


main()


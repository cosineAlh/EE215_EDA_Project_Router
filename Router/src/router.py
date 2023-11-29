import numpy as np
from queue import Queue
import copy
import sys

file_name = sys.argv[1]

# Data path
grid_path = "../benchmark/"+file_name+".grid"
nl_path = "../benchmark/"+file_name+".nl"
output_path = "../out/bench.route"

# Read data
grid_in = open(grid_path,"r+")
grid_in = grid_in.readlines()
nl_in = open(nl_path,"r+")
nl_in = nl_in.readlines()

# Basic parameters
X_size = int(grid_in[0].split()[0])
Y_size = int(grid_in[0].split()[1])
Bend_Penalty = int(grid_in[0].split()[2])
Via_Penalty = int(grid_in[0].split()[3])
Net_num = int(nl_in[0])

grid_ = []
for line in grid_in[1:]:
    grid_.append(line.split())

grid = np.zeros((Y_size, X_size, 2),dtype=int)
for i in range(Y_size):
    grid[i,:,0] = [int(grid_[i][j]) for j in range(len(grid_[i])) if(grid_[i][j])]
    grid[i,:,1] = [int(grid_[Y_size+i][j]) for j in range(len(grid_[Y_size+i])) if(grid_[Y_size+i][j])]
grid = np.transpose(grid,(1,0,2))

nl = []
for line in nl_in[1:]:
    nl.append(line.split())

correct = 0

# --------------------------------
def move_check(x, y, l):
    check = True
    if x>=0 and x<X_size and y>=0 and y<Y_size and l>=0 and l<2: # check boundary
        if grid[x][y][l] != -1: # check if is blocked
            check = True
        else:
            check = False
    else:
        check = False

    return check

def route(cost, net, output):
    global correct
    global grid

    # Get pins' position
    net_ID = net[0]
    layer_pin1 = int(net[1])-1
    x_pin1 = int(net[2])
    y_pin1 = int(net[3])
    layer_pin2 = int(net[4])-1
    x_pin2 = int(net[5])
    y_pin2 = int(net[6])

    start = (x_pin1, y_pin1, layer_pin1) # start pin
    grid[x_pin1][y_pin1][layer_pin1] = 1
    cost[x_pin1][y_pin1][layer_pin1] = 1
    end = (x_pin2, y_pin2, layer_pin2) # end pin
    grid[x_pin2][y_pin2][layer_pin2] = 1
    cost[x_pin2][y_pin2][layer_pin2] = 1
    Q = Queue()

    # Search order
    offset = [(0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1)] # up, right, down, left, above, bottom

    # Initial
    x, y, l = start
    cost[x][y][l] = cost[x][y][l] + 1 # initial cost
    tmp_x = 0
    tmp_y = 0
    tmp_l = 0

    # BFS
    while Q:
        # Search around
        cost_tmp = {}
        for i in range(6):
            tmp_x = x + offset[i][0]
            tmp_y = y + offset[i][1]
            tmp_l = l + offset[i][2]
            tmp = (tmp_x, tmp_y, tmp_l)
            if move_check(tmp_x, tmp_y, tmp_l) and (cost[tmp_x][tmp_y][tmp_l] == grid[tmp_x][tmp_y][tmp_l]): # if net is not marked
                cost_tmp[tmp] = cost[x][y][l] + cost[tmp_x][tmp_y][tmp_l]

        # Sort the cells by cost
        cost_tmp = sorted(cost_tmp.items(), key = lambda x:x[1])

        # Put the cells into the queue by cheapest order
        for i in range(len(cost_tmp)):
            tmp_x, tmp_y, tmp_l = cost_tmp[i][0]
            cost[tmp_x][tmp_y][tmp_l] = cost_tmp[i][1]
            if (tmp_x, tmp_y, tmp_l) == end: # if finish routing
                break
            Q.put((tmp_x, tmp_y, tmp_l))

        # Finish routing this net
        if (tmp_x, tmp_y, tmp_l) == end:
            print('Net '+net_ID+' Routed!')
            correct += 1
            break
        
        # Empty case
        if Q.empty():
            print('Net '+net_ID+' Faild!')
            break

        # Next point
        x, y, l = Q.get()
    
    # Backtrace
    path = []
    search_start = cost[x_pin2][y_pin2][layer_pin2]
    search_end = cost[x_pin1][y_pin1][layer_pin1]
    x, y, l = end
    grid[x][y][l] = -1

    i = search_start-1
    while i > search_end-1:
        path.append((x, y, l))
        for j in range(6):
            tmp_x = x + offset[j][0]
            tmp_y = y + offset[j][1]
            tmp_l = l + offset[j][2]
            if move_check(tmp_x, tmp_y, tmp_l):
                if cost[tmp_x][tmp_y][tmp_l] == i: # untill find the last point
                    i = i - grid[tmp_x][tmp_y][tmp_l] + 1 # backtrace cost
                    grid[x][y][l] = -1
                    break
        x = tmp_x
        y = tmp_y
        l = tmp_l
        i -= 1

    if Q.empty():
        output.append('nan') # no routing
    else: 
        path.append(start) # append start pin position
        output.append(path) # output the whole path

def calc_cost(cost, net_ID, path):
    tot_cost = 0
    bend_num = 0
    via_num = 0

    if path == 'nan':
        tot_cost += 0
        #print('Net '+str(net_ID+1)+' cost: '+str(tot_cost))
    else:
        # Initial cost
        tot_cost = cost[path[0][0]][path[0][1]][path[0][2]] - cost[path[-1][0]][path[-1][1]][path[-1][2]] +1

        # Check if has bends or via
        for i in range(1, len(path)-1):
            x0, y0, l0 = path[i-1]
            x1, y1, l1 = path[i]
            x2, y2, l2 = path[i+1]
            dx1 = x0 - x1
            dy1 = y0 - y1
            dx2 = x1 - x2
            dy2 = y1 - y2
            if (dx1*dy2 - dy1*dx2) != 0:
                bend_num += 1

            if l0 != l1:
                via_num += 1
        
        if path[-1][2] != path[-2][2]:
            via_num += 1

        # Total cost
        tot_cost += bend_num*Bend_Penalty + via_num*Via_Penalty
        #print('Net '+str(net_ID+1)+' cost: '+str(tot_cost))

    return tot_cost

def write_file(output):
    with open(output_path, "w") as f:
        f.write(str(Net_num))
        f.write("\n")
        for i in range(0, Net_num):
            f.write(str(i+1))
            f.write("\n")
            if output[i] == 'nan':
                pass
            else:
                for j in range(0, len(output[i])):
                    x, y, l = output[i][len(output[i])-1-j]
                    if j > 0:
                        if l != output[i][len(output[i])-1-j+1][2]:
                            s = str(3) + " " + str(x) + " " + str(y) + " \n" # 3 + x + y
                            f.write(s)
                    s = str(l+1) + " " + str(x) + " " + str(y) + " \n" # layer + x + y
                    f.write(s)
            f.write("0\n")

if __name__ == '__main__':
    # Route
    output = []
    tot_cost = 0
    for i in range(Net_num):
        cost = copy.deepcopy(grid)
        route(cost, nl[i], output)
        tot_cost += calc_cost(cost, i, output[i])

    # Write output
    write_file(output)

    # Accuracy
    print("Accuracy: "+str(correct)+"/"+str(Net_num))
    print("Total Cost: "+str(tot_cost))

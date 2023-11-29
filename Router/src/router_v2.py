import numpy as np
import heapq
import sys

file_name = sys.argv[1]
case = int(sys.argv[2])

# Data path
grid_path = "../benchmark/"+file_name+".grid"
nl_path = "../benchmark/"+file_name+".nl"
output_path = "../out/"+file_name+".route"

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

# Sort by distance
nets = np.zeros((Net_num,7),dtype=int)
dis = np.zeros((Net_num,1),dtype=int)
x_nets = []
y_nets = []
l_nets = []
for i in range(Net_num):
    nets[i] = [int(nl[i][j]) for j in range(len(nl[i])) if(nl[i][j])]
    dis[i] = abs(nets[i][2]-nets[i][5]) + abs(nets[i][3]-nets[i][6])
    if (nets[i][2] == nets[i][5]):
        y_nets.append(np.hstack([nets[i],dis[i]]))
    elif (nets[i][3] == nets[i][6]):
        x_nets.append(np.hstack([nets[i],dis[i]]))
    else:
        l_nets.append(np.hstack([nets[i],dis[i]]))

x_nets = sorted(x_nets, key = lambda x:x[7])
y_nets = sorted(y_nets, key = lambda x:x[7])
l_nets = sorted(l_nets, key = lambda x:x[7])
nets = x_nets+y_nets+l_nets

nets = np.array(nets)

def route(net, grid, redo):
    # get pins' position
    net_ID = net[0]
    layer_pin1 = net[1]
    x_pin1 = net[2]
    y_pin1 = net[3]
    layer_pin2 = net[4]
    x_pin2 = net[5]
    y_pin2 = net[6]

    # start pin
    # pathcost layer x y
    start = (1, layer_pin1, x_pin1, y_pin1)
    grid[x_pin1, y_pin1, layer_pin1-1] = 1
    
    # end pin
    end = (0, layer_pin2, x_pin2, y_pin2)
    grid[x_pin2, y_pin2, layer_pin2-1] = 1
    
    # Wavefront, initialized with start
    wavefront = []
    heapq.heappush(wavefront,start)
    curr = start

    # predecessor 
    pred = np.zeros((np.size(grid,0),np.size(grid,1),np.size(grid,2)),dtype=int)

    # offset order: 1:left 2:right 3:up 4:down 5:above 6:bottom 0:null
    offset = {1:[0,0,1], 2:[0,0,-1], 3:[0,1,0], 4:[0,-1,0], 5:[-1,0,0], 6:[1,0,0]}

    while(curr[1:] != end[1:]):
        # failed case
        if(wavefront == []):
            return 0, 0
        else:
            # next point
            curr = heapq.heappop(wavefront)

            # finish routing this net
            if(curr[1:] == end[1:]):
                path = []
                cost = 0
                # until find the start point
                while(curr[1:] != start[1:]):
                    path.insert(0, (curr[1], curr[2], curr[3]))
                    cost += curr[0]
                    del_l, del_x, del_y = offset[pred[curr[2], curr[3], curr[1]-1]]
                    curr = (0, curr[1]-del_l, curr[2]-del_x, curr[3]-del_y)
                path.insert(0, start[1:])
                return path, cost
            else:
                l, x, y = curr[1]-1, curr[2], curr[3]
                grid[x, y, l] = -1
                
                # first layer: major y
                if(l == 0):
                    if(grid[x, y, l+1] != -1):
                        heapq.heappush(wavefront, (curr[0]+Via_Penalty+grid[x,y,l+1], l+2, x, y))
                        grid[x, y, l+1] = -1
                        pred[x, y, l+1] = 6
                    if(y>0 and (grid[x, y-1, l]!=-1)):
                        if((pred[x, y, l]==3)|(pred[x, y, l]==4)):
                            heapq.heappush(wavefront, (curr[0]+grid[x,y-1,l]+Bend_Penalty, l+1, x, y-1))
                        else:
                            heapq.heappush(wavefront, (curr[0]+grid[x,y-1,l], l+1, x, y-1))
                        grid[x, y-1, l] = -1
                        pred[x, y-1, l] = 2
                    if(y<Y_size-1 and (grid[x, y+1, l]!=-1)):
                        if((pred[x, y, l]==3)|(pred[x, y, l]==4)):
                            heapq.heappush(wavefront, (curr[0]+grid[x,y+1,l]+Bend_Penalty, l+1, x, y+1))
                        else:
                            heapq.heappush(wavefront, (curr[0]+grid[x,y+1,l], l+1, x, y+1))
                        grid[x, y+1, l] = -1
                        pred[x, y+1, l] = 1

                    if redo:
                        if(x<X_size-1 and (grid[x+1, y, l]!=-1)):
                            if((pred[x, y, l]==1)|(pred[x, y, l]==2)):
                                heapq.heappush(wavefront, (curr[0]+grid[x+1,y,l]+Bend_Penalty, l+1, x+1, y))
                            else:
                                heapq.heappush(wavefront, (curr[0]+grid[x+1,y,l], l+1, x+1, y))
                            grid[x+1, y, l] = -1
                            pred[x+1, y, l] = 3
                        if(x > 0 and (grid[x-1, y, l]!=-1)):
                            if((pred[x, y, l]==1)|(pred[x, y, l]==2)):
                                heapq.heappush(wavefront, (curr[0]+grid[x-1,y,l]+Bend_Penalty, l+1, x-1, y))
                            else:
                                heapq.heappush(wavefront, (curr[0]+grid[x-1,y,l], l+1, x-1, y))
                            grid[x-1, y, l] = -1
                            pred[x-1, y, l] = 4     

                # second layer: major x
                if(l == 1):
                    if(grid[x, y, l-1] != -1):
                        heapq.heappush(wavefront, (curr[0]+Via_Penalty+grid[x,y,l-1], l, x, y))
                        grid[x, y, l-1] = -1
                        pred[x, y, l-1] = 5
                    if(x<X_size-1 and (grid[x+1, y, l]!=-1)):
                        if((pred[x, y, l]==1)|(pred[x, y, l]==2)):
                            heapq.heappush(wavefront, (curr[0]+grid[x+1,y,l]+Bend_Penalty, l+1, x+1, y))
                        else:
                            heapq.heappush(wavefront, (curr[0]+grid[x+1,y,l], l+1, x+1, y))
                        grid[x+1, y, l] = -1
                        pred[x+1, y, l] = 3
                    if(x > 0 and (grid[x-1, y, l]!=-1)):
                        if((pred[x, y, l]==1)|(pred[x, y, l]==2)):
                            heapq.heappush(wavefront, (curr[0]+grid[x-1,y,l]+Bend_Penalty, l+1, x-1, y))
                        else:
                            heapq.heappush(wavefront, (curr[0]+grid[x-1,y,l], l+1, x-1, y))
                        grid[x-1, y, l] = -1
                        pred[x-1, y, l] = 4      

                    if redo:
                        if(y>0 and (grid[x, y-1, l]!=-1)):
                            if((pred[x, y, l]==3)|(pred[x, y, l]==4)):
                                heapq.heappush(wavefront, (curr[0]+grid[x,y-1,l]+Bend_Penalty, l+1, x, y-1))
                            else:
                                heapq.heappush(wavefront, (curr[0]+grid[x,y-1,l], l+1, x, y-1))
                            grid[x, y-1, l] = -1
                            pred[x, y-1, l] = 2
                        if(y<Y_size-1 and (grid[x, y+1, l]!=-1)):
                            if((pred[x, y, l]==3)|(pred[x, y, l]==4)):
                                heapq.heappush(wavefront, (curr[0]+grid[x,y+1,l]+Bend_Penalty, l+1, x, y+1))
                            else:
                                heapq.heappush(wavefront, (curr[0]+grid[x,y+1,l], l+1, x, y+1))
                            grid[x, y+1, l] = -1
                            pred[x, y+1, l] = 1
    return 0, 0

def write_file(file, path, net_id):
    for i in range(len(path)-1):
        file.write(str(path[i][0])+" "+str(path[i][1])+" "+str(path[i][2])+"\n")
        grid[path[i][1],path[i][2],path[i][0]-1] = -1

        # If via
        if(path[i][0]!=path[i+1][0]):
            file.write("3 "+str(path[i][1])+" "+str(path[i][2])+"\n")
    file.write(str(nets[net_id][4])+" "+str(nets[net_id][5])+" "+str(nets[net_id][6])+"\n")
    file.write("0 \n")

if __name__ == '__main__':
    f = open(output_path,"w")
    f.write(str(Net_num)+"\n")

    correct = 0
    tot_cost = 0
    for i in range(Net_num):
        path, cost = route(nets[i],np.copy(grid), case) # for bench1-4 "redo" should be 1

        tot_cost += cost
        #print('Net '+str(nets[i][0])+' cost: '+str(cost))
        f.write(str(nets[i][0])+"\n")

        if(path==0):
            path, cost = route(nets[i],np.copy(grid), 1) # redo
            tot_cost += cost
            #print('Net '+str(nets[i][0])+' cost: '+str(cost))
            if(path==0):
                f.write(str(0)+"\n")
                #print('Net '+str(i+1)+' Faild!')
            else:
                write_file(f, path, i)
                correct += 1
                #print('Net '+str(i+1)+' Routed!')
        else:
            write_file(f, path, i)
            correct += 1
            #print('Net '+str(i+1)+' Routed!')
            
    print("Accuracy: "+str(correct)+"/"+str(Net_num))
    print("Total Cost: "+str(tot_cost))

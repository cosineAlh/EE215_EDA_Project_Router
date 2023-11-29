import matplotlib.pyplot as plt
import numpy as np
import sys

file_name = sys.argv[1]

# Data path
grid_path = "../benchmark/"+file_name+".grid"
route_path = "../out/"+file_name+".route"

# Read data
grid_in = open(grid_path,"r+")
grid_in = grid_in.readlines()
route_in = open(route_path,"r+")
route_in = route_in.readlines()

# Basic parameters
Net_num = int(route_in[0])
X_size = int(grid_in[0].split()[0])
Y_size = int(grid_in[0].split()[1])
route = []
for line in route_in[1:]:
    route.append(line.split())

grid = []
for line in grid_in[1:]:
    grid.append(line.split())
grid = np.array(grid).astype(int)

# Show origin grid
plt.subplot(1,3,1)
plt.imshow(grid)
plt.title("origin grid")
plt.colorbar()

# Draw the route
grid[grid>1] = 1

i = 0
while i < len(route):
    if int(route[i][0]) != 0 and len(route[i])==1:
        if int(route[i+1][0]) == 2:
            layer = 1
        elif int(route[i+1][0]) == 0:
            i += 1
            continue
        else:
            layer = 0
        grid[int(route[i+1][2])+layer*Y_size][int(route[i+1][1])] = 5

        cntr = 2
        while True:
            if int(route[i+cntr][0]) == 0:
                if int(route[i+cntr-1][0]) == 2:
                    layer = 1
                else:
                    layer = 0
                grid[int(route[i+cntr-1][2])+layer*Y_size][int(route[i+cntr-1][1])] = 5
                break

            if int(route[i+cntr][0]) == 2:
                layer = 1
            else:
                layer = 0
            grid[int(route[i+cntr][2])+layer*Y_size][int(route[i+cntr][1])] = 4

            cntr += 1
        i += cntr
    i += 1

# Show the gird after routing
plt.subplot(1,3,2)
plt.imshow(grid)
plt.title("after routing")

for i in range(Y_size):
    for j in range(X_size):
        if grid[i][j] == 1:
            grid[i][j] = grid[i+Y_size][j]

# Show the overlay gird
plt.subplot(1,3,3)
plt.imshow(grid[:Y_size])
plt.title("overlay")

plt.show()

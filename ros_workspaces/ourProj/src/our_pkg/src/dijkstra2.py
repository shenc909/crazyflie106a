import numpy as np
import math
import pprint

def dijkstra(occupancy_map, start, goal):
    goal_found = False

    occupancy_map = np.transpose(occupancy_map)
    occ_map = occupancy_map.tolist()
 
    delta = [[0, -1],  # go up
             [-1, 0],  # go left
             [0, 1],  # go down
             [1, 0]]  # go right

    cost = 1

    x = start.item(0)  # startingx
    y = start.item(1)  # startingy
    goalX =goal.item(0)
    goalY = goal.item(1)
    # print "Start Pose: ", x, y
    # print "Goal Pose: ", goalX, goalY

    possible_nodes = [[0 for row in range(len(occ_map[0]))] for col in range(len(occ_map))]
    row = y
    col = x

    step_count = 0
    outermost_nodes = [(step_count, col, row)] # dist, x, y
    searched_nodes = []
    parent_node = {}  # Dictionary that Maps {child node : parent node}
    loopcount = 0

    while len(outermost_nodes) != 0:
        
        outermost_nodes.sort(reverse=True) #sort from shortest distance to farthest
        current_node = outermost_nodes.pop()
        
        if current_node[1] == goalX and current_node[2] == goalY:
            # print "Goal found!"
            goal_found = True
            
            break
        step_count, col, row = current_node

        # Check surrounding neighbors.
        for i in delta:
            possible_expansion_x = col + i[0] 
            possible_expansion_y = row + i[1] 
            inside_grid = 0 <= possible_expansion_y < len(occupancy_map[0]) and 0 <= possible_expansion_x < len(occ_map)

            if inside_grid:
                try:
                    not_searched = possible_nodes[possible_expansion_y][possible_expansion_x] == 0
                    not_occupied = occ_map[possible_expansion_y][possible_expansion_x] == 0
                    
                except:
                    not_searched = False
                    not_occupied = False
                if not_searched and not_occupied:
                    possible_nodes[possible_expansion_y][possible_expansion_x] = 3
                    possible_node = (step_count + cost, possible_expansion_x, possible_expansion_y)
                    outermost_nodes.append(possible_node)


                    # This now builds parent/child relationship
                    parent_node[possible_node] = current_node
        loopcount = loopcount+1

    if goal_found == True:

        # print "Generating path..."

        route = []
        child_node = current_node
        while parent_node.has_key(child_node):
            route.append(parent_node[child_node])
            child_node = parent_node[child_node]
            route.sort()

        path = []
        position = [start.item(0), start.item(1)]  # Starting point passed in by function
        path.append(position)  # Add it to the list for the path

        for i in range(0, len(route)):
            position = [round((route[i][1])*1, 3), round((route[i][2])*1, 3)]
            path.append(position)

        position = [goal.item(0), goal.item(1)]
        path.append(position)

        # print "Path: "
        # pprint.pprint(path)

        path = np.array(path)
        return path

    else:
        return False

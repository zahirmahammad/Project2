# Github -> https://github.com/zahirmahammad/Project2.git
import numpy as np
import pygame
import heapq
import time

# ------------------Obstacle Space Creation-----------------------------------

# Initializing Color
bcolor = (20,90,50)  
color = (156,74,0)

bloat=5

# Define the center and size of the hexagon
center_x = 650
center_y = 250
size = 150          # Side length of the hexagon

center_x = 650
center_y = 250      # Center of Hexagon
size = 150          # Side length of the hexagon

# Calculate the vertices of the hexagon with a 90-degree rotation
num_sides = 6
angle = 2 * np.pi / num_sides
rotation_angle = np.pi / 2          # 90 degrees in radians
vertices = []
bloat_vertices = []
# find vertices of the hexagon
for i in range(num_sides):
    rotated_angle = angle * i + rotation_angle
    cx = center_x + size * np.cos(rotated_angle)
    cy = center_y + size * np.sin(rotated_angle)
    bloatx = center_x + (size+bloat) * np.cos(rotated_angle)
    bloaty = center_y + (size+bloat) * np.sin(rotated_angle)
    vertices.append((cx, cy))
    bloat_vertices.append((bloatx, bloaty))

# Define a function to calculate the equation of a line given two points
def line_equation(p1, p2):
    # Calculate slope (m)
    if round(p1[0]) != round(p2[0]):
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        # y = mx + b -> b = y - mx
        b = p1[1] - m * p1[0]
        return round(m, 2), round(b, 2)
    else:
        # If the line is vertical, return None for slope and x-intercept
        return None, p1[0]

# Calculate the equations of the lines of the hexagon
line_equations = []
for i in range(num_sides):
    p1 = bloat_vertices[i]
    p2 = bloat_vertices[(i + 1) % num_sides]  # Wrap around to the first vertex for the last side
    m, b = line_equation(p1, p2)
    line_equations.append((m, b))


def InsideObstacle(location):
    x, y = location
    # hexagon check with bloat
    f1 = y - line_equations[0][0]*x-line_equations[0][1]
    f2 = x - line_equations[1][1]
    f3 = y - line_equations[2][0]*x-line_equations[2][1]
    f4 = y - line_equations[3][0]*x-line_equations[3][1]
    f5 = x - line_equations[4][1]
    f6 = y - line_equations[5][0]*x-line_equations[5][1]
    if f1<0 and f2>0 and f3>0 and f4>0 and f5<0 and f6<0:
        return True

    # Check walls with bloat
    if x<0+bloat or x>1200-bloat or y<0+bloat or y>500-bloat:
        return True
    # First Rectangle Check
    if x>100-bloat and x<175+bloat and y>0-bloat and y<400+bloat:
        return True
    # Second Rectangle Check with bloat
    if x>275-bloat and x<350+bloat and y>100-bloat and y<500+bloat:
        return True
    # Reverse C Shape Check with bloat
    if x>900-bloat and x<1020 and y>50-bloat and y<125+bloat:
        return True
    if x>1020-bloat and x<1100+bloat and y>50-bloat and y<450+bloat:
        return True
    if x>900-bloat and x<1020 and y>375+bloat and y<450+bloat:
        return True
         
    else:
        return False 



def create_obstacle_space():
    obstacle_space = np.zeros((1200, 500))
    for i in range(obstacle_space.shape[0]):
        for j in range(obstacle_space.shape[1]):
            obstacle_space[i, j] = 0 if not InsideObstacle((i, j)) else -1
    return obstacle_space

# ------------------Building Environment-----------------------------------

# --> Perform Up Action given pose and cost
def Up(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]
        y = pose[1]-1
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

# --> Perform Down Action given pose and cost
def Down(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]
        y = pose[1]+1
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

# --> Perform Left Action given pose and cost
def Left(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]-1
        y = pose[1]
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

# --> Perform Right Action given pose and cost
def Right(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]+1
        y = pose[1]
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

# --> Perform TopLeft Action given pose and cost
def TopLeft(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]-1
        y = pose[1]-1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None

# --> Perform TopRight Action given pose and cost
def TopRight(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]+1
        y = pose[1]-1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None

# --> Perform BottomLeft Action given pose and cost
def BottomLeft(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]-1
        y = pose[1]+1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None

# --> Perform BottomRight Action given pose and cost
def BottomRight(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]+1
        y = pose[1]+1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None


# Dijkstra Algorithm
def dijkstra(start, goal):
    open_list = []                      # open list
    closed_list=[]                      # closed list
    check_closed = set()                # closed set to check if node in closed list

    open_list.append([0, start, start])    

    while True:
        if len(open_list)!=0:
            new_node = heapq.heappop(open_list)     # heappop

            closed_list.append(new_node)            # add to closedlist
            check_closed.add(new_node[1])           # add to check_closed

            if new_node[1] == goal:                 # check if its the goal node
                print("BackTracking..!")            
                return (closed_list, new_node)      # return closed_list and goal_node
            else:
                actions =  [Up(new_node[1], new_node[0]),       # perform all actions and add them to list
                            Down(new_node[1], new_node[0]),
                            Right(new_node[1], new_node[0]),
                            Left(new_node[1], new_node[0]),
                            TopLeft(new_node[1], new_node[0]),
                            TopRight(new_node[1], new_node[0]),
                            BottomLeft(new_node[1], new_node[0]),
                            BottomRight(new_node[1], new_node[0])] 
                for action in actions:
                    if action is not None:                  # check if action not None
                        x, y = action[1]
                        if obstacle_space[x, y] == 0 and action[1] not in check_closed:     # check if state is not in obstacle and closed_list
                            if len(open_list)!=0:
                                arr = np.array(open_list, dtype=object)
                                if action[1] in list(arr[:,1]):                             # if state in action list
                                    row_index = list(arr[:, 1]).index(action[1])        
                                    if open_list[row_index][0] > action[0]:                 # if current cost is less, update the cost in openlist
                                        open_list[row_index][0] = action[0]
                                        open_list[row_index][2] = new_node[1]
                                    continue
                            open_list.append([action[0], action[1], new_node[1]])          # else, append to the open_list
        else:
            print("No path Found!")
            break

# --Backtrack using closed_list
def backtrack(closed_list, goal_node):
    track = []
    curr_node = goal_node           # set goal_node as current_node
    # heapq.heapify(closed_list)  
    while curr_node[1]!=start:              # until current_node is start_node
        for node in closed_list:        
            if curr_node[2] == node[1]:     # find the parent_node
                track.append(node)
                break
        curr_node = track[-1]       # set parent_node as curr_node

    return track[::-1]              # reverse the list and return



# ---Dijkstra Algorithm Implement--------
obstacle_space = create_obstacle_space()
print("\n-------------------------------------------")
print("Dijkstra -> Environment Size: ", obstacle_space.shape)
print("-------------------------------------------")


r_color = (255,0,255)
e_color = (0,255,0)

# -- User Input----#
while True:
    print("\n##----------Start-Node----------##")
    startx = int(input("Start Point - Enter x: "))
    starty = int(input("Start Point - Enter y: "))
    try:
        if obstacle_space[startx, starty] == -1:
            print("Start pose - in Obstacle Space!")
            print("Please select a pose that is not in Obstacle Space")
        else:
            start = (startx, 499-starty)
            break
    except:
        print("Enter values given in the range")
while True:
    print("\n##----------Goal-Node----------##")
    endx = int(input("End Point - Enter x: "))
    endy = int(input("End Point - Enter y: "))
    try:
        if obstacle_space[endx, endy] == -1:
            print("End pose - in Obstacle Space!")
            print("Please select a pose that is not in Obstacle Space")
        else:
            end = (endx, 499-endy)
            break
    except:
        print("Enter values given in the range")

print("-------------------------------------------")
print("Djikstra Loading....")

# ---Calculate time while implementing dijkstra and tracking---
s_time = time.time()
closed_list, goal_node = dijkstra(start, end)
track = backtrack(closed_list, goal_node)
e_time = time.time()

print("Time taken: ",e_time-s_time," seconds")



# ------------------------------Draw Environment---------------------
pygame.init()

# Initializing env
env = pygame.display.set_mode((1200,500))

env.fill(bcolor)

# ---Draw obstacle space---
for i in range(obstacle_space.shape[0]):
    for j in range(obstacle_space.shape[1]):
        if obstacle_space[i,j]==-1:
            env.set_at((i, j), color)
    if i%100==0:
        pygame.display.flip()
        pygame.time.Clock().tick(60)        


update_interval = 500  # Update display after processing every 500 nodes

# ---Draw the end point---
pygame.draw.circle(env, e_color, end, 3)

# -- Draw states Explored---
for i, node in enumerate(closed_list):
    env.set_at(node[1], (10, 13, 235))
    if i % update_interval == 0:
        pygame.display.flip()  # Update display periodically
        pygame.time.Clock().tick(60)
pygame.display.flip()

# ---Draw start point---
pygame.draw.circle(env, r_color, start, 3)
pygame.display.flip()

# ---Draw the path-----
for i, node in enumerate(track):
    env.set_at(node[1], (0,255,0))
    # if i % 100 == 0:
    pygame.display.flip()
    pygame.time.Clock().tick(60)

# -- Hold the window untill closed--- 
run = True


while run:
    for event in pygame.event.get():
     if event.type ==pygame.QUIT:
        run = False
    pygame.display.update()


pygame.quit()

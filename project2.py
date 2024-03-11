import numpy as np
import pygame
import heapq
import time

# ------------------Obstacle Space Creation-----------------------------------

# Initializing Color
clearance = (255,255,0)
color = (255,0,0)

bloat=5



# Define the center and size of the hexagon
center_x = 650
center_y = 250
size = 150  # Side length of the hexagon

# Calculate the vertices of the hexagon
# Define the center and size of the hexagon
center_x = 650
center_y = 250
size = 150  # Side length of the hexagon

# Calculate the vertices of the hexagon with a 90-degree rotation
num_sides = 6
angle = 2 * np.pi / num_sides
rotation_angle = np.pi / 2  # 90 degrees in radians
vertices = []
bloat_vertices = []
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
    p1 = vertices[i]
    p2 = vertices[(i + 1) % num_sides]  # Wrap around to the first vertex for the last side
    m, b = line_equation(p1, p2)
    line_equations.append((m, b))
    # print(f"Side {i+1}: y = {m}x + {b}")

def InsideObstacle(location):
    # print(location)
    x, y = location
    # hexagon check
    f1 = y - line_equations[0][0]*x-line_equations[0][1]
    f2 = x - line_equations[1][1]
    f3 = y - line_equations[2][0]*x-line_equations[2][1]
    f4 = y - line_equations[3][0]*x-line_equations[3][1]
    f5 = x - line_equations[4][1]
    f6 = y - line_equations[5][0]*x-line_equations[5][1]
    if f1<0 and f2>0 and f3>0 and f4>0 and f5<0 and f6<0:
        return True

    # Check walls
    if x<0+bloat or x>1200-bloat or y<0+bloat or y>500-bloat:
        return True
    # First Rectangle Check
    if x>100-bloat and x<175+bloat and y>0-bloat and y<400+bloat:
        return True
    # Second Rectangle Check
    if x>275-bloat and x<350+bloat and y>100-bloat and y<500+bloat:
        return True
    # Reverse C Shape Check
    if x>900-bloat and x<1020 and y>50-bloat and y<125+bloat:
        return True
    if x>1020-bloat and x<1100+bloat and y>50-bloat and y<450+bloat:
        return True
    if x>900-bloat and x<1020 and y>375+bloat and y<450+bloat:
        return True
         
    else:
        return False 



def create_obstacle_space():
    # env_as_array = pygame.surfarray.array3d(env)
    # obstacle_space = np.copy(env_as_array)
    # obstacle_space = obstacle_space[:,:,0]
    obstacle_space = np.zeros((1200, 500))
    for i in range(obstacle_space.shape[0]):
        for j in range(obstacle_space.shape[1]):
            obstacle_space[i, j] = 0 if not InsideObstacle((i, j)) else -1
    return obstacle_space

# ------------------Building Environment-----------------------------------







# --------- Not in use ------------------
class Node():
    def __init__(self, location, parent_loc, cost) -> None:
        self.location = location
        self.parent = parent_loc
        self.c2c = cost
# --------- Not in use ------------------


def Up(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]
        y = pose[1]-1
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

def Down(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]
        y = pose[1]+1
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

def Left(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]-1
        y = pose[1]
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

def Right(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]+1
        y = pose[1]
        c2c = cost+1
        return (c2c, (x,y))
    else:
        return None

def TopLeft(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]-1
        y = pose[1]-1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None

def TopRight(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]+1
        y = pose[1]-1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None

def BottomLeft(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]-1
        y = pose[1]+1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None

def BottomRight(pose, cost):
    if 1200>pose[0]>=0 and 500>pose[1]>=0:
        x = pose[0]+1
        y = pose[1]+1
        c2c = cost+1.4
        return (c2c, (x,y))
    else:
        return None





def dijkstra(start, goal):
    open_list = []
    closed_list=[]
    check_closed = set()

    open_list.append([0, start, start])

    while True:
        if len(open_list)!=0:
            new_node = heapq.heappop(open_list)

            closed_list.append(new_node)
            check_closed.add(new_node[1])

            if new_node[1] == goal:
                print("BackTracking..!")
                return (closed_list, new_node)
            else:
                actions =  [Up(new_node[1], new_node[0]), 
                            Down(new_node[1], new_node[0]),
                            Right(new_node[1], new_node[0]),
                            Left(new_node[1], new_node[0]),
                            TopLeft(new_node[1], new_node[0]),
                            TopRight(new_node[1], new_node[0]),
                            BottomLeft(new_node[1], new_node[0]),
                            BottomRight(new_node[1], new_node[0])] 
                for action in actions:
                    if action is not None:
                        x, y = action[1]
                        # print("y: ", y)
                        if obstacle_space[x, y] == 0 and action[1] not in check_closed:
                            # print(action[1])
                            # print(open_list_array[:,1])
                            if len(open_list)!=0:
                                arr = np.array(open_list, dtype=object)
                                if action[1] in list(arr[:,1]):
                                    row_index = list(arr[:, 1]).index(action[1])
                                    if open_list[row_index][0] > action[0]:
                                        # print("index: ", row_index)
                                        open_list[row_index][0] = action[0]
                                        open_list[row_index][2] = new_node[1]
                                        # heapq.heapify(open_list)
                                    continue
                            # else:
                            open_list.append([action[0], action[1], new_node[1]])
                            # print([action[0], action[1], new_node[1]])
                            # c+=1
                            # heapq.heappush(open_list, [action[0], action[1], new_node[1]])
                # print("ACtions: ", c)

        else:
            print("No path Found!")
            break





def backtrack(closed_list, goal_node):
    track = []
    curr_node = goal_node
    heapq.heapify(closed_list)
    while curr_node[1]!=start:
        for node in closed_list:
            if curr_node[2] == node[1]:
                track.append(node)
                break
        curr_node = track[-1]

    return track[::-1]











# ---Dijkstra Algorithm Implement--------
obstacle_space = create_obstacle_space()
print("-------------------------------------------")
print("Environment Size: ", obstacle_space.shape)


r_color = (255,0,255)
e_color = (0,255,0)

while True:
    startx = int(input("Start Point - Enter x: "))
    starty = int(input("Start Point - Enter y "))
    try:
        if obstacle_space[startx, starty] == -1:
            print("Start pose is in Obstacle Space!")
            print("Please select a pose that is not in Obstacle Space")
        else:
            start = (startx, 500-starty)
            break
    except:
        print("Enter values given in the range")
while True:
    endx = int(input("End Point - Enter x: "))
    endy = int(input("End Point - Enter y: "))
    try:
        if obstacle_space[endx, endy] == -1:
            print("End pose is in Obstacle Space!")
            print("Please select a pose that is not in Obstacle Space")
        else:
            end = (endx, 500-endy)
            break
    except:
        print("Enter values given in the range")

print("-------------------------------------------")
print("Djikstra Loading....")

s_time = time.time()
closed_list, goal_node = dijkstra(start, end)
track = backtrack(closed_list, goal_node)
e_time = time.time()

print("Time taken: ",e_time-s_time," seconds")



# ------------------------------Draw Environment---------------------

pygame.init()

# Initializing env
env = pygame.display.set_mode((1200,500))

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
    env.set_at(node[1], (0, 0, 255))
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





run = True


while run:
    for event in pygame.event.get():
     if event.type ==pygame.QUIT:
        run = False
    pygame.display.update()


pygame.quit()

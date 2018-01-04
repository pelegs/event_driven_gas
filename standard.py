import numpy as np
import pygame
import sys
from itertools import chain
import colorsys
import time
import argparse

# colors
colors = {
          'white': [255, 255, 255],
          'black': [0, 0, 0],
          'red'  : [255, 0, 0],
          'green': [0, 255, 0],
          'blue' : [0, 0, 255],
         }

# no. of dimensions
nD = 2

directions = {
              'horiz': np.array([1, 0]),
              'vert' : np.array([0, 1]),
             }              

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    else:
        return v / norm

def distance(v1, v2):
    return(np.linalg.norm(v2-v1))

# Return noral vector to a given vector v
def calc_normal(v):
    return normalize(np.array([-v[1], v[0]]))

# Check if a, b, c are counter-clockwise
def ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

# Check if two line segments l1 and l2 intersect
def intersect(l1_start, l1_end, l2_start, l2_end):
    c1 = ccw(l1_start, l2_start, l2_end) != ccw(l1_end, l2_start, l2_end)
    c2 = ccw(l1_start, l1_end, l2_start) != ccw(l1_start, l1_end, l2_end)
    return c1 and c2

# Get intersection point of line segments l1 and l2
def intersection_point(l1_start, l1_end, l2_start, l2_end):
    x1, x2 = l1_start[0], l1_end[0]
    y1, y2 = l1_start[1], l1_end[1]
    x3, x4 = l2_start[0], l2_end[0]
    y3, y4 = l2_start[1], l2_end[1]
    t = ((y3-y4) * (x1-x3) + (x4-x3) * (y1-y3)) / ((x4-x3) * (y1-y2) - (x1-x2) * (y4-y3))
    return l1_start + t * (l1_end - l1_start)

def list_neighbor_indices(x, y, N):
    return [(i,j)
            for i in range(x-1, x+2)
            for j in range(y-1, y+2)
            if 0 <= i < N and 0 <= j < N]

class ball:
    cell = np.zeros(2)

    def __init__(self,
                 pos   = np.zeros(nD),
                 vel   = np.zeros(nD),
                 rad   = 1.0,
                 mass  = 1.0,
                 color = colors['black'],
                 ID    = -1,
                 grid  = None):

        self.pos    = pos.flatten()
        self.vel    = vel.flatten()
        self.rad    = rad
        self.mass   = mass 
        self.color  = color
        self.events = []
        self.ID     = ID
        if grid is not None:
            self.cell = np.array([int(np.floor(x*grid.N)) for x in self.pos])
        self.neighbors = []

    def update_neighbors(self, grid):
        cells = list(chain(*[grid.cells[i][j]
                     for (i, j) in list_neighbor_indices(self.cell[0], self.cell[1], grid.N)]))
        self.neighbors = [b for b in cells if b is not self]

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, self.pos.astype(int), self.rad)

    def move(self, dt):
        #self.vel = self.vel + np.array([0, 100]) * dt
        self.pos = self.pos + self.vel * dt
        self.cell = np.array([int(np.floor(x*grid.N))+1 for x in self.pos])

    def bounce(self, wall):
        self.vel = self.vel - 2 * np.dot(self.vel, wall.normal) * wall.normal 

    def Ek(self):
        return 0.5 * self.mass * np.dot(self.vel, self.vel)

class wall:
    def __init__(self,
                 direction = directions['horiz'],
                 start     = np.zeros(nD),
                 length    = 1,
                 width     = 5,
                 color     = colors['black'],
                 ID        = -1):

        self.direction = direction
        self.normal    = calc_normal(direction)
        self.start     = start
        self.length    = length
        self.end       = start + length * direction
        self.width     = width
        self.color     = color
        self.ID        = ID

    def draw(self, surface): 
        pygame.draw.line(surface, self.color, self.start.astype(int), self.end.astype(int), self.width)

class sim_grid:
    def __init__(self, N):
        self.cells = [[[] for _ in range(N)]
                          for _ in range(N)]
        self.N = N

    def insert(self, b, L):
        cell = np.array([int(np.floor(x/L*self.N)) for x in b.pos])
        b.cell = cell
        if (0 <= cell[0] < self.N) and (0 <= cell[1] < self.N):
            self.cells[cell[0]][cell[1]].append(b)

    def reset(self):
        self.cells = [[[] for _ in range(self.N)]
                          for _ in range(self.N)]
       
    def draw(self, surface, L):
        for i, row in enumerate(self.cells):
            for j, cell in enumerate(row):
                if cell == []:
                    color = 3*[200]
                else:
                    color = [100, 0, 0]
                pygame.draw.rect(surface, color, [i*L/self.N, j*L/self.N, L/self.N, L/self.N], 2)

def ball_collision(b1, b2):
    dr = b2.pos - b1.pos
    dist = np.linalg.norm(dr)
    overlap = b1.rad + b2.rad - dist
    if overlap > 0:
        b2.pos += normalize(dr) * overlap
    dv = b2.vel - b1.vel
    b1.vel = b1.vel - 2*b2.mass / (b1.mass + b2.mass) * np.dot(-dv, -dr)/np.dot(-dr, -dr) * -dr
    b2.vel = b2.vel - 2*b1.mass / (b1.mass + b2.mass) * np.dot(+dv, +dr)/np.dot(+dr, +dr) * +dr
    if b2 in b1.neighbors:
        b1.neighbors.remove(b2)
    if b1 in b2.neighbors:
        b2.neighbors.remove(b1)

def time_ball_collision(b1, b2):
    dr  = b2.pos - b1.pos
    dv  = b2.vel - b1.vel
    dr2 = np.dot(dr, dr)
    dv2 = np.dot(dv, dv)
    dvr = np.dot(dv, dr)
    sig = b1.rad + b2.rad
    d   = dvr**2 - dv2 * (dr2 - sig**2)
    if dvr >= 0 or d < 0:
        return float('inf')
    else:
        return -1 * (dvr + np.sqrt(d)) / dv2

def time_wall_collision(b, w):
    l1_start = b.pos
    l1_end   = b.pos + 1E10 * normalize(b.vel)
    if intersect(l1_start, l1_end, w.start, w.end):
        p = intersection_point(l1_start, l1_end, w.start, w.end)
        d = distance(b.pos, p) - b.rad
        v = np.linalg.norm(b.vel)
        if v != 0:
            return d / v
        else:
            return float('inf')
    else:
        return float('inf')

parser = argparse.ArgumentParser (description="A simple 2D gas simulation")
parser.add_argument('-i','--input_file', help='Input file', required=True)
parser.parse_args()
args = vars (parser.parse_args())
read_mode = 'particles'
with open(args['input_file']) as f:
    lines = f.readlines()
N_balls = int(lines[0][:-1].split()[0])
rad = 5
balls = []
walls = []
for i in range(1, N_balls+1):
    line = lines[i][:-1].split(' ')
    x, y, vx, vy, rad, mass = line[:6]
    color = np.array(line[6:9]).astype(int).flatten()
    balls.append(ball(pos   = np.array([float(x), float(y)]),
                      vel   = np.array([float(vx), float(vy)]),
                      rad   = int(rad),
                      mass  = float(mass),
                      color = color))
N_walls = int(lines[N_balls+1][:-1].split()[0])
for j in range(N_balls+2, N_balls+2+N_walls):
    line = lines[j][:-1].split(' ')
    sx, sy, ax, ay, L, w = line[:6]
    color = np.array(line[6:9]).astype(int).flatten()
    walls.append(wall(start     = np.array([float(sx), float(sy)]),
                      direction = np.array([float(ax), float(ay)]),
                      length    = float(L),
                      width     = int(w),
                      color     = color))
last_line = lines[-1][:-1].split(' ')
scr_size = int(last_line[0])
dt = float(last_line[1])

num_grid_cells = int(scr_size / (int(rad) * 2))
grid = sim_grid(num_grid_cells)

pygame.init()
screen = pygame.display.set_mode ((scr_size, scr_size))
big_ball_pos = []
while True:
    start_time = time.time()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                pygame.quit()
                sys.exit()
    
    screen.fill(colors['white'])
    # Insert balls into grid
    grid.reset()
    for ball in balls:
        grid.insert(ball, scr_size) 
    
    #grid.draw(screen, scr_size)
    for i, ball1 in enumerate(balls):
        ball1.update_neighbors(grid)
        for ball2 in ball1.neighbors:
            if time_ball_collision(ball1, ball2) < dt:
                ball_collision(ball1, ball2)
        for wall in walls:
            if time_wall_collision(ball1, wall) < dt:
                ball1.bounce(wall)
        ball1.move(dt)
        ball1.draw(screen)
    for wall in walls:
        wall.draw(screen)
    pygame.display.update()
    #pygame.time.delay(500)

    Ek_tot = sum([ball.Ek() for ball in balls ])
    #print('\rEk total =', Ek_tot, '              ', end='')

    elapsed_time = time.time() - start_time
    print('\r', int(1/elapsed_time), end='')

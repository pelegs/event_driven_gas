import numpy as np
import pygame
import sys
from itertools import chain

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
        pygame.draw.line(surface, self.color, self.start, self.end, self.width)          

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
        

def put_balls(balls, N, r, min_pos = 0, max_pos = 800, v_sigma = 100, color = colors['black']):
    # Enforce no overlaps
    i = 0
    while i < N:
        c = np.random.uniform(min_pos, max_pos, size=(1,nD))
        overlap = False
        for b in balls:
            if distance(c, b.pos) < 2*r:
                overlap = True
                break
        if overlap is False:
            if c.flatten()[0] < dL+L/2:
                m = 5 
                r = 10
                color = [255, 0, 0]
            else:
                m = 1
                r = 6
                color = [0, 255, 0]
            balls.append(ball(pos   = c,
                              vel   = np.random.normal(0, v_sigma, size=(1,nD)),
                              rad   = r,
                              mass  = m,
                              color = color,
                              ID    = i))
            i += 1

    # Enforce static center of mass for system
    v_com = np.zeros(nD)
    for b in balls:
        v_com += b.vel
    balls[0].vel -= v_com

def ball_collision(b1, b2):
    dr = b2.pos - b1.pos
    dist = np.linalg.norm(dr)
    overlap = b1.rad + b2.rad - dist
    if overlap > 0:
        b2.pos += normalize(dr) * overlap
    dv = b2.vel - b1.vel
    #print('Before:', b1.Ek(), b2.Ek(), b1.Ek()+b2.Ek(), end='')
    b1.vel = b1.vel - 2*b2.mass / (b1.mass + b2.mass) * np.dot(-dv, -dr)/np.dot(-dr, -dr) * -dr
    b2.vel = b2.vel - 2*b1.mass / (b1.mass + b2.mass) * np.dot(+dv, +dr)/np.dot(+dr, +dr) * +dr
    #print(', after:', b1.Ek(), b2.Ek(), b1.Ek()+b2.Ek())
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

scr_size = 800

dt = 0.01

L = 600
dL = 100
num_grid_cells = 35
grid = sim_grid(num_grid_cells)

N_balls = 100
balls = []
put_balls(balls,
          N       = N_balls,
          r       = 5,
          min_pos = dL+25,
          max_pos = L+dL,
          v_sigma = 250,
          color   = colors['red'])

wall0 = wall(start  = np.array([dL, dL]),
             length = L,
             width  = 5,
             color  = colors['blue'],
             ID     = 0)

wall1 = wall(start  = np.array([dL, dL]),
             direction = directions['vert'],
             length = L,
             width  = 5,
             color  = colors['blue'],
             ID     = 1)

wall2 = wall(start  = np.array([dL, L+dL]),
             length = L,
             width  = 5,
             color  = colors['blue'],
             ID     = 2)

wall3 = wall(start  = np.array([L+dL, dL]),
             direction = directions['vert'],
             length = L,
             width  = 5,
             color  = colors['blue'],
             ID     = 3)

wall4 = wall(start     = np.array([dL+L/2, dL]),
             direction = np.array([0, 1]),
             length    = L,
             width     = 5,
             color     = colors['black'],
             ID        = 4)
             
walls = [wall0, wall1, wall2, wall3, wall4]

pygame.init()
screen = pygame.display.set_mode ((scr_size, scr_size))
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                pygame.quit()
                sys.exit()
            if event.key == pygame.K_a:
                walls.remove(wall4)
    
    screen.fill(colors['white'])
    # Insert balls into grid
    grid.reset()
    for ball in balls:
        grid.insert(ball, L) 
    
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
    print('\rEk total =', Ek_tot, '              ', end='')

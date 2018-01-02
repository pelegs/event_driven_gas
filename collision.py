import numpy as np
import pygame
import sys
from queue import PriorityQueue

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

class ball:
    def __init__(self,
                 pos   = np.zeros(nD),
                 vel   = np.zeros(nD),
                 rad   = 1.0,
                 mass  = 1.0,
                 color = colors['black']):
    
        self.pos   = pos.flatten()
        self.vel   = vel.flatten()
        self.rad   = rad
        self.mass  = mass 
        self.color = color

    def draw(self, surface):
        pos_int = [int(x) for x in self.pos]
        pygame.draw.circle(surface, self.color, pos_int, self.rad)
        #pygame.draw.line(surface, colors['black'], pos_int, pos_int + self.vel, 3)

    def move(self, dt):
        self.pos += self.vel * dt

    def bounce(self, wall):
        return 1

class wall:
    def __init__(self,
                 direction = directions['horiz'],
                 start     = np.zeros(nD),
                 length    = 1,
                 width     = 5,
                 color     = colors['black']):

        self.direction = direction
        self.normal    = calc_normal(direction)
        self.start     = start
        self.length    = length
        self.end       = start + length * direction
        self.width     = width
        self.color     = color

    def draw(self, surface): 
        pygame.draw.line(surface, self.color, self.start, self.end, self.width)         

def put_balls(balls, N, r, min_pos = 0, max_pos = 800, min_v = 0, max_v = 100, color = colors['black']):
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
            balls.append(ball(pos = c,
                              vel = np.random.uniform(min_v, max_v, size=(1,nD)),
                              rad = r,
                              color = color))
            i += 1

    # Enforce static center of mass for system
    v_com = np.zeros(nD)
    for b in balls:
        v_com += b.vel
    balls[0].vel -= v_com

def ball_collision(b1, b2):
    dr = b2.pos - b1.pos
    dv = b2.vel - b1.vel
    J_size = 2 * b1.mass * b2.mass * np.dot(dv, dr) / ((b1.rad + b2.rad) * (b1.mass + b2.mass))
    J = J_size * dr / (b1.rad + b2.rad)
    b1.vel = b1.vel + J/b1.mass
    b2.vel = b2.vel - J/b2.mass

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

'''def calc_ball_collision_time(balls):
    for b1 in
'''
scr_size = 800
dt = 0.005
N_balls = 200
balls = []
'''put_balls(balls,
          N       = N_balls,
          r       = 10,
          min_pos = 115,
          max_pos = 684,
          min_v   = -40,
          max_v   = 40,
          color   = colors['red'])
'''
ball1 = ball(pos   = np.array([330.0, 300.0]),
             vel   = np.array([3, 5]),
             rad   = 30,
             mass  = 1,
             color = colors['blue'])

ball2 = ball(pos   = np.array([500.0, 500.0]),
             vel   = np.array([-3, -5]),
             rad   = 30,
             mass  = 1,
             color = colors['red'])

balls = [ball1, ball2]

wall1 = wall(direction = np.array([1, 2]),
             start     = np.array([100, 200]),
             length    = 200.0)

'''
wall1 = wall(start  = np.array([100, 100]),
             length = 600,
             width  = 5,
             color  = colors['blue'])

wall2 = wall(start  = np.array([100, 100]),
             direction = directions['vert'],
             length = 600,
             width  = 5,
             color  = colors['blue'])

wall3 = wall(start  = np.array([100, 700]),
             length = 600,
             width  = 5,
             color  = colors['blue'])

wall4 = wall(start  = np.array([700, 100]),
             direction = directions['vert'],
             length = 600,
             width  = 5,
             color  = colors['blue'])

walls = [wall1, wall2, wall3, wall4]
'''
pygame.init()
screen = pygame.display.set_mode ((scr_size, scr_size))
time = 0
t_b1_b2 = 0
t_b_w = [0, 0]
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                pygame.quit()
                sys.exit()
    
    screen.fill(colors['white'])
    for i, ball in enumerate(balls):
        ball.move(dt)
        if distance(ball1.pos, ball2.pos) <= ball1.rad + ball2.rad:
            ball_collision(ball1, ball2)
        ball.draw(screen)
        t_b_w[i] = time_wall_collision(ball, wall1)
        if t_b_w[i] <= dt:
            ball.bounce(wall1)
    t_b1_b2 = time_ball_collision(balls[0], balls[1])
    wall1.draw(screen)
    '''for wall in walls:
        wall.draw(screen)
    '''
    pygame.display.update()
    #pygame.time.delay(100)
    
    print('\r{} {} {}                  '.format(t_b1_b2, t_b_w[0], t_b_w[1]), end='')
    time += dt

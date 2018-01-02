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
                 color = colors['black'],
                 ID    = -1):

        self.pos    = pos.flatten()
        self.vel    = vel.flatten()
        self.rad    = rad
        self.mass   = mass 
        self.color  = color
        self.events = []
        self.ID     = ID

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, self.pos.astype(int), self.rad)

    def move(self, dt):
        self.pos = self.pos + self.vel * dt

    def bounce(self, wall):
        self.vel = self.vel - 2 * np.dot(self.vel, wall.normal) * wall.normal 

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
            if color == 'random':
                color = np.random.randint(100, 255, size=(1,3)).flatten()
            balls.append(ball(pos = c,
                              vel = np.random.uniform(min_v, max_v, size=(1,nD)),
                              rad = r,
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
    dv = b2.vel - b1.vel
    J_size = 2 * b1.mass * b2.mass * np.dot(dv, dr) / ((b1.rad + b2.rad) * (b1.mass + b2.mass))
    J = J_size * dr / (b1.rad + b2.rad)
    b1.vel = b1.vel + J/b1.mass
    b2.vel = b2.vel - J/b2.mass

def time_ball_collision(b1, b2, time):
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
        return -1 * (dvr + np.sqrt(d)) / dv2 + time

def time_wall_collision(b, w, time):
    l1_start = b.pos
    l1_end   = b.pos + 1E10 * normalize(b.vel)
    if intersect(l1_start, l1_end, w.start, w.end):
        p = intersection_point(l1_start, l1_end, w.start, w.end)
        d = distance(b.pos, p) - b.rad
        v = np.linalg.norm(b.vel)
        if v != 0:
            return d / v + time
        else:
            return float('inf')
    else:
        return float('inf')

class event:
    def __init__(self, ball1, ball2=None, wall=None, time=0):
        self.ball1  = ball1
        self.ball2  = ball2
        self.wall   = wall
        self.active = True
        if ball2 is None:
            if wall is None:
                print('Error: No second ball nor wall provided.')
                return None
            else:
                self.time = time_wall_collision(ball1, wall, time)
        else:
            if wall is not None:
                print('Error: Second ball and wall provided.')
                return None
            else:
                self.time = time_ball_collision(ball1, ball2, time)
        
    def __lt__(self, other):
        return self.time < other.time

def create_events(i, ball1, balls, walls, queue, current_time):
    for e in ball1.events:
        e.active = False
    ball1.events = []
    for j, ball2 in enumerate(balls):
        if i < j < len(balls):
            e = event(ball1, ball2, wall=None, time=current_time)
            queue.put(e)
            ball1.events.append(e)
    
    for wall in walls:
        e = event(ball1, None, wall, current_time)
        queue.put(e)
        ball1.events.append(e)

scr_size = 800
dt = 0.05
N_balls = 10
balls = []
put_balls(balls,
          N       = N_balls,
          r       = 10,
          min_pos = 150,
          max_pos = 600,
          min_v   = -40,
          max_v   = 40,
          color   = 'random')

wall0 = wall(start  = np.array([100, 100]),
             length = 600,
             width  = 5,
             color  = colors['blue'],
             ID     = 0)

wall1 = wall(start  = np.array([100, 100]),
             direction = directions['vert'],
             length = 600,
             width  = 5,
             color  = colors['blue'],
             ID     = 1)

wall2 = wall(start  = np.array([100, 700]),
             length = 600,
             width  = 5,
             color  = colors['blue'],
             ID     = 2)

wall3 = wall(start  = np.array([700, 100]),
             direction = directions['vert'],
             length = 600,
             width  = 5,
             color  = colors['blue'],
             ID     = 3)

walls = [wall0, wall1, wall2, wall3]

current_time = 0
delta_time = 0
event_queue = PriorityQueue()
for i, ball1 in enumerate(balls):
    create_events(i, ball1, balls, walls, event_queue, current_time)

pygame.init()
screen = pygame.display.set_mode ((scr_size, scr_size))
while True:
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_q:
                pygame.quit()
                sys.exit()
    
    if not event_queue.empty():
        next_event = event_queue.get()
        if next_event.time > current_time and next_event.active:
            print(current_time, next_event.time)
            delta_time = next_event.time - current_time
            current_time = next_event.time
            ball1 = next_event.ball1
            ball2 = next_event.ball2
            wall  = next_event.wall
            
            #a, b = -1, -1
            #if ball2 is not None:
            #    a = ball2.ID
            #if wall is not None:
            #    b = wall.ID
            #print(ball1.ID, a, b, delta_time, current_time)
            
            if ball2 is not None:
                ball_collision(ball1, ball2)
                create_events(ball2.ID, ball2, balls, walls, event_queue, current_time)
            if wall is not None:
                ball1.bounce(wall)
            create_events(ball1.ID, ball1, balls, walls, event_queue, current_time)
        else:
            print('passed.')

    screen.fill(colors['white'])
    for ball in balls:
        ball.move(delta_time)
        ball.draw(screen)
    for wall in walls:
        wall.draw(screen)
    pygame.display.update()
    pygame.time.delay(500)

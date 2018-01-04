import numpy as np

def distance(v1, v2):
    return(np.linalg.norm(v2-v1))

class ball:
    def __init__(self,
                 pos = np.zeros(2),
                 vel = np.zeros(2),
                 rad = 1.0,
                 mass = 1.0,
                 color = [0, 0, 0],
                 ID    = -1):
        self.pos   = pos.flatten()
        self.vel   = vel.flatten()
        self.rad   = rad
        self.mass  = mass
        self.color = color
        self.ID    = ID

    def show(self):
        print(self.pos[0], ' ',
              self.pos[1], ' ',
              self.vel[0], ' ',
              self.vel[1], ' ',
              self.rad, ' ',
              self.mass, ' ',
              self.color[0], ' ',
              self.color[1], ' ',
              self.color[2], ' ')

balls = []
N = 75
r = 10
m = 1
color = [0, 0, 255]
min_pos = 50+r
max_pos = 750-r
v_sigma = 50

def normal():
    # Enforce no overlaps
    i = 0
    while i < N:
        c = np.random.uniform(min_pos, max_pos, size=(1,2)).flatten()
        overlap = False
        for b in balls:
            if distance(c, b.pos) < 2*r:
                overlap = True
                break
        if overlap is False:
            if c[0] <= 400:
                color = [255, 0, 0]
            else:
                color = [0, 0, 255]
            balls.append(ball(pos   = c,
                              vel   = np.random.normal(0, v_sigma, size=(1,2)),
                              rad   = r,
                              mass  = m,
                              color = color,
                              ID    = i))
            i += 1

    # Enforce static center of mass for system
    v_com = np.zeros(2)
    for b in balls:
        v_com += b.vel
    balls[0].vel -= v_com

def SFD():
    xs = np.arange(min_pos, max_pos, 2*r)
    for x in xs:
        vx = np.random.normal(0, v_sigma)
        balls.append(ball(pos   = np.array([x, 400]),
                          vel   = np.array([vx, 0]),
                          rad   = 10,
                          mass  = 1,
                          color = color))
                          
#SFD()
normal()
print(len(balls))
for i, ball in enumerate(balls):
    print(ball.show())

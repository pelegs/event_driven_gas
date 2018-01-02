import numpy as np
import sys
import pygame

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    else:
        return v / norm

def ccw(a, b, c):
    return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

def intersect(l1, l2):
    c1 = ccw(l1.start, l2.start, l2.end) != ccw(l1.end, l2.start, l2.end)
    c2 = ccw(l1.start, l1.end, l2.start) != ccw(l1.start, l1.end, l2.end)
    return c1 and c2

def intersection_point(l1, l2):
    x1, x2 = l1.start[0], l1.end[0]
    y1, y2 = l1.start[1], l1.end[1]
    x3, x4 = l2.start[0], l2.end[0]
    y3, y4 = l2.start[1], l2.end[1]
    t = ((y3-y4) * (x1-x3) + (x4-x3) * (y1-y3)) / ((x4-x3) * (y1-y2) - (x1-x2) * (y4-y3))
    return l1.start + t * (l1.end - l1.start)

class line_seg:
    def __init__(self, start, end, color):
        self.start = start.flatten()
        self.end   = end.flatten()
        self.color = color
        
        self.normal = np.zeros(2)
        self.norm()
        self.center = self.start + 0.5 * (self.end - self.start)

    def norm(self):
        v = self.end - self.start
        self.normal = normalize(np.array([-v[1], v[0]]))

    def draw(self, surface):
        self.norm()
        self.center = self.start + 0.5 * (self.end - self.start)
        pygame.draw.line(surface, self.color, self.start, self.end, 4)

    def reflect(self, line2):
        d = self.end - self.start
        n = line2.normal
        return d - 2 * np.dot(d, n) * n

line1 = line_seg(np.array([200, 100]), np.array([300, 400]), [255, 0, 0])
line2 = line_seg(np.array([400, 200]), np.array([100, 200]), [0, 255, 0])
lines = [line1, line2]

w, h = 500, 500
pygame.init()
screen = pygame.display.set_mode ((w, h))
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                pygame.quit()
                sys.exit()
    
    screen.fill([255, 255, 255])
    line2.end = np.array(pygame.mouse.get_pos())
    for line in lines:
        line.draw(screen)
    if intersect(line1, line2):
        p = intersection_point(line1, line2).astype(int)
        pygame.draw.circle(screen, [0, 0, 0], p, 5)
        ref = line2.reflect(line1)
        ref_size = np.linalg.norm(p - line2.start)
        ref = ref_size * normalize(ref)
        pygame.draw.line(screen, [0, 0, 255], p, p+ref, 4)

    pygame.display.update()
    #pygame.time.delay(100)

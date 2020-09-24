from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
from orientation import *
import pygame
import math
import threading


class AHRS_Visualizer (threading.Thread):
    keep_going = True
    orientation = None
    sema = threading.Semaphore()

    def stop(self):
        self.keep_going == False

    def update_orientation(self, data):
        self.sema.acquire()
        self.orientation.quat_update(data[0][0],data[0][1],data[0][2],data[1][0] * math.pi/180 ,data[1][1] * math.pi/180 ,data[1][2] * math.pi/180 ,data[2][0],data[2][1],data[2][2])
        self.sema.release()

    def draw(self,w, nx, ny, nz):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)

        glRotatef(2 * math.acos(w) * 180.00 / math.pi, -1 * nx, nz, ny)

        #Bottom (Z+)
        glBegin(GL_QUADS)   #Technically deprecated
        glColor3f(1.0, 0.5, 0.0)
        #glNormal3f(0.0, 1.0, 0.0);
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        #Top side(Z-)
        glColor3f(1.0, 0.5, 0.0)
        #glNormal3f(0.0, 0.0, 1.0);
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

    def resizewin(self,width, height):
        if width < 1:
            width = 1
        if height < 1:
            height = 1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0 * width/height, 0.1, 125.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def __init__(self, sampling_rate):
        threading.Thread.__init__(self)
        self.sampling_rate = sampling_rate

    def run(self):
        self.orientation = Orientation(0.4, (1 / self.sampling_rate),1,0,0,0)
        pygame.init()
        screen = pygame.display.set_mode((640, 480), OPENGL | DOUBLEBUF)
        pygame.display.set_caption("Orientation Visualizer")
        self.resizewin(640, 480)
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        ticks = pygame.time.get_ticks()
        while self.keep_going == True:
            event = pygame.event.poll()
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                break
            self.sema.acquire()
            [w, nx, ny, nz] = [self.orientation.q1, self.orientation.q2, self.orientation.q3, self.orientation.q4]
            self.sema.release()
            self.draw(w, nx, ny, nz)
            pygame.display.flip()
